/*
 * ARM Virtual Generic Interrupt Controller support
 *
 * Ian Campbell <ian.campbell@citrix.com>
 * Copyright (c) 2011 Citrix Systems.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ASM_ARM_VGIC_H__
#define __ASM_ARM_VGIC_H__

#include <xen/bitops.h>
#include <asm/mmio.h>

struct pending_irq
{
    /*
     * The following two states track the lifecycle of the guest irq.
     * However because we are not sure and we don't want to track
     * whether an irq added to an LR register is PENDING or ACTIVE, the
     * following states are just an approximation.
     *
     * GIC_IRQ_GUEST_QUEUED: the irq is asserted and queued for
     * injection into the guest's LRs.
     *
     * GIC_IRQ_GUEST_VISIBLE: the irq has been added to an LR register,
     * therefore the guest is aware of it. From the guest point of view
     * the irq can be pending (if the guest has not acked the irq yet)
     * or active (after acking the irq).
     *
     * In order for the state machine to be fully accurate, for level
     * interrupts, we should keep the interrupt's pending state until
     * the guest deactivates the irq. However because we are not sure
     * when that happens, we instead track whether there is an interrupt
     * queued using GIC_IRQ_GUEST_QUEUED. We clear it when we add it to
     * an LR register. We set it when we receive another interrupt
     * notification.  Therefore it is possible to set
     * GIC_IRQ_GUEST_QUEUED while the irq is GIC_IRQ_GUEST_VISIBLE. We
     * could also change the state of the guest irq in the LR register
     * from active to active and pending, but for simplicity we simply
     * inject a second irq after the guest EOIs the first one.
     *
     *
     * An additional state is used to keep track of whether the guest
     * irq is enabled at the vgicd level:
     *
     * GIC_IRQ_GUEST_ENABLED: the guest IRQ is enabled at the VGICD
     * level (GICD_ICENABLER/GICD_ISENABLER).
     *
     * GIC_IRQ_GUEST_MIGRATING: the irq is being migrated to a different
     * vcpu while it is still inflight and on an GICH_LR register on the
     * old vcpu.
     *
     */
#define GIC_IRQ_GUEST_QUEUED   0
#define GIC_IRQ_GUEST_ACTIVE   1
#define GIC_IRQ_GUEST_VISIBLE  2
#define GIC_IRQ_GUEST_ENABLED  3
#define GIC_IRQ_GUEST_MIGRATING   4
    unsigned long status;
    struct irq_desc *desc; /* only set it the irq corresponds to a physical irq */
    unsigned int irq;
#define GIC_INVALID_LR         ~(uint8_t)0
    uint8_t lr;
    uint8_t priority;
    /* inflight is used to append instances of pending_irq to
     * vgic.inflight_irqs */
    struct list_head inflight;
    /* lr_queue is used to append instances of pending_irq to
     * lr_pending. lr_pending is a per vcpu queue, therefore lr_queue
     * accesses are protected with the vgic lock.
     * TODO: when implementing irq migration, taking only the current
     * vgic lock is not going to be enough. */
    struct list_head lr_queue;
};

/* Represents state corresponding to a block of 32 interrupts */
struct vgic_irq_rank {
    spinlock_t lock; /* Covers access to all other members of this struct */
    uint32_t ienable;
    uint32_t icfg[2];
    uint32_t ipriority[8];
    union {
        struct {
            uint32_t itargets[8];
        }v2;
        struct {
            uint64_t irouter[32];
        }v3;
    };
};

struct sgi_target {
    uint8_t aff1;
    uint16_t list;
};

static inline void sgi_target_init(struct sgi_target *sgi_target)
{
    sgi_target->aff1 = 0;
    sgi_target->list = 0;
}

struct vgic_info {
    bool_t its_enabled;
};

struct vgic_ops {
    /* Hold vGIC information */
    const struct vgic_info *info;
    /* Initialize vGIC */
    int (*vcpu_init)(struct vcpu *v);
    /* Domain specific initialization of vGIC */
    int (*domain_init)(struct domain *d);
    /* Get priority for a given irq stored in vgic structure */
    int (*get_irq_priority)(struct vcpu *v, unsigned int irq);
    /* Get the target vcpu for a given virq. The rank lock is already taken
     * when calling this. */
    struct vcpu *(*get_target_vcpu)(struct vcpu *v, unsigned int irq);
    /* vGIC sysreg emulation */
    int (*emulate_sysreg)(struct cpu_user_regs *regs, union hsr hsr);
    /* Maximum number of vCPU supported */
    const unsigned int max_vcpus;
};

/* Number of ranks of interrupt registers for a domain */
#define DOMAIN_NR_RANKS(d) (((d)->arch.vgic.nr_spis+31)/32)

#define vgic_lock(v)   spin_lock_irq(&(v)->domain->arch.vgic.lock)
#define vgic_unlock(v) spin_unlock_irq(&(v)->domain->arch.vgic.lock)

#define vgic_lock_rank(v, r, flags)   spin_lock_irqsave(&(r)->lock, flags)
#define vgic_unlock_rank(v, r, flags) spin_unlock_irqrestore(&(r)->lock, flags)

/*
 * Rank containing GICD_<FOO><n> for GICD_<FOO> with
 * <b>-bits-per-interrupt
 */
static inline int REG_RANK_NR(int b, uint32_t n)
{
    switch ( b )
    {
    /*
     * IRQ ranks are of size 32. So n cannot be shifted beyond 5 for 32
     * and above. For 64-bit n is already shifted DBAT_DOUBLE_WORD
     * by the caller
     */
    case 64:
    case 32: return n >> 5;
    case 16: return n >> 4;
    case 8: return n >> 3;
    case 4: return n >> 2;
    case 2: return n >> 1;
    case 1: return n;
    default: BUG();
    }
}

static inline uint32_t vgic_byte_read(uint32_t val, int sign, int offset)
{
    int byte = offset & 0x3;

    val = val >> (8*byte);
    if ( sign && (val & 0x80) )
        val |= 0xffffff00;
    else
        val &= 0x000000ff;
    return val;
}

static inline void vgic_byte_write(uint32_t *reg, uint32_t var, int offset)
{
    int byte = offset & 0x3;

    var &= (0xff << (8*byte));

    *reg &= ~(0xff << (8*byte));
    *reg |= var;
}

static inline uint64_t vgic_reg_mask(enum dabt_size size)
{
    if ( size == DABT_DOUBLE_WORD )
        return ~0ULL;
    else
        return ((1ULL << ((1 << size) * 8)) - 1);
}

/*
 * The check on the size supported by the register has to be done by
 * the caller of vgic_regN_*.
 *
 * vgic_reg_* should never be called directly. Instead use the vgic_regN_*
 * according to size of the emulated register
 *
 * Note that the alignment fault will always be taken in the guest
 * (see B3.12.7 DDI0406.b).
 */
static inline bool vgic_reg64_check_access(struct hsr_dabt dabt)
{
    /*
     * 64 bits registers can be accessible using 32-bit and 64-bit unless
     * stated otherwise (See 8.1.3 ARM IHI 0069A).
     */
    return ( dabt.size == DABT_DOUBLE_WORD || dabt.size == DABT_WORD );
}

static inline register_t vgic_reg_read(uint64_t reg,
                                       unsigned int offset,
                                       enum dabt_size size)
{
    reg >>= 8 * offset;
    reg &= vgic_reg_mask(size);

    return reg;
}

static inline void vgic_reg_write(uint64_t *reg, register_t val,
                                  unsigned int offset,
                                  enum dabt_size size)
{
    uint64_t mask = vgic_reg_mask(size);
    int shift = offset * 8;

    *reg &= ~(mask << shift);
    *reg |= ((uint64_t)val & mask) << shift;
}

static inline void vgic_reg_setbit(uint64_t *reg, register_t bits,
                                   unsigned int offset,
                                   enum dabt_size size)
{
    uint64_t mask = vgic_reg_mask(size);
    int shift = offset * 8;

    *reg |= ((uint64_t)bits & mask) << shift;
}

static inline void vgic_reg_clearbit(uint64_t *reg, register_t bits,
                                     unsigned int offset,
                                     enum dabt_size size)
{
    uint64_t mask = vgic_reg_mask(size);
    int shift = offset * 8;

    *reg &= ~(((uint64_t)bits & mask) << shift);
}

/* N-bit register helpers */
#define VGIC_REG_HELPERS(sz, offmask)                                   \
static inline register_t vgic_reg##sz##_read(uint##sz##_t reg,          \
                                             const mmio_info_t *info)   \
{                                                                       \
    return vgic_reg_read(reg, info->gpa & offmask,                      \
                         info->dabt.size);                              \
}                                                                       \
                                                                        \
static inline void vgic_reg##sz##_write(uint##sz##_t *reg,              \
                                        register_t val,                 \
                                        const mmio_info_t *info)        \
{                                                                       \
    uint64_t tmp = *reg;                                                \
                                                                        \
    vgic_reg_write(&tmp, val, info->gpa & offmask,                      \
                   info->dabt.size);                                    \
                                                                        \
    *reg = tmp;                                                         \
}                                                                       \
                                                                        \
static inline void vgic_reg##sz##_setbit(uint##sz##_t *reg,             \
                                         register_t bits,               \
                                         const mmio_info_t *info)       \
{                                                                       \
    uint64_t tmp = *reg;                                                \
                                                                        \
    vgic_reg_setbit(&tmp, bits, info->gpa & offmask,                    \
                    info->dabt.size);                                   \
                                                                        \
    *reg = tmp;                                                         \
}                                                                       \
                                                                        \
static inline void vgic_reg##sz##_clearbit(uint##sz##_t *reg,           \
                                           register_t bits,             \
                                           const mmio_info_t *info)     \
{                                                                       \
    uint64_t tmp = *reg;                                                \
                                                                        \
    vgic_reg_clearbit(&tmp, bits, info->gpa & offmask,                  \
                    info->dabt.size);                                   \
                                                                        \
    *reg = tmp;                                                         \
}

VGIC_REG_HELPERS(64, 0x7);
VGIC_REG_HELPERS(32, 0x3);

#undef VGIC_REG_HELPERS

enum gic_sgi_mode;

/*
 * Offset of GICD_<FOO><n> with its rank, for GICD_<FOO> size <s> with
 * <b>-bits-per-interrupt.
 */
#define REG_RANK_INDEX(b, n, s) ((((n) >> s) & ((b)-1)) % 32)

#define vgic_num_line_irqs(d)        ((d)->arch.vgic.nr_spis + 32)

extern bool_t vgic_is_lpi_supported(struct domain *d);
extern int domain_vgic_init(struct domain *d, unsigned int nr_spis);
extern void domain_vgic_free(struct domain *d);
extern int vcpu_vgic_init(struct vcpu *v);
extern struct vcpu *vgic_get_target_vcpu(struct vcpu *v, unsigned int irq);
extern void vgic_vcpu_inject_irq(struct vcpu *v, unsigned int virq);
extern void vgic_vcpu_inject_spi(struct domain *d, unsigned int virq);
extern void vgic_clear_pending_irqs(struct vcpu *v);
extern struct pending_irq *irq_to_pending(struct vcpu *v, unsigned int irq);
extern struct pending_irq *spi_to_pending(struct domain *d, unsigned int irq);
extern struct vgic_irq_rank *vgic_rank_offset(struct vcpu *v, int b, int n, int s);
extern struct vgic_irq_rank *vgic_rank_irq(struct vcpu *v, unsigned int irq);
extern int vgic_emulate(struct cpu_user_regs *regs, union hsr hsr);
extern void vgic_disable_irqs(struct vcpu *v, uint32_t r, int n);
extern void vgic_enable_irqs(struct vcpu *v, uint32_t r, int n);
extern void register_vgic_ops(struct domain *d, const struct vgic_ops *ops);
int vgic_v2_init(struct domain *d);
int vgic_v3_init(struct domain *d);

extern int vcpu_vgic_free(struct vcpu *v);
extern int vgic_to_sgi(struct vcpu *v, register_t sgir,
                       enum gic_sgi_mode irqmode, int virq,
                       const struct sgi_target *target);
extern void vgic_migrate_irq(struct vcpu *old, struct vcpu *new, unsigned int irq);

/* Reserve a specific guest vIRQ */
extern bool_t vgic_reserve_virq(struct domain *d, unsigned int virq);

/*
 * Allocate a guest VIRQ
 *  - spi == 0 => allocate a PPI. It will be the same on every vCPU
 *  - spi == 1 => allocate an SPI
 */
extern int vgic_allocate_virq(struct domain *d, bool_t spi);

static inline int vgic_allocate_ppi(struct domain *d)
{
    return vgic_allocate_virq(d, 0 /* ppi */);
}

static inline int vgic_allocate_spi(struct domain *d)
{
    return vgic_allocate_virq(d, 1 /* spi */);
}

extern void vgic_free_virq(struct domain *d, unsigned int virq);

void vgic_v2_setup_hw(paddr_t dbase, paddr_t cbase, paddr_t vbase);

#ifdef HAS_GICV3
struct rdist_region;
void vgic_v3_setup_hw(paddr_t dbase,
                      unsigned int nr_rdist_regions,
                      const struct rdist_region *regions,
                      uint32_t rdist_stride);
#endif

#endif /* __ASM_ARM_VGIC_H__ */

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
