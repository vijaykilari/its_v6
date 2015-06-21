/*
 * xen/arch/arm/vgic-v3.c
 *
 * ARM Virtual Generic Interrupt Controller v3 support
 * based on xen/arch/arm/vgic.c
 *
 * Vijaya Kumar K <vijaya.kumar@caviumnetworks.com>
 * Copyright (c) 2014 Cavium Inc.
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

#include <xen/bitops.h>
#include <xen/config.h>
#include <xen/lib.h>
#include <xen/init.h>
#include <xen/softirq.h>
#include <xen/irq.h>
#include <xen/sched.h>
#include <xen/sizes.h>
#include <asm/current.h>
#include <asm/mmio.h>
#include <asm/gic_v3_defs.h>
#include <asm/gic.h>
#include <asm/gic-its.h>
#include <asm/vgic.h>

/* GICD_PIDRn register values for ARM implementations */
#define GICV3_GICD_PIDR0  0x92
#define GICV3_GICD_PIDR1  0xb4
#define GICV3_GICD_PIDR2  0x3b
#define GICV3_GICD_PIDR4  0x04

/* GICR_PIDRn register values for ARM implementations */
#define GICV3_GICR_PIDR0  0x93
#define GICV3_GICR_PIDR1  GICV3_GICD_PIDR1
#define GICV3_GICR_PIDR2  GICV3_GICD_PIDR2
#define GICV3_GICR_PIDR4  GICV3_GICD_PIDR4

/*
 * GICD_CTLR default value:
 *      - No GICv2 compatibility => ARE = 1
 */
#define VGICD_CTLR_DEFAULT  (GICD_CTLR_ARE_NS)

static struct vgic_info vgic_v3_info;

static struct {
    bool_t enabled;
    /* Distributor interface address */
    paddr_t dbase;
    /* Re-distributor regions */
    unsigned int nr_rdist_regions;
    const struct rdist_region *regions;
    uint32_t rdist_stride; /* Re-distributor stride */
} vgic_v3_hw;

void vgic_v3_setup_hw(paddr_t dbase,
                      unsigned int nr_rdist_regions,
                      const struct rdist_region *regions,
                      uint32_t rdist_stride)
{
    vgic_v3_hw.enabled = 1;
    vgic_v3_hw.dbase = dbase;
    vgic_v3_hw.nr_rdist_regions = nr_rdist_regions;
    vgic_v3_hw.regions = regions;
    vgic_v3_hw.rdist_stride = rdist_stride;
}

static struct vcpu *vgic_v3_irouter_to_vcpu(struct domain *d, uint64_t irouter)
{
    unsigned int vcpu_id;

    /*
     * When the Interrupt Route Mode is set, the IRQ targets any vCPUs.
     * For simplicity, the IRQ is always routed to vCPU0.
     */
    if ( irouter & GICD_IROUTER_SPI_MODE_ANY )
        return d->vcpu[0];

    vcpu_id = vaffinity_to_vcpuid(irouter);
    if ( vcpu_id >= d->max_vcpus )
        return NULL;

    return d->vcpu[vcpu_id];
}

static struct vcpu *vgic_v3_get_target_vcpu(struct vcpu *v, unsigned int irq)
{
    struct vcpu *v_target;
    struct vgic_irq_rank *rank = vgic_rank_irq(v, irq);

    ASSERT(spin_is_locked(&rank->lock));

    v_target = vgic_v3_irouter_to_vcpu(v->domain, rank->v3.irouter[irq % 32]);

    ASSERT(v_target != NULL);

    return v_target;
}

static void vits_disable_lpi(struct vcpu *v, uint32_t vlpi)
{
    struct pending_irq *p;
    struct vcpu *v_target = v->domain->vcpu[0];

    p = irq_to_pending(v_target, vlpi);
    if ( test_bit(GIC_IRQ_GUEST_ENABLED, &p->status) )
    {
        clear_bit(GIC_IRQ_GUEST_ENABLED, &p->status);
        gic_remove_from_queues(v_target, vlpi);
    }
}

static void vits_enable_lpi(struct vcpu *v, uint32_t vlpi, uint8_t priority)
{
    struct pending_irq *p;
    unsigned long flags;
    struct vcpu *v_target = v->domain->vcpu[0];

    /*
     * We don't have vlpi to plpi mapping and hence we cannot
     * have target on which corresponding vlpi is enabled.
     * So for now we are always injecting vlpi on vcpu0.
     * (See vgic_vcpu_inject_lpi() function) and so we get pending_irq
     * structure on vcpu0.
     * TODO: Get correct target vcpu
     */
    p = irq_to_pending(v_target, vlpi);

    set_bit(GIC_IRQ_GUEST_ENABLED, &p->status);

    spin_lock_irqsave(&v_target->arch.vgic.lock, flags);

    if ( !list_empty(&p->inflight) &&
         !test_bit(GIC_IRQ_GUEST_VISIBLE, &p->status) )
        gic_raise_guest_irq(v_target, vlpi, p->priority);

    spin_unlock_irqrestore(&v_target->arch.vgic.lock, flags);
}

static int vgic_v3_gits_lpi_mmio_read(struct vcpu *v, mmio_info_t *info)
{
    uint32_t offset;
    void *addr;
    uint64_t val;
    unsigned long flags;
    struct vgic_its *vits = v->domain->arch.vgic.vits;
    struct hsr_dabt dabt = info->dabt;
    struct cpu_user_regs *regs = guest_cpu_user_regs();
    register_t *r = select_user_reg(regs, dabt.reg);

    offset = info->gpa - (vits->propbase & GICR_PROPBASER_PA_MASK);
    if ( offset > vits->prop_size )
    {
        dprintk(XENLOG_G_ERR, "%pv: vITS: LPI property table out of range\n",
                current);
        return 1;
    }

    /* Neglect if not LPI. */
    if ( offset < FIRST_GIC_LPI )
    {
        *r = 0;
        return 1;
    }

    addr = (void *)((u8*)vits->prop_page + offset);
    spin_lock_irqsave(&vits->prop_lock, flags);
    switch (dabt.size)
    {
    case DABT_DOUBLE_WORD:
        val = *((u64*)addr);
        *r = val;
        break;
    case DABT_WORD:
        val = *((u32*)addr);
        *r = (u32)val;
        break;
    case DABT_HALF_WORD:
        val = *((u16*)addr);
        *r = (u16)val;
        break;
    default:
        val = *((u8*)addr);
        *r = (u8)val;
    }
    spin_unlock_irqrestore(&vits->prop_lock, flags);

    return 1;
}

static void vgic_v3_gits_update_lpis_state(struct vcpu *v, uint32_t vid,
                                           uint32_t size)
{
    struct vgic_its *vits = v->domain->arch.vgic.vits;
    uint32_t i;
    uint8_t cfg, *p;
    bool_t enable;

    p = ((u8*)vits->prop_page + vid);

    for ( i = 0 ; i < size; i++ )
    {
        cfg = *p;
        enable = cfg & LPI_PROP_ENABLED;

        if ( !enable )
            vits_enable_lpi(v, vid,  (cfg & LPI_PRIORITY_MASK));
        else
            vits_disable_lpi(v, vid);

        p++;
        vid++;
    }
}

static int vgic_v3_gits_lpi_mmio_write(struct vcpu *v, mmio_info_t *info)
{
    uint32_t offset;
    uint8_t cfg, *p, *val, i, iter;
    bool_t enable;
    unsigned long flags;
    struct vgic_its *vits = v->domain->arch.vgic.vits;
    struct hsr_dabt dabt = info->dabt;
    struct cpu_user_regs *regs = guest_cpu_user_regs();
    register_t *r = select_user_reg(regs, dabt.reg);

    offset = info->gpa - (vits->propbase & GICR_PROPBASER_PA_MASK);
    if ( offset > vits->prop_size )
    {
        dprintk(XENLOG_G_ERR, "%pv: vITS: LPI property table out of range\n",
                current);
        return 1;
    }

    /* Neglect if not LPI. */
    if ( offset < FIRST_GIC_LPI )
        return 1;

    switch (dabt.size)
    {
    case DABT_DOUBLE_WORD:
        iter = 8;
        break;
    case DABT_WORD:
        iter = 4;
        break;
    case DABT_HALF_WORD:
        iter = 2;
        break;
    default:
        iter = 1;
    }
    spin_lock_irqsave(&vits->prop_lock, flags);

    p = ((u8*)vits->prop_page + offset);
    val = (u8*)r;

    for ( i = 0 ; i < iter; i++ )
    {
        cfg = *p;
        enable = (cfg & *val) & LPI_PROP_ENABLED;

        if ( !enable )
            vits_enable_lpi(v, offset,  (*val & LPI_PRIORITY_MASK));
        else
            vits_disable_lpi(v, offset);

        /* Update virtual prop page */
        *p = (*val & 0xff);
        val++;
        p++;
        offset++;
    }

    spin_unlock_irqrestore(&vits->prop_lock, flags);

    return 1;
}

static const struct mmio_handler_ops vgic_gits_lpi_mmio_handler = {
    .read_handler  = vgic_v3_gits_lpi_mmio_read,
    .write_handler = vgic_v3_gits_lpi_mmio_write,
};

static int vits_map_lpi_prop(struct vcpu *v)
{
    struct vgic_its *vits = v->domain->arch.vgic.vits;
    paddr_t gaddr, addr;
    unsigned long mfn, flags;
    uint32_t lpi_size, id_bits;
    int i;

    gaddr = vits->propbase & GICR_PROPBASER_PA_MASK;
    id_bits = ((vits->propbase & GICR_PROPBASER_IDBITS_MASK) + 1);

    if ( id_bits > v->domain->arch.vgic.id_bits )
        id_bits = v->domain->arch.vgic.id_bits;

    lpi_size = 1UL << id_bits;
    if ( lpi_size < FIRST_GIC_LPI )
    {
        dprintk(XENLOG_G_ERR,
                "d%"PRIu16": vITS: LPI property table does not hold LPI config\n",
                v->domain->domain_id);
        return 0;
    }

    vits->prop_size = lpi_size;
    /* Allocate Virtual LPI Property table */
    /* TODO: To re-use guest property table? */
    vits->prop_page = alloc_xenheap_pages(get_order_from_bytes(lpi_size), 0);
    if ( !vits->prop_page )
    {
        dprintk(XENLOG_G_ERR,
                "d%"PRIu16": vITS: Fail to allocate LPI Prop page\n",
                v->domain->domain_id);
        return 0;
    }

    addr = gaddr;

    /*
     * LPIs start from 8192 index in LPI property table.
     * So subtract 8192 from lpi_size.
     */
    spin_lock_irqsave(&vits->prop_lock, flags);
    for ( i = 0; i < (lpi_size - FIRST_GIC_LPI) / PAGE_SIZE; i++ )
    {
        vits_access_guest_table(v->domain, addr,
            (void *)(vits->prop_page + FIRST_GIC_LPI + (i * PAGE_SIZE)),
            PAGE_SIZE, 0);

        vgic_v3_gits_update_lpis_state(v,  FIRST_GIC_LPI + (i * PAGE_SIZE),
                                       PAGE_SIZE);
        addr += PAGE_SIZE;
    }
    spin_unlock_irqrestore(&vits->prop_lock, flags);

    /*
     * Each re-distributor shares a common LPI configuration table
     * So one set of mmio handlers to manage configuration table is enough
     */
    addr = gaddr;
    for ( i = 0; i < lpi_size / PAGE_SIZE; i++ )
    {
        mfn = gmfn_to_mfn(v->domain, paddr_to_pfn(addr));
        if ( unlikely(!mfn_valid(mfn)) )
        {
            dprintk(XENLOG_G_ERR,
                    "vITS: Invalid propbaser address for domain %"PRIu16"\n",
                    v->domain->domain_id);
            return 0;
        }
        guest_physmap_remove_page(v->domain, paddr_to_pfn(addr), mfn, 0);
        addr += PAGE_SIZE;
    }

    /* Register mmio handlers for this region */
    register_mmio_handler(v->domain, &vgic_gits_lpi_mmio_handler,
                          gaddr, lpi_size);

    return 1;
}

static int __vgic_v3_rdistr_rd_mmio_read(struct vcpu *v, mmio_info_t *info,
                                         uint32_t gicr_reg)
{
    struct hsr_dabt dabt = info->dabt;
    struct cpu_user_regs *regs = guest_cpu_user_regs();
    register_t *r = select_user_reg(regs, dabt.reg);
    struct vgic_its *vits;

    switch ( gicr_reg )
    {
    case GICR_CTLR:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        vgic_lock(v);
        *r = vgic_reg32_read(v->domain->arch.vgic.gicr_ctlr, info);
        vgic_unlock(v);
        return 1;
    case GICR_IIDR:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        *r = vgic_reg32_read(GICV3_GICR_IIDR_VAL, info);
        return 1;
    case GICR_TYPER:
    case GICR_TYPER + 4:
    {
        uint64_t typer, aff;

        if ( !vgic_reg64_check_access(dabt) ) goto bad_width;
        /* TBD: Update processor id in [23:8] when ITS support is added */
        aff = (MPIDR_AFFINITY_LEVEL(v->arch.vmpidr, 3) << 56 |
               MPIDR_AFFINITY_LEVEL(v->arch.vmpidr, 2) << 48 |
               MPIDR_AFFINITY_LEVEL(v->arch.vmpidr, 1) << 40 |
               MPIDR_AFFINITY_LEVEL(v->arch.vmpidr, 0) << 32);
        typer = aff;

        if ( v->arch.vgic.flags & VGIC_V3_RDIST_LAST )
            typer |= GICR_TYPER_LAST;

        /* Set Physical LPIs support */
        if ( vgic_is_lpi_supported(v->domain) )
            typer |= GICR_TYPER_PLPIS;
        /* GITS_TYPER.PTA is 0. Provide vcpu number as target address */
        typer |= (v->vcpu_id << GICR_TYPER_PROCESSOR_SHIFT);

        if ( dabt.size == DABT_DOUBLE_WORD )
            *r = vgic_reg64_read(typer, info);
        else
            *r = vgic_reg32_read(typer, info);

        return 1;
    }
    case GICR_STATUSR:
        /* Not implemented */
        goto read_as_zero_32;
    case GICR_WAKER:
        /* Power management is not implemented */
        goto read_as_zero_32;
    case GICR_SETLPIR:
        /* WO. Read as zero */
        goto read_as_zero_64;
    case GICR_CLRLPIR:
        /* WO. Read as zero */
        goto read_as_zero_64;
    case GICR_PROPBASER:
        if ( dabt.size != DABT_DOUBLE_WORD ) goto bad_width;
        if ( vgic_is_lpi_supported(v->domain) )
        {
            vits = v->domain->arch.vgic.vits;
            vgic_lock(v);
            *r = vgic_reg64_read(vits->propbase, info);
            vgic_unlock(v);
            return 1;
        }
        goto read_as_zero_64;
    case GICR_PENDBASER:
        if ( dabt.size != DABT_DOUBLE_WORD ) goto bad_width;
        if ( vgic_is_lpi_supported(v->domain) )
        {
            uint64_t val;

            vgic_lock(v);
            val = vgic_reg64_read(v->arch.vgic.pendbase, info);
            /* PTZ field is WO */
            *r = val & ~GICR_PENDBASER_PTZ_MASK;
            vgic_unlock(v);
            return 1;
        }
        goto read_as_zero_64;
    case GICR_INVLPIR:
        /* WO. Read as zero */
        goto read_as_zero_64;
    case GICR_INVALLR:
        /* WO. Read as zero */
        goto read_as_zero_64;
        return 0;
    case GICR_SYNCR:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        /* RO . But when read it always returns busy bito bit[0] */
        *r = vgic_reg32_read(GICR_SYNCR_NOT_BUSY, info);
        return 1;
    case GICR_MOVLPIR:
        /* WO Read as zero */
        goto read_as_zero_64;
    case GICR_MOVALLR:
        /* WO Read as zero */
        goto read_as_zero_64;
    case GICR_PIDR0:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        *r = vgic_reg32_read(GICV3_GICR_PIDR0, info);
         return 1;
    case GICR_PIDR1:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        *r = vgic_reg32_read(GICV3_GICR_PIDR1, info);
         return 1;
    case GICR_PIDR2:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        *r = vgic_reg32_read(GICV3_GICR_PIDR2, info);
         return 1;
    case GICR_PIDR3:
        /* Manufacture/customer defined */
        goto read_as_zero_32;
    case GICR_PIDR4:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        *r = vgic_reg32_read(GICV3_GICR_PIDR4, info);
         return 1;
    case GICR_PIDR5 ... GICR_PIDR7:
        /* Reserved0 */
        goto read_as_zero_32;
    default:
        printk(XENLOG_G_ERR
               "%pv: vGICR: unhandled read r%d offset %#08x\n",
               v, dabt.reg, gicr_reg);
        return 0;
    }
bad_width:
    printk(XENLOG_G_ERR "%pv vGICR: bad read width %d r%d offset %#08x\n",
           v, dabt.size, dabt.reg, gicr_reg);
    domain_crash_synchronous();
    return 0;

read_as_zero_64:
    if ( !vgic_reg64_check_access(dabt) ) goto bad_width;
    *r = 0;
    return 1;

read_as_zero_32:
    if ( dabt.size != DABT_WORD ) goto bad_width;
    *r = 0;
    return 1;
}

static int __vgic_v3_rdistr_rd_mmio_write(struct vcpu *v, mmio_info_t *info,
                                          uint32_t gicr_reg)
{
    struct hsr_dabt dabt = info->dabt;
    struct cpu_user_regs *regs = guest_cpu_user_regs();
    register_t *r = select_user_reg(regs, dabt.reg);

    switch ( gicr_reg )
    {
    case GICR_CTLR:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        if ( vgic_is_lpi_supported(v->domain) )
        {
            /*
             * Enable LPI's for ITS. Direct injection of LPI
             * by writing to GICR_{SET,CLR}LPIR is not supported.
             */
            vgic_lock(v);
            vgic_reg32_write(&v->domain->arch.vgic.gicr_ctlr,
                             (*r & GICR_CTLR_ENABLE_LPIS), info);
            vgic_unlock(v);
            return 1;
        }
        goto write_ignore_32;
    case GICR_IIDR:
        /* RO */
        goto write_ignore_32;
    case GICR_TYPER:
        /* RO */
        goto write_ignore_64;
    case GICR_STATUSR:
        /* Not implemented */
        goto write_ignore_32;
    case GICR_WAKER:
        /* Power mgmt not implemented */
        goto write_ignore_32;
    case GICR_SETLPIR:
        /* LPI is not implemented */
        goto write_ignore_64;
    case GICR_CLRLPIR:
        /* LPI is not implemented */
        goto write_ignore_64;
    case GICR_PROPBASER:
        if ( dabt.size != DABT_DOUBLE_WORD ) goto bad_width;
        if ( vgic_is_lpi_supported(v->domain) )
        {
            vgic_lock(v);
            /* LPI configuration tables are shared across cpus. Should be same */
            /*
             * Allow updating on propbase only once with below check.
             * TODO: Manage change in property table.
             */
            if ( v->domain->arch.vgic.vits->propbase != 0 )
            {
                printk(XENLOG_G_WARNING
                       "%pv: vGICR: Updating LPI propbase is not allowed\n", v);
                vgic_unlock(v);
                return 1;
            }
            vgic_reg64_write(&v->domain->arch.vgic.vits->propbase, *r, info);
            vgic_unlock(v);
            return vits_map_lpi_prop(v);
        }
        goto write_ignore_64;
    case GICR_PENDBASER:
        if ( dabt.size != DABT_DOUBLE_WORD ) goto bad_width;
        if ( vgic_is_lpi_supported(v->domain) )
        {
            /* Just hold pendbaser value for guest read */
            vgic_lock(v);
            vgic_reg64_write(&v->arch.vgic.pendbase, *r, info);
            vgic_unlock(v);
           return 1;
        }
        goto write_ignore_64;
    case GICR_INVLPIR:
        /* LPI is not implemented */
        goto write_ignore_64;
    case GICR_INVALLR:
        /* LPI is not implemented */
        goto write_ignore_64;
    case GICR_SYNCR:
        /* RO */
        goto write_ignore_32;
    case GICR_MOVLPIR:
        /* LPI is not implemented */
        goto write_ignore_64;
    case GICR_MOVALLR:
        /* LPI is not implemented */
        goto write_ignore_64;
    case GICR_PIDR7... GICR_PIDR0:
        /* RO */
        goto write_ignore_32;
    default:
        printk(XENLOG_G_ERR "%pv: vGICR: unhandled write r%d offset %#08x\n",
               v, dabt.reg, gicr_reg);
        return 0;
    }
bad_width:
    printk(XENLOG_G_ERR
          "%pv: vGICR: bad write width %d r%d=%"PRIregister" offset %#08x\n",
          v, dabt.size, dabt.reg, *r, gicr_reg);
    domain_crash_synchronous();
    return 0;

write_ignore_64:
    if ( vgic_reg64_check_access(dabt) ) goto bad_width;
    return 1;

write_ignore_32:
    if ( dabt.size != DABT_WORD ) goto bad_width;
    return 1;
}

static int __vgic_v3_distr_common_mmio_read(const char *name, struct vcpu *v,
                                            mmio_info_t *info, uint32_t reg)
{
    struct hsr_dabt dabt = info->dabt;
    struct cpu_user_regs *regs = guest_cpu_user_regs();
    register_t *r = select_user_reg(regs, dabt.reg);
    struct vgic_irq_rank *rank;
    unsigned long flags;

    switch ( reg )
    {
    case GICD_IGROUPR ... GICD_IGROUPRN:
        /* We do not implement security extensions for guests, read zero */
        if ( dabt.size != DABT_WORD ) goto bad_width;
        goto read_as_zero;
    case GICD_ISENABLER ... GICD_ISENABLERN:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        rank = vgic_rank_offset(v, 1, reg - GICD_ISENABLER, DABT_WORD);
        if ( rank == NULL ) goto read_as_zero;
        vgic_lock_rank(v, rank, flags);
        *r = vgic_reg32_read(rank->ienable, info);
        vgic_unlock_rank(v, rank, flags);
        return 1;
    case GICD_ICENABLER ... GICD_ICENABLERN:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        rank = vgic_rank_offset(v, 1, reg - GICD_ICENABLER, DABT_WORD);
        if ( rank == NULL ) goto read_as_zero;
        vgic_lock_rank(v, rank, flags);
        *r = vgic_reg32_read(rank->ienable, info);
        vgic_unlock_rank(v, rank, flags);
        return 1;
    /* Read the pending status of an IRQ via GICD/GICR is not supported */
    case GICD_ISPENDR ... GICD_ISPENDRN:
    case GICD_ICPENDR ... GICD_ICPENDRN:
        goto read_as_zero;

    /* Read the active status of an IRQ via GICD/GICR is not supported */
    case GICD_ISACTIVER ... GICD_ISACTIVERN:
    case GICD_ICACTIVER ... GICD_ICACTIVERN:
        goto read_as_zero;

    case GICD_IPRIORITYR ... GICD_IPRIORITYRN:
    {
        uint32_t ipriority;

        if ( dabt.size != DABT_BYTE && dabt.size != DABT_WORD ) goto bad_width;
        rank = vgic_rank_offset(v, 8, reg - GICD_IPRIORITYR, DABT_WORD);
        if ( rank == NULL ) goto read_as_zero;

        vgic_lock_rank(v, rank, flags);
        ipriority = rank->ipriority[REG_RANK_INDEX(8, reg - GICD_IPRIORITYR,
                                                   DABT_WORD)];
        vgic_unlock_rank(v, rank, flags);

        *r = vgic_reg32_read(ipriority, info);

        return 1;
    }
    case GICD_ICFGR ... GICD_ICFGRN:
    {
        uint32_t icfgr;

        if ( dabt.size != DABT_WORD ) goto bad_width;
        rank = vgic_rank_offset(v, 2, reg - GICD_ICFGR, DABT_WORD);
        if ( rank == NULL ) goto read_as_zero;
        vgic_lock_rank(v, rank, flags);
        icfgr = rank->icfg[REG_RANK_INDEX(2, reg - GICD_ICFGR, DABT_WORD)];
        vgic_unlock_rank(v, rank, flags);

        *r = vgic_reg32_read(icfgr, info);

        return 1;
    }
    default:
        printk(XENLOG_G_ERR
               "%pv: %s: unhandled read r%d offset %#08x\n",
               v, name, dabt.reg, reg);
        return 0;
    }

bad_width:
    printk(XENLOG_G_ERR "%pv: %s: bad read width %d r%d offset %#08x\n",
           v, name, dabt.size, dabt.reg, reg);
    domain_crash_synchronous();
    return 0;

read_as_zero:
    *r = 0;
    return 1;
}

static int __vgic_v3_distr_common_mmio_write(const char *name, struct vcpu *v,
                                             mmio_info_t *info, uint32_t reg)
{
    struct hsr_dabt dabt = info->dabt;
    struct cpu_user_regs *regs = guest_cpu_user_regs();
    register_t *r = select_user_reg(regs, dabt.reg);
    struct vgic_irq_rank *rank;
    uint32_t tr;
    unsigned long flags;

    switch ( reg )
    {
    case GICD_IGROUPR ... GICD_IGROUPRN:
        /* We do not implement security extensions for guests, write ignore */
        goto write_ignore_32;
    case GICD_ISENABLER ... GICD_ISENABLERN:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        rank = vgic_rank_offset(v, 1, reg - GICD_ISENABLER, DABT_WORD);
        if ( rank == NULL ) goto write_ignore;
        vgic_lock_rank(v, rank, flags);
        tr = rank->ienable;
        vgic_reg32_setbit(&rank->ienable, *r, info);
        /* The irq number is extracted from offset. so shift by register size */
        vgic_enable_irqs(v, (*r) & (~tr), (reg - GICD_ISENABLER) >> DABT_WORD);
        vgic_unlock_rank(v, rank, flags);
        return 1;
    case GICD_ICENABLER ... GICD_ICENABLERN:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        rank = vgic_rank_offset(v, 1, reg - GICD_ICENABLER, DABT_WORD);
        if ( rank == NULL ) goto write_ignore;
        vgic_lock_rank(v, rank, flags);
        tr = rank->ienable;
        vgic_reg32_clearbit(&rank->ienable, *r, info);
        rank->ienable &= ~*r;
        /* The irq number is extracted from offset. so shift by register size */
        vgic_disable_irqs(v, (*r) & tr, (reg - GICD_ICENABLER) >> DABT_WORD);
        vgic_unlock_rank(v, rank, flags);
        return 1;
    case GICD_ISPENDR ... GICD_ISPENDRN:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        printk(XENLOG_G_ERR
               "%pv: %s: unhandled word write %#"PRIregister" to ISPENDR%d\n",
               v, name, *r, reg - GICD_ISPENDR);
        return 0;

    case GICD_ICPENDR ... GICD_ICPENDRN:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        printk(XENLOG_G_ERR
               "%pv: %s: unhandled word write %#"PRIregister" to ICPENDR%d\n",
               v, name, *r, reg - GICD_ICPENDR);
        return 0;

    case GICD_ISACTIVER ... GICD_ISACTIVERN:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        printk(XENLOG_G_ERR
               "%pv: %s: unhandled word write %#"PRIregister" to ISACTIVER%d\n",
               v, name, *r, reg - GICD_ISACTIVER);
        return 0;

    case GICD_ICACTIVER ... GICD_ICACTIVERN:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        printk(XENLOG_G_ERR
               "%pv: %s: unhandled word write %#"PRIregister" to ICACTIVER%d\n",
               v, name, *r, reg - GICD_ICACTIVER);
        return 0;

    case GICD_IPRIORITYR ... GICD_IPRIORITYRN:
        if ( dabt.size != DABT_BYTE && dabt.size != DABT_WORD ) goto bad_width;
        rank = vgic_rank_offset(v, 8, reg - GICD_IPRIORITYR, DABT_WORD);
        if ( rank == NULL ) goto write_ignore;
        vgic_lock_rank(v, rank, flags);
        vgic_reg32_write(&rank->ipriority[REG_RANK_INDEX(8,
                                                         reg - GICD_IPRIORITYR,
                                                         DABT_WORD)],
                         *r, info);
        vgic_unlock_rank(v, rank, flags);
        return 1;
    case GICD_ICFGR: /* Restricted to configure SGIs */
        goto write_ignore_32;
    case GICD_ICFGR + 4 ... GICD_ICFGRN: /* PPI + SPIs */
        /* ICFGR1 for PPI's, which is implementation defined
           if ICFGR1 is programmable or not. We chose to program */
        if ( dabt.size != DABT_WORD ) goto bad_width;
        rank = vgic_rank_offset(v, 2, reg - GICD_ICFGR, DABT_WORD);
        if ( rank == NULL ) goto write_ignore;
        vgic_lock_rank(v, rank, flags);
        vgic_reg32_write(&rank->icfg[REG_RANK_INDEX(2, reg - GICD_ICFGR,
                                                    DABT_WORD)],
                         *r, info);
        vgic_unlock_rank(v, rank, flags);
        return 1;
    default:
        printk(XENLOG_G_ERR
               "%pv: %s: unhandled write r%d=%"PRIregister" offset %#08x\n",
               v, name, dabt.reg, *r, reg);
        return 0;
    }

bad_width:
    printk(XENLOG_G_ERR
           "%pv: %s: bad write width %d r%d=%"PRIregister" offset %#08x\n",
           v, name, dabt.size, dabt.reg, *r, reg);
    domain_crash_synchronous();
    return 0;

write_ignore_32:
    if ( dabt.size != DABT_WORD ) goto bad_width;
write_ignore:
    return 1;
}

static int vgic_v3_rdistr_sgi_mmio_read(struct vcpu *v, mmio_info_t *info,
                                        uint32_t gicr_reg)
{
    struct hsr_dabt dabt = info->dabt;
    struct cpu_user_regs *regs = guest_cpu_user_regs();
    register_t *r = select_user_reg(regs, dabt.reg);

    switch ( gicr_reg )
    {
    case GICR_IGRPMODR0:
        /* We do not implement security extensions for guests, read zero */
        goto read_as_zero_32;
    case GICR_IGROUPR0:
    case GICR_ISENABLER0:
    case GICR_ICENABLER0:
    case GICR_ISACTIVER0:
    case GICR_ICACTIVER0:
    case GICR_IPRIORITYR0...GICR_IPRIORITYR7:
    case GICR_ICFGR0... GICR_ICFGR1:
         /*
          * Above registers offset are common with GICD.
          * So handle in common with GICD handling
          */
        return __vgic_v3_distr_common_mmio_read("vGICR: SGI", v, info,
                                                gicr_reg);

    /* Read the pending status of an SGI is via GICR is not supported */
    case GICR_ISPENDR0:
    case GICR_ICPENDR0:
        goto read_as_zero;

    case GICR_NSACR:
        /* We do not implement security extensions for guests, read zero */
        goto read_as_zero_32;

    default:
        printk(XENLOG_G_ERR
               "%pv: vGICR: SGI: unhandled read r%d offset %#08x\n",
               v, dabt.reg, gicr_reg);
        return 0;
    }
bad_width:
    printk(XENLOG_G_ERR "%pv: vGICR: SGI: bad read width %d r%d offset %#08x\n",
           v, dabt.size, dabt.reg, gicr_reg);
    domain_crash_synchronous();
    return 0;

read_as_zero_32:
    if ( dabt.size != DABT_WORD ) goto bad_width;
read_as_zero:
    *r = 0;
    return 1;
}

static int vgic_v3_rdistr_sgi_mmio_write(struct vcpu *v, mmio_info_t *info,
                                         uint32_t gicr_reg)
{
    struct hsr_dabt dabt = info->dabt;
    struct cpu_user_regs *regs = guest_cpu_user_regs();
    register_t *r = select_user_reg(regs, dabt.reg);

    switch ( gicr_reg )
    {
    case GICR_IGRPMODR0:
        /* We do not implement security extensions for guests, write ignore */
        goto write_ignore_32;
    case GICR_IGROUPR0:
    case GICR_ISENABLER0:
    case GICR_ICENABLER0:
    case GICR_ISACTIVER0:
    case GICR_ICACTIVER0:
    case GICR_ICFGR1:
    case GICR_IPRIORITYR0...GICR_IPRIORITYR7:
         /*
          * Above registers offset are common with GICD.
          * So handle common with GICD handling
          */
        return __vgic_v3_distr_common_mmio_write("vGICR: SGI", v,
                                                 info, gicr_reg);
    case GICR_ISPENDR0:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        printk(XENLOG_G_ERR
               "%pv: vGICR: SGI: unhandled word write %#"PRIregister" to ISPENDR0\n",
               v, *r);
        return 0;

    case GICR_ICPENDR0:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        printk(XENLOG_G_ERR
               "%pv: vGICR: SGI: unhandled word write %#"PRIregister" to ICPENDR0\n",
               v, *r);
        return 0;

    case GICR_NSACR:
        /* We do not implement security extensions for guests, write ignore */
        goto write_ignore_32;
    default:
        printk(XENLOG_G_ERR
               "%pv: vGICR: SGI: unhandled write r%d offset %#08x\n",
               v, dabt.reg, gicr_reg);
        return 0;
    }

bad_width:
    printk(XENLOG_G_ERR
           "%pv: vGICR: SGI: bad write width %d r%d=%"PRIregister" offset %#08x\n",
           v, dabt.size, dabt.reg, *r, gicr_reg);
    domain_crash_synchronous();
    return 0;

write_ignore_32:
    if ( dabt.size != DABT_WORD ) goto bad_width;
    return 1;
}

static inline struct vcpu *get_vcpu_from_rdist(paddr_t gpa,
                                               struct vcpu *v,
                                               uint32_t *offset)
{
    struct domain *d = v->domain;
    uint32_t stride = d->arch.vgic.rdist_stride;
    paddr_t base;
    int i, vcpu_id;
    struct vgic_rdist_region *region;

    *offset = gpa & (stride - 1);
    base = gpa & ~((paddr_t)stride - 1);

    /* Fast path: the VCPU is trying to access its re-distributor */
    if ( likely(v->arch.vgic.rdist_base == base) )
        return v;

    /* Slow path: the VCPU is trying to access another re-distributor */

    /*
     * Find the region where the re-distributor lives. For this purpose,
     * we look one region ahead as only MMIO range for redistributors
     * traps here.
     * Note: The region has been ordered during the GIC initialization
     */
    for ( i = 1; i < d->arch.vgic.nr_regions; i++ )
    {
        if ( base < d->arch.vgic.rdist_regions[i].base )
            break;
    }

    region = &d->arch.vgic.rdist_regions[i - 1];

    vcpu_id = region->first_cpu + ((base - region->base) / stride);

    if ( unlikely(vcpu_id >= d->max_vcpus) )
        return NULL;

    return d->vcpu[vcpu_id];
}

static int vgic_v3_rdistr_mmio_read(struct vcpu *v, mmio_info_t *info)
{
    uint32_t offset;

    perfc_incr(vgicr_reads);

    v = get_vcpu_from_rdist(info->gpa, v, &offset);
    if ( unlikely(!v) )
        return 0;

    if ( offset < SZ_64K )
        return __vgic_v3_rdistr_rd_mmio_read(v, info, offset);
    else  if ( (offset >= SZ_64K) && (offset < 2 * SZ_64K) )
        return vgic_v3_rdistr_sgi_mmio_read(v, info, (offset - SZ_64K));
    else
        printk(XENLOG_G_WARNING
               "%pv: vGICR: unknown gpa read address %"PRIpaddr"\n",
                v, info->gpa);

    return 0;
}

static int vgic_v3_rdistr_mmio_write(struct vcpu *v, mmio_info_t *info)
{
    uint32_t offset;

    perfc_incr(vgicr_writes);

    v = get_vcpu_from_rdist(info->gpa, v, &offset);
    if ( unlikely(!v) )
        return 0;

    if ( offset < SZ_64K )
        return __vgic_v3_rdistr_rd_mmio_write(v, info, offset);
    else  if ( (offset >= SZ_64K) && (offset < 2 * SZ_64K) )
        return vgic_v3_rdistr_sgi_mmio_write(v, info, (offset - SZ_64K));
    else
        printk(XENLOG_G_WARNING
               "%pv: vGICR: unknown gpa write address %"PRIpaddr"\n",
               v, info->gpa);

    return 0;
}

static int vgic_v3_distr_mmio_read(struct vcpu *v, mmio_info_t *info)
{
    struct hsr_dabt dabt = info->dabt;
    struct cpu_user_regs *regs = guest_cpu_user_regs();
    register_t *r = select_user_reg(regs, dabt.reg);
    struct vgic_irq_rank *rank;
    unsigned long flags;
    int gicd_reg = (int)(info->gpa - v->domain->arch.vgic.dbase);

    perfc_incr(vgicd_reads);

    switch ( gicd_reg )
    {
    case GICD_CTLR:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        vgic_lock(v);
        *r = vgic_reg32_read(v->domain->arch.vgic.ctlr, info);
        vgic_unlock(v);
        return 1;
    case GICD_TYPER:
    {
        /*
         * Number of interrupt identifier bits supported by the GIC
         * Stream Protocol Interface
         */
        unsigned int irq_bits = get_count_order(vgic_num_line_irqs(v->domain));
        /*
         * Number of processors that may be used as interrupt targets when ARE
         * bit is zero. The maximum is 8.
         */
        unsigned int ncpus = min_t(unsigned int, v->domain->max_vcpus, 8);
        uint32_t typer;

        if ( dabt.size != DABT_WORD ) goto bad_width;
        /* No secure world support for guests. */
        typer = ((ncpus - 1) << GICD_TYPE_CPUS_SHIFT |
                 DIV_ROUND_UP(v->domain->arch.vgic.nr_spis, 32));

        irq_bits = v->domain->arch.vgic.id_bits;
        typer |= (irq_bits - 1) << GICD_TYPE_ID_BITS_SHIFT;

        if ( vgic_is_lpi_supported(v->domain) )
            typer |= GICD_TYPE_LPIS;

        *r = vgic_reg32_read(typer, info);
        return 1;
    }
    case GICD_STATUSR:
        /*
         *  Optional, Not implemented for now.
         *  Update to support guest debugging.
         */
        goto read_as_zero_32;
    case GICD_IIDR:
        if ( dabt.size != DABT_WORD ) goto bad_width;
        *r = vgic_reg32_read(GICV3_GICD_IIDR_VAL, info);
        return 1;
    case 0x020 ... 0x03c:
    case 0xc000 ... 0xffcc:
        /* Implementation defined -- read as zero */
        goto read_as_zero_32;
    case GICD_IGROUPR ... GICD_IGROUPRN:
    case GICD_ISENABLER ... GICD_ISENABLERN:
    case GICD_ICENABLER ... GICD_ICENABLERN:
    case GICD_ISPENDR ... GICD_ISPENDRN:
    case GICD_ICPENDR ... GICD_ICPENDRN:
    case GICD_ISACTIVER ... GICD_ISACTIVERN:
    case GICD_IPRIORITYR ... GICD_IPRIORITYRN:
    case GICD_ICFGR ... GICD_ICFGRN:
        /*
         * Above all register are common with GICR and GICD
         * Manage in common
         */
        return __vgic_v3_distr_common_mmio_read("vGICD", v, info, gicd_reg);
    case GICD_IROUTER ... GICD_IROUTER31:
        /* SGI/PPI is RES0 */
        goto read_as_zero_64;
    case GICD_IROUTER32 ... GICD_IROUTERN:
    {
        uint64_t irouter;

        if ( !vgic_reg64_check_access(dabt) ) goto bad_width;
        rank = vgic_rank_offset(v, 64, gicd_reg - GICD_IROUTER,
                                DABT_DOUBLE_WORD);
        if ( rank == NULL ) goto read_as_zero;
        vgic_lock_rank(v, rank, flags);
        irouter = rank->v3.irouter[REG_RANK_INDEX(64,
                                                  (gicd_reg - GICD_IROUTER),
                                                  DABT_DOUBLE_WORD)];
        vgic_unlock_rank(v, rank, flags);

        *r = vgic_reg64_read(irouter, info);

        return 1;
    }
    case GICD_NSACR ... GICD_NSACRN:
        /* We do not implement security extensions for guests, read zero */
        goto read_as_zero_32;
    case GICD_SGIR:
        /* Read as ICH_SGIR system register with SRE set. So ignore */
        goto read_as_zero_32;
    case GICD_CPENDSGIR ... GICD_CPENDSGIRN:
        /* Replaced with GICR_ICPENDR0. So ignore write */
        goto read_as_zero_32;
    case GICD_SPENDSGIR ... GICD_SPENDSGIRN:
        /* Replaced with GICR_ISPENDR0. So ignore write */
        goto read_as_zero_32;
    case GICD_PIDR0:
        /* GICv3 identification value */
        if ( dabt.size != DABT_WORD ) goto bad_width;
        *r = vgic_reg32_read(GICV3_GICD_PIDR0, info);
        return 1;
    case GICD_PIDR1:
        /* GICv3 identification value */
        if ( dabt.size != DABT_WORD ) goto bad_width;
        *r = vgic_reg32_read(GICV3_GICD_PIDR1, info);
        return 1;
    case GICD_PIDR2:
        /* GICv3 identification value */
        if ( dabt.size != DABT_WORD ) goto bad_width;
        *r = vgic_reg32_read(GICV3_GICD_PIDR2, info);
        return 1;
    case GICD_PIDR3:
        /* GICv3 identification value. Manufacturer/Customer defined */
        goto read_as_zero_32;
    case GICD_PIDR4:
        /* GICv3 identification value */
        if ( dabt.size != DABT_WORD ) goto bad_width;
        *r = vgic_reg32_read(GICV3_GICD_PIDR4, info);
        return 1;
    case GICD_PIDR5 ... GICD_PIDR7:
        /* Reserved0 */
        goto read_as_zero_32;
    case 0x00c:
    case 0x044:
    case 0x04c:
    case 0x05c ... 0x07c:
    case 0xf30 ... 0x5fcc:
    case 0x8000 ... 0xbfcc:
        /* These are reserved register addresses */
        printk(XENLOG_G_DEBUG
               "%pv: vGICD: RAZ on reserved register offset %#08x\n",
               v, gicd_reg);
        goto read_as_zero;
    default:
        printk(XENLOG_G_ERR "%pv: vGICD: unhandled read r%d offset %#08x\n",
               v, dabt.reg, gicd_reg);
        return 0;
    }

bad_width:
    printk(XENLOG_G_ERR "%pv: vGICD: bad read width %d r%d offset %#08x\n",
           v, dabt.size, dabt.reg, gicd_reg);
    domain_crash_synchronous();
    return 0;

read_as_zero_64:
    if ( vgic_reg64_check_access(dabt) ) goto bad_width;
    *r = 0;
    return 1;

read_as_zero_32:
    if ( dabt.size != DABT_WORD ) goto bad_width;
    *r = 0;
    return 1;

read_as_zero:
    *r = 0;
    return 1;
}

static int vgic_v3_distr_mmio_write(struct vcpu *v, mmio_info_t *info)
{
    struct hsr_dabt dabt = info->dabt;
    struct cpu_user_regs *regs = guest_cpu_user_regs();
    register_t *r = select_user_reg(regs, dabt.reg);
    struct vgic_irq_rank *rank;
    unsigned long flags;
    uint64_t new_irouter, old_irouter;
    struct vcpu *old_vcpu, *new_vcpu;
    int gicd_reg = (int)(info->gpa - v->domain->arch.vgic.dbase);

    perfc_incr(vgicd_writes);

    switch ( gicd_reg )
    {
    case GICD_CTLR:
        if ( dabt.size != DABT_WORD ) goto bad_width;

        vgic_lock(v);
        /* Only EnableGrp1A can be changed */
        if ( *r & GICD_CTLR_ENABLE_G1A )
            v->domain->arch.vgic.ctlr |= GICD_CTLR_ENABLE_G1A;
        else
            v->domain->arch.vgic.ctlr &= ~GICD_CTLR_ENABLE_G1A;
        vgic_unlock(v);

        return 1;
    case GICD_TYPER:
        /* RO -- write ignored */
        goto write_ignore_32;
    case GICD_IIDR:
        /* RO -- write ignored */
        goto write_ignore_32;
    case GICD_STATUSR:
        /* RO -- write ignored */
        goto write_ignore_32;
    case GICD_SETSPI_NSR:
        /* Message based SPI is not implemented */
        goto write_ignore_32;
    case GICD_CLRSPI_NSR:
        /* Message based SPI is not implemented */
        goto write_ignore_32;
    case GICD_SETSPI_SR:
        /* Message based SPI is not implemented */
        goto write_ignore_32;
    case GICD_CLRSPI_SR:
        /* Message based SPI is not implemented */
        goto write_ignore_32;
    case 0x020 ... 0x03c:
    case 0xc000 ... 0xffcc:
        /* Implementation defined -- write ignored */
        printk(XENLOG_G_DEBUG
               "%pv: vGICD: WI on implementation defined register offset %#08x\n",
               v, gicd_reg);
        goto write_ignore_32;
    case GICD_IGROUPR ... GICD_IGROUPRN:
    case GICD_ISENABLER ... GICD_ISENABLERN:
    case GICD_ICENABLER ... GICD_ICENABLERN:
    case GICD_ISPENDR ... GICD_ISPENDRN:
    case GICD_ICPENDR ... GICD_ICPENDRN:
    case GICD_ISACTIVER ... GICD_ISACTIVERN:
    case GICD_ICACTIVER ... GICD_ICACTIVERN:
    case GICD_IPRIORITYR ... GICD_IPRIORITYRN:
    case GICD_ICFGR ... GICD_ICFGRN:
        /* Above registers are common with GICR and GICD
         * Manage in common */
        return __vgic_v3_distr_common_mmio_write("vGICD", v, info, gicd_reg);
    case GICD_IROUTER ... GICD_IROUTER31:
        /* SGI/PPI is RES0 */
        goto write_ignore_64;
    case GICD_IROUTER32 ... GICD_IROUTERN:
        if ( !vgic_reg64_check_access(dabt) ) goto bad_width;
        rank = vgic_rank_offset(v, 64, gicd_reg - GICD_IROUTER,
                                DABT_DOUBLE_WORD);
        if ( rank == NULL ) goto write_ignore;
        vgic_lock_rank(v, rank, flags);

        old_irouter = rank->v3.irouter[REG_RANK_INDEX(64,
                                       (gicd_reg - GICD_IROUTER),
                                       DABT_DOUBLE_WORD)];
        new_irouter = old_irouter;
        vgic_reg64_write(&new_irouter, *r, info);

        old_vcpu = vgic_v3_irouter_to_vcpu(v->domain, old_irouter);
        new_vcpu = vgic_v3_irouter_to_vcpu(v->domain, new_irouter);

        if ( !new_vcpu )
        {
            printk(XENLOG_G_DEBUG
                   "%pv: vGICD: wrong irouter at offset %#08x val %#"PRIregister,
                   v, gicd_reg, *r);
            vgic_unlock_rank(v, rank, flags);
            /*
             * TODO: Don't inject a fault to the guest when the MPIDR is
             * not valid. From the spec, the interrupt should be
             * ignored.
             */
            return 0;
        }
        rank->v3.irouter[REG_RANK_INDEX(64, (gicd_reg - GICD_IROUTER),
                         DABT_DOUBLE_WORD)] = new_irouter;
        if ( old_vcpu != new_vcpu )
            vgic_migrate_irq(old_vcpu, new_vcpu, (gicd_reg - GICD_IROUTER)/8);
        vgic_unlock_rank(v, rank, flags);
        return 1;
    case GICD_NSACR ... GICD_NSACRN:
        /* We do not implement security extensions for guests, write ignore */
        goto write_ignore_32;
    case GICD_SGIR:
        /* it is accessed as system register in GICv3 */
        goto write_ignore_32;
    case GICD_CPENDSGIR ... GICD_CPENDSGIRN:
        /* Replaced with GICR_ICPENDR0. So ignore write */
        if ( dabt.size != DABT_WORD ) goto bad_width;
        return 0;
    case GICD_SPENDSGIR ... GICD_SPENDSGIRN:
        /* Replaced with GICR_ISPENDR0. So ignore write */
        if ( dabt.size != DABT_WORD ) goto bad_width;
        return 0;
    case GICD_PIDR7... GICD_PIDR0:
        /* RO -- write ignore */
        goto write_ignore_32;
    case 0x00c:
    case 0x044:
    case 0x04c:
    case 0x05c ... 0x07c:
    case 0xf30 ... 0x5fcc:
    case 0x8000 ... 0xbfcc:
        /* Reserved register addresses */
        printk(XENLOG_G_DEBUG
               "%pv: vGICD: write unknown 0x00c 0xfcc  r%d offset %#08x\n",
               v, dabt.reg, gicd_reg);
        goto write_ignore;
    default:
        printk(XENLOG_G_ERR
               "%pv: vGICD: unhandled write r%d=%"PRIregister" offset %#08x\n",
               v, dabt.reg, *r, gicd_reg);
        return 0;
    }

bad_width:
    printk(XENLOG_G_ERR
           "%pv: vGICD: bad write width %d r%d=%"PRIregister" offset %#08x\n",
           v, dabt.size, dabt.reg, *r, gicd_reg);
    domain_crash_synchronous();
    return 0;

write_ignore_32:
    if ( dabt.size != DABT_WORD ) goto bad_width;
    return 1;

write_ignore_64:
    if ( vgic_reg64_check_access(dabt) ) goto bad_width;
    return 1;

write_ignore:
    return 1;
}

static int vgic_v3_to_sgi(struct vcpu *v, register_t sgir)
{
    int virq;
    int irqmode;
    enum gic_sgi_mode sgi_mode;
    struct sgi_target target;

    irqmode = (sgir >> ICH_SGI_IRQMODE_SHIFT) & ICH_SGI_IRQMODE_MASK;
    virq = (sgir >> ICH_SGI_IRQ_SHIFT ) & ICH_SGI_IRQ_MASK;

    /* Map GIC sgi value to enum value */
    switch ( irqmode )
    {
    case ICH_SGI_TARGET_LIST:
        sgi_target_init(&target);
        /* We assume that only AFF1 is used in ICC_SGI1R_EL1. */
        target.aff1 = (sgir >> ICH_SGI_AFFINITY_LEVEL(1)) & ICH_SGI_AFFx_MASK;
        target.list = sgir & ICH_SGI_TARGETLIST_MASK;
        sgi_mode = SGI_TARGET_LIST;
        break;
    case ICH_SGI_TARGET_OTHERS:
        sgi_mode = SGI_TARGET_OTHERS;
        break;
    default:
        gprintk(XENLOG_WARNING, "Wrong irq mode in SGI1R_EL1 register\n");
        return 0;
    }

    return vgic_to_sgi(v, sgir, sgi_mode, virq, &target);
}

static int vgic_v3_emulate_sysreg(struct cpu_user_regs *regs, union hsr hsr)
{
    struct vcpu *v = current;
    struct hsr_sysreg sysreg = hsr.sysreg;
    register_t *r = select_user_reg(regs, sysreg.reg);

    ASSERT (hsr.ec == HSR_EC_SYSREG);

    if ( sysreg.read )
        perfc_incr(vgic_sysreg_reads);
    else
        perfc_incr(vgic_sysreg_writes);

    switch ( hsr.bits & HSR_SYSREG_REGS_MASK )
    {
    case HSR_SYSREG_ICC_SGI1R_EL1:
        /* WO */
        if ( !sysreg.read )
            return vgic_v3_to_sgi(v, *r);
        else
        {
            gprintk(XENLOG_WARNING, "Reading SGI1R_EL1 - WO register\n");
            return 0;
        }
    default:
        return 0;
    }
}

static const struct mmio_handler_ops vgic_rdistr_mmio_handler = {
    .read_handler  = vgic_v3_rdistr_mmio_read,
    .write_handler = vgic_v3_rdistr_mmio_write,
};

static const struct mmio_handler_ops vgic_distr_mmio_handler = {
    .read_handler  = vgic_v3_distr_mmio_read,
    .write_handler = vgic_v3_distr_mmio_write,
};

static int vgic_v3_get_irq_priority(struct vcpu *v, unsigned int irq)
{
    int priority = 0;
    unsigned long flags;
    struct vgic_irq_rank *rank;

    if ( !gic_is_lpi(irq) )
    {
        rank = vgic_rank_irq(v, irq);

        vgic_lock_rank(v, rank, flags);
        priority = vgic_byte_read(rank->ipriority[REG_RANK_INDEX(8,
                                              irq, DABT_WORD)], 0, irq & 0x3);
        vgic_unlock_rank(v, rank, flags);
    }
    else if ( vgic_is_domain_lpi(v->domain, irq) )
    {
        struct vgic_its *vits = v->domain->arch.vgic.vits;

        /*
         * Guest can receive LPI before availability of LPI property table.
         * Hence assert is wrong.
         * TODO: Handle LPI which is valid and does not have LPI property
         * table entry and remove below assert.
         */
        ASSERT(irq  < vits->prop_size);

        spin_lock_irqsave(&vits->prop_lock, flags);
        /*
         * LPI property table always starts from 0 and LPI information
         * is availeble above 8192 index.
         * So, We don't subtract FIRST_GIC_LPI from irq value.
         */
        priority = *((u8*)vits->prop_page + irq);
        /*
         * Bits[7:2] specify priority with bits[1:0] of priority
         * is set to zero. Hence only mask bits[7:2]
         */
        priority &= LPI_PRIORITY_MASK;
        spin_unlock_irqrestore(&vits->prop_lock, flags);
    }

    return priority;
}

static int vgic_v3_vcpu_init(struct vcpu *v)
{
    int i;
    uint64_t affinity;
    paddr_t rdist_base;
    struct vgic_rdist_region *region;
    unsigned int last_cpu;

    /* Convenient alias */
    struct domain *d = v->domain;
    uint32_t rdist_stride = d->arch.vgic.rdist_stride;

    /* For SGI and PPI the target is always this CPU */
    affinity = (MPIDR_AFFINITY_LEVEL(v->arch.vmpidr, 3) << 32 |
                MPIDR_AFFINITY_LEVEL(v->arch.vmpidr, 2) << 16 |
                MPIDR_AFFINITY_LEVEL(v->arch.vmpidr, 1) << 8  |
                MPIDR_AFFINITY_LEVEL(v->arch.vmpidr, 0));

    for ( i = 0 ; i < 32 ; i++ )
        v->arch.vgic.private_irqs->v3.irouter[i] = affinity;

    /*
     * Find the region where the re-distributor lives. For this purpose,
     * we look one region ahead as we have only the first CPU in hand.
     */
    for ( i = 1; i < d->arch.vgic.nr_regions; i++ )
    {
        if ( v->vcpu_id < d->arch.vgic.rdist_regions[i].first_cpu )
            break;
    }

    region = &d->arch.vgic.rdist_regions[i - 1];

    /* Get the base address of the redistributor */
    rdist_base = region->base;
    rdist_base += (v->vcpu_id - region->first_cpu) * rdist_stride;

    /* Check if a valid region was found for the re-distributor */
    if ( (rdist_base < region->base) ||
         ((rdist_base + rdist_stride) > (region->base + region->size)) )
    {
        dprintk(XENLOG_ERR,
                "d%u: Unable to find a re-distributor for VCPU %u\n",
                d->domain_id, v->vcpu_id);
        return -EINVAL;
    }

    v->arch.vgic.rdist_base = rdist_base;

    /*
     * If the redistributor is the last one of the
     * contiguous region of the vCPU is the last of the domain, set
     * VGIC_V3_RDIST_LAST flags.
     * Note that we are assuming max_vcpus will never change.
     */
    last_cpu = (region->size / rdist_stride) + region->first_cpu - 1;

    if ( v->vcpu_id == last_cpu || (v->vcpu_id == (d->max_vcpus - 1)) )
        v->arch.vgic.flags |= VGIC_V3_RDIST_LAST;

    return 0;
}

static int vgic_v3_domain_init(struct domain *d)
{
    int i, idx;

    /*
     * Domain 0 gets the hardware address.
     * Guests get the virtual platform layout.
     */
    if ( is_hardware_domain(d) )
    {
        unsigned int first_cpu = 0;

        d->arch.vgic.dbase = vgic_v3_hw.dbase;

        d->arch.vgic.rdist_stride = vgic_v3_hw.rdist_stride;
        /*
         * If the stride is not set, the default stride for GICv3 is 2 * 64K:
         *     - first 64k page for Control and Physical LPIs
         *     - second 64k page for Control and Generation of SGIs
         */
        if ( !d->arch.vgic.rdist_stride )
            d->arch.vgic.rdist_stride = 2 * SZ_64K;

        for ( i = 0; i < vgic_v3_hw.nr_rdist_regions; i++ )
        {
            paddr_t size = vgic_v3_hw.regions[i].size;

            d->arch.vgic.rdist_regions[i].base = vgic_v3_hw.regions[i].base;
            d->arch.vgic.rdist_regions[i].size = size;

            /* Set the first CPU handled by this region */
            d->arch.vgic.rdist_regions[i].first_cpu = first_cpu;

            first_cpu += size / d->arch.vgic.rdist_stride;
        }
        d->arch.vgic.nr_regions = vgic_v3_hw.nr_rdist_regions;
    }
    else
    {
        d->arch.vgic.dbase = GUEST_GICV3_GICD_BASE;

        /* XXX: Only one Re-distributor region mapped for the guest */
        BUILD_BUG_ON(GUEST_GICV3_RDIST_REGIONS != 1);

        d->arch.vgic.nr_regions = GUEST_GICV3_RDIST_REGIONS;
        d->arch.vgic.rdist_stride = GUEST_GICV3_RDIST_STRIDE;

        /* The first redistributor should contain enough space for all CPUs */
        BUILD_BUG_ON((GUEST_GICV3_GICR0_SIZE / GUEST_GICV3_RDIST_STRIDE) < MAX_VIRT_CPUS);
        d->arch.vgic.rdist_regions[0].base = GUEST_GICV3_GICR0_BASE;
        d->arch.vgic.rdist_regions[0].size = GUEST_GICV3_GICR0_SIZE;
        d->arch.vgic.rdist_regions[0].first_cpu = 0;
    }

    /* By default deliver to CPU0 */
    for ( i = 0; i < DOMAIN_NR_RANKS(d); i++ )
    {
        for ( idx = 0; idx < 32; idx++ )
            d->arch.vgic.shared_irqs[i].v3.irouter[idx] = 0;
    }

    /* Register mmio handle for the Distributor */
    register_mmio_handler(d, &vgic_distr_mmio_handler, d->arch.vgic.dbase,
                          SZ_64K);

    /*
     * Register mmio handler per contiguous region occupied by the
     * redistributors. The handler will take care to choose which
     * redistributor is targeted.
     */
    for ( i = 0; i < d->arch.vgic.nr_regions; i++ )
        register_mmio_handler(d, &vgic_rdistr_mmio_handler,
            d->arch.vgic.rdist_regions[i].base,
            d->arch.vgic.rdist_regions[i].size);

    d->arch.vgic.ctlr = VGICD_CTLR_DEFAULT;

    if ( is_hardware_domain(d) && gic_lpi_supported() )
        vgic_v3_info.its_enabled = 1;

    return 0;
}

static const struct vgic_ops v3_ops = {
    .info = &vgic_v3_info,
    .vcpu_init   = vgic_v3_vcpu_init,
    .domain_init = vgic_v3_domain_init,
    .get_irq_priority = vgic_v3_get_irq_priority,
    .get_target_vcpu  = vgic_v3_get_target_vcpu,
    .emulate_sysreg  = vgic_v3_emulate_sysreg,
    /*
     * We use both AFF1 and AFF0 in (v)MPIDR. Thus, the max number of CPU
     * that can be supported is up to 4096(==256*16) in theory.
     */
    .max_vcpus = 4096,
};

int vgic_v3_init(struct domain *d)
{
    if ( !vgic_v3_hw.enabled )
    {
        printk(XENLOG_G_ERR
               "d%d: vGICv3 is not supported on this platform.\n",
               d->domain_id);
        return -ENODEV;
    }

    register_vgic_ops(d, &v3_ops);

    return 0;
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
