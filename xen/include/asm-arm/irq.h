#ifndef _ASM_HW_IRQ_H
#define _ASM_HW_IRQ_H

#include <xen/config.h>
#include <xen/device_tree.h>

#define NR_VECTORS 256 /* XXX */

typedef struct {
    DECLARE_BITMAP(_bits,NR_VECTORS);
} vmask_t;

struct arch_pirq
{
};

struct arch_irq_desc {
    unsigned int type;
};

struct msi_desc {
#ifdef CONFIG_HAS_GICV3
    unsigned int eventID;
    struct its_device *dev;
#endif
};

#define NR_LOCAL_IRQS	32
/* Number of SGIs + PPIs + SPIs */
#define NR_ITLINES	1024

#define nr_irqs NR_ITLINES
#define nr_static_irqs NR_ITLINES
#define arch_hwdom_irqs(domid) NR_ITLINES

struct irq_desc;
struct irqaction;

struct irq_desc *__irq_to_desc(int irq);

#define irq_to_desc(irq)    __irq_to_desc(irq)

void do_IRQ(struct cpu_user_regs *regs, unsigned int irq, int is_fiq);

#define domain_pirq_to_irq(d, pirq) (pirq)

bool_t is_assignable_irq(unsigned int irq);

void init_IRQ(void);
void init_secondary_IRQ(void);

int route_lpi_to_guest(struct domain *d, unsigned int irq,
                       const char *devname);
int route_irq_to_guest(struct domain *d, unsigned int virq,
                       unsigned int irq, const char *devname);
int release_guest_irq(struct domain *d, unsigned int irq);

void arch_move_irqs(struct vcpu *v);

#define arch_evtchn_bind_pirq(d, pirq) ((void)((d) + (pirq)))

/* Set IRQ type for an SPI */
int irq_set_spi_type(unsigned int spi, unsigned int type);

int platform_get_irq(const struct dt_device_node *device, int index);

void irq_set_affinity(struct irq_desc *desc, const cpumask_t *cpu_mask);
void irq_set_msi_desc(struct irq_desc *desc, struct msi_desc *msi);
struct msi_desc *irq_get_msi_desc(struct irq_desc *desc);
int init_lpi(void);

#endif /* _ASM_HW_IRQ_H */
/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
