/*
 * Copyright (C) 2015 Cavium Inc.
 * Vijaya Kumar K <Vijaya.Kumar@caviumnetworks.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __ASM_ARM_GIC_ITS_H__
#define __ASM_ARM_GIC_ITS_H__

#include <asm/gic_v3_defs.h>
#include <xen/rbtree.h>

#define MAPC_ITT_IPA_SHIFT              8
/*
 * ITS registers, offsets from ITS_base
 */
#define GITS_CTLR                       0x0000
#define GITS_IIDR                       0x0004
#define GITS_TYPER                      0x0008
#define GITS_CBASER                     0x0080
#define GITS_CWRITER                    0x0088
#define GITS_CREADR                     0x0090
#define GITS_BASER0                     0x0100
#define GITS_BASER1                     0x0108
#define GITS_BASER                      0x0100
#define GITS_BASERN                     0x013c
#define GITS_PIDR0                      GICR_PIDR0
#define GITS_PIDR1                      GICR_PIDR1
#define GITS_PIDR2                      GICR_PIDR2
#define GITS_PIDR3                      GICR_PIDR3
#define GITS_PIDR4                      GICR_PIDR4
#define GITS_PIDR5                      GICR_PIDR5
#define GITS_PIDR7                      GICR_PIDR7

#define GITS_TRANSLATER                 0x10040
#define GITS_CTLR_QUIESCENT             (1U << 31)
#define GITS_CTLR_ENABLE                (1U << 0)

#define GITS_TYPER_DEVBITS_SHIFT        13
#define GITS_TYPER_DEVBITS(r)           ((((r) >> GITS_TYPER_DEVBITS_SHIFT) & 0x1f) + 1)
#define GITS_TYPER_IDBITS_SHIFT         8
#define GITS_TYPER_IDBITS(r)		((((r) >> GITS_TYPER_IDBITS_SHIFT) & 0x1f) + 1)
#define GITS_TYPER_PTA                  (1UL << 19)
#define GITS_TYPER_HCC_SHIFT            (24)
#define GITS_TYPER_PHYSICAL_LPIS        (1UL)
#define GITS_TYPER_ITT_SIZE_SHIFT       (4)

#define GITS_CBASER_VALID               (1UL << 63)
#define GITS_CBASER_nC                  (1UL << 59)
#define GITS_CBASER_WaWb                (5UL << 59)
#define GITS_CBASER_InnerShareable      (1UL << 10)
#define GITS_CBASER_SHAREABILITY_MASK   (3UL << 10)
#define GITS_CBASER_CACHEABILITY_MASK   (7UL << 59)
#define GITS_CBASER_PA_MASK             (0xfffffffff000UL)

#define GITS_BASER_NR_REGS              8

#define GITS_BASER_VALID                (1UL << 63)
#define GITS_BASER_INDIRECT_SHIFT       (62)
#define GITS_BASER_nC                   (1UL << 59)
#define GITS_BASER_WaWb                 (5UL << 59)
#define GITS_BASER_TYPE_SHIFT           (56)
#define GITS_BASER_TYPE_MASK            (0x7)
#define GITS_BASER_TYPE(r)              (((r) >> GITS_BASER_TYPE_SHIFT) & 7)
#define GITS_BASER_ENTRY_SIZE_SHIFT     (48)
#define GITS_BASER_ENTRY_SIZE(r)        ((((r) >> GITS_BASER_ENTRY_SIZE_SHIFT) & 0xff) + 1)
#define GITS_BASER_InnerShareable       (1UL << 10)
#define GITS_BASER_SHAREABILITY_SHIFT   (10)
#define GITS_BASER_SHAREABILITY_MASK    (3UL << GITS_BASER_SHAREABILITY_SHIFT)
#define GITS_BASER_PAGE_SIZE_SHIFT      (8)
#define GITS_BASER_PAGE_SIZE_4K         (0UL << GITS_BASER_PAGE_SIZE_SHIFT)
#define GITS_BASER_PAGE_SIZE_16K        (1UL << GITS_BASER_PAGE_SIZE_SHIFT)
#define GITS_BASER_PAGE_SIZE_64K        (2UL << GITS_BASER_PAGE_SIZE_SHIFT)
#define GITS_BASER_PAGE_SIZE_MASK       (3UL << GITS_BASER_PAGE_SIZE_SHIFT)
#define GITS_BASER_PAGE_SIZE_4K_VAL     (0)
#define GITS_BASER_PAGE_SIZE_16K_VAL    (1)
#define GITS_BASER_PAGE_SIZE_MASK_VAL   (0x3)
#define GITS_BASER_PAGES_MASK_VAL       (0xff)
#define GITS_BASER_TYPE_NONE            0
#define GITS_BASER_TYPE_DEVICE          1
#define GITS_BASER_TYPE_VCPU            2
#define GITS_BASER_TYPE_CPU             3
#define GITS_BASER_TYPE_COLLECTION      4
#define GITS_BASER_TYPE_RESERVED5       5
#define GITS_BASER_TYPE_RESERVED6       6
#define GITS_BASER_TYPE_RESERVED7       7
#define GITS_BASER_PA_MASK              (0xfffffffff000UL)

/*
 * ITS commands
 */
#define GITS_CMD_MAPD                   0x08
#define GITS_CMD_MAPC                   0x09
#define GITS_CMD_MAPVI                  0x0a
#define GITS_CMD_MAPI                   0x0b
#define GITS_CMD_MOVI                   0x01
#define GITS_CMD_DISCARD                0x0f
#define GITS_CMD_INV                    0x0c
#define GITS_CMD_MOVALL                 0x0e
#define GITS_CMD_INVALL                 0x0d
#define GITS_CMD_INT                    0x03
#define GITS_CMD_CLEAR                  0x04
#define GITS_CMD_SYNC                   0x05

#define LPI_PROP_ENABLED                (1 << 0)
#define LPI_PROP_GROUP1                 (1 << 1)

/*
 * Collection structure - just an ID, and a redistributor address to
 * ping. We use one per CPU as collection of interrupts assigned to this
 * CPU.
 */
struct its_collection {
    u64 target_address;
    u16 col_id;
};

/*
 * Per domain virtual ITS structure.
 */
struct vgic_its
{
   spinlock_t lock;
   /* Emulation of BASER0 */
   paddr_t baser0;
   /* Command queue base */
   paddr_t cmd_base;
   /* Command queue write pointer */
   paddr_t cmd_write;
   /* Command queue read pointer */
   atomic_t cmd_read;
   /* Command queue size */
   unsigned long cmd_qsize;
   /* ITS mmio physical base */
   paddr_t gits_base;
   /* ITS mmio physical size */
   unsigned long gits_size;
   /* GICR ctrl register */
   uint32_t ctrl;
   /* vITT device table ipa */
   paddr_t dt_ipa;
   /* vITT device table size */
   uint64_t dt_size;
   /* collections mapped */
   struct its_collection *collections;
};

/* ITS command structure */
typedef union {
    u64 bits[4];
    struct __packed {
        uint8_t cmd;
        uint8_t pad[7];
    } hdr;
    struct __packed {
        u8 cmd;
        u8 res1[3];
        u32 devid;
        u64 size:5;
        u64 res2:59;
        /* XXX: Though itt is 40 bit. Keep it 48 to avoid shift */
        u64 res3:8;
        u64 itt:40;
        u64 res4:15;
        u64 valid:1;
        u64 res5;
    } mapd;
    struct __packed {
        u8 cmd;
        u8 res1[7];
        u64 res2;
        u16 col;
        u32 ta;
        u16 res3:15;
        u16 valid:1;
        u64 res4;
    } mapc;
    struct __packed {
        u8 cmd;
        u8 res1[3];
        u32 devid;
        u32 event;
        u32 res2;
        u16 col;
        u8 res3[6];
        u64 res4;
    } mapi;
    struct __packed {
        u8 cmd;
        u8 res1[3];
        u32 devid;
        u32 event;
        u32 phy_id;
        u16 col;
        u8 res2[6];
        u64 res3;
    } mapvi;
    struct __packed {
        u8 cmd;
        u8 res1[3];
        u32 devid;
        u32 event;
        u32 res2;
        u16 col;
        u8 res3[6];
        u64 res4;
    } movi;
    struct __packed {
        u8 cmd;
        u8 res1[3];
        u32 devid;
        u32 event;
        u32 res2;
        u64 res3;
        u64 res4;
    } discard;
    struct __packed {
        u8 cmd;
        u8 res1[3];
        u32 devid;
        u32 event;
        u32 res2;
        u64 res3;
        u64 res4;
    } inv;
    struct __packed {
        u8 cmd;
        u8 res1[7];
        u64 res2;
        u16 res3;
        u32 ta1;
        u16 res4;
        u16 res5;
        u32 ta2;
        u16 res6;
    } movall;
    struct __packed {
        u8 cmd;
        u8 res1[7];
        u64 res2;
        u16 col;
        u8 res3[6];
        u64 res4;
    } invall;
    struct __packed {
        u8 cmd;
        u8 res1[3];
        u32 devid;
        u32 event;
        u32 res2;
        u64 res3;
        u64 res4;
    } int_cmd;
    struct __packed {
        u8 cmd;
        u8 res1[3];
        u32 devid;
        u32 event;
        u32 res2;
        u64 res3;
        u64 res4;
    } clear;
    struct __packed {
        u8 cmd;
        u8 res1[7];
        u64 res2;
        u16 res3;
        u32 ta;
        u16 res4;
        u64 res5;
    } sync;
} its_cmd_block;

struct event_lpi_map {
    /* Physical LPI map */
    unsigned long *lpi_map;
    /* Collection map */
    u16           *col_map;
    /* First Physical LPI number assigned */
    u32           lpi_base;
    /* Number of ITES/Physical LPIs assigned */
    u32           nr_lpis;
};

/*
 * The ITS view of a device.
 */
struct its_device {
    /* Physical ITS */
    struct its_node         *its;
    /* Device ITT address */
    paddr_t                 *itt_addr;
    /* Device ITT size */
    unsigned long           itt_size;
    /* LPI and event mapping */
    struct event_lpi_map    event_map;
    /* Physical Device id */
    u32                     device_id;
    /* Virtual Device id */
    u32                     virt_device_id;
    /* Domain id */
    int                     domain_id;
    /* RB-tree entry */
    struct rb_node          node;
};

/*
 * struct vdevice_table and struct vitt are typically stored in memory
 * which has been provided by the guest out of its own address space
 * and which remains accessible to the guest.
 *
 * Therefore great care _must_ be taken when accessing an entry in
 * either table to validate the sanity of any values which are used.
 */
struct vdevice_table {
    uint64_t vitt_ipa;
    uint32_t vitt_size;
    uint32_t padding;
};

struct vitt {
    uint16_t valid:1;
    uint16_t pad:15;
    uint16_t vcollection;
    uint32_t vlpi;
};

struct gic_its_info {
    uint32_t eventid_bits;
    uint32_t dev_bits;
};

void irqdesc_set_lpi_event(struct irq_desc *desc, unsigned id);
unsigned int irqdesc_get_lpi_event(struct irq_desc *desc);
struct its_device *irqdesc_get_its_device(struct irq_desc *desc);
void irqdesc_set_its_device(struct irq_desc *desc, struct its_device *dev);
bool_t is_valid_collection(struct domain *d, uint32_t col);
unsigned int its_get_nr_event_ids(void);
int its_init(struct rdist_prop *rdists);
int its_cpu_init(void);
int its_add_device(u32 devid, u32 nr_ites, struct dt_device_node *dt_its);
int its_assign_device(struct domain *d, u32 vdevid, u32 pdevid);
int vits_access_guest_table(struct domain *d, paddr_t entry, void *addr,
                            uint32_t size, bool_t set);
int vits_domain_init(struct domain *d);
void vits_domain_free(struct domain *d);
int vits_get_vitt_entry(struct domain *d, uint32_t devid,
                        uint32_t event, struct vitt *entry);
int vits_get_vdevice_entry(struct domain *d, uint32_t devid,
                           struct vdevice_table *entry);
void vits_setup_hw(uint32_t dev_bits, uint32_t eventid_bits,
                   paddr_t base, unsigned long size);

#endif /* __ASM_ARM_GIC_ITS_H__ */
/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
