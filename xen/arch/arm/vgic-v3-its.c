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

#include <xen/bitops.h>
#include <xen/config.h>
#include <xen/init.h>
#include <xen/irq.h>
#include <xen/list.h>
#include <xen/sched.h>
#include <xen/sizes.h>
#include <xen/domain_page.h>
#include <xen/delay.h>
#include <asm/device.h>
#include <asm/mmio.h>
#include <asm/io.h>
#include <asm/atomic.h>
#include <asm/gic_v3_defs.h>
#include <asm/gic.h>
#include <asm/gic-its.h>
#include <asm/vgic.h>
#include <asm/gic-its.h>
#include <asm/vits.h>
#include <xen/log2.h>
#include <asm/vgic-emul.h>

//#define DEBUG_ITS

/* GITS_PIDRn register values for ARM implementations */
#define GITS_PIDR0_VAL               (0x94)
#define GITS_PIDR1_VAL               (0xb4)
#define GITS_PIDR2_VAL               (0x3b)
#define GITS_PIDR3_VAL               (0x00)
#define GITS_PIDR4_VAL               (0x04)

/*
 * GITS_BASER0 is used to allocate memory for the Device table.
 * Some of the fields GITS_BASER0.Type and GITS_BASER0.Entry_size
 * are read-only. Initialize GITS_BASER0_Type = 1 ( Device )
 * and GITS_BASER0.Entry_size = size of vitt structure (vitt)-1.
 */
#define GITS_BASER0_INIT_VAL         ((1UL << GITS_BASER_TYPE_SHIFT) | \
                                      ((sizeof(struct vitt) - 1) <<    \
                                      GITS_BASER_ENTRY_SIZE_SHIFT))

#ifdef DEBUG_ITS
# define DPRINTK(fmt, args...) dprintk(XENLOG_DEBUG, fmt, ##args)
#else
# define DPRINTK(fmt, args...) do {} while ( 0 )
#endif

#ifdef DEBUG_ITS
static void dump_cmd(const its_cmd_block *cmd)
{
    printk("VITS:CMD[0] = 0x%lx CMD[1] = 0x%lx CMD[2] = 0x%lx CMD[3] = 0x%lx\n",
           cmd->bits[0], cmd->bits[1], cmd->bits[2], cmd->bits[3]);
}
#else
static void dump_cmd(const its_cmd_block *cmd) { }
#endif

static struct {
    bool_t enabled;
    uint8_t devID_bits;
    uint8_t eventID_bits;
    /* GITS physical base */
    paddr_t phys_base;
    /* GITS physical size */
    unsigned long phys_size;
} vits_hw;

void vits_setup_hw(uint8_t devID_bits, uint8_t eventID_bits,
                   paddr_t phys_base, unsigned long phys_size)
{
    vits_hw.enabled = 1;
    vits_hw.devID_bits = devID_bits;
    vits_hw.eventID_bits = eventID_bits;
    vits_hw.phys_base = phys_base;
    vits_hw.phys_size = phys_size;
}

static inline uint16_t vits_get_max_collections(struct domain *d)
{
    /*
     * ITS only supports up to 256 collections without
     * provisioning external memory. As per vITS design, number of
     * vCPUS should not exceed max number of collections.
     */
    ASSERT(d->max_vcpus < 256);

    /*
     * Each collection corresponds to one CPU(vCPU). Collections are
     * used to move interrupts from one CPU to another. The ITS
     * mandates to implement N + 1 collections where N is the number of
     * processor on the platform (i.e max number of VCPUs for a given
     * guest).
     * Refer to PRD03-GENC-010745 24 section 4.9.15.
     */
    return (d->max_vcpus + 1);
}

bool_t is_valid_collection(struct domain *d, uint16_t col)
{
    return (col <= vits_get_max_collections(d));
}

/* ITS device table helper functions */
static int vits_vdevice_entry(struct domain *d, uint32_t dev_id,
                              struct vdevice_table *entry, bool_t set)
{
    uint64_t offset;
    paddr_t dt_entry;
    const struct vgic_its *vits = d->arch.vgic.vits;

    BUILD_BUG_ON(sizeof(struct vdevice_table) != 16);

    offset = dev_id * sizeof(struct vdevice_table);
    if ( offset > vits->dt_size )
    {
        printk(XENLOG_G_ERR
               "d%d: vITS: Out of range off 0x%"PRIx64" id 0x%"PRIx32"\n",
               d->domain_id, offset, dev_id);
        return -EINVAL;
    }

    dt_entry = vits->dt_ipa + offset;

    return vgic_access_guest_memory(d, dt_entry, entry,
                                   sizeof(struct vdevice_table), set);
}

static int vits_set_vdevice_entry(struct domain *d, uint32_t devid,
                                  struct vdevice_table *entry)
{
    return vits_vdevice_entry(d, devid, entry, 1);
}

int vits_get_vdevice_entry(struct domain *d, uint32_t devid,
                           struct vdevice_table *entry)
{
    return vits_vdevice_entry(d, devid, entry, 0);
}

static int vits_vitt_entry(struct domain *d, uint32_t devid,
                           uint32_t event, struct vitt *entry, bool_t set)
{
    struct vdevice_table dt_entry;
    paddr_t vitt_entry;
    uint64_t offset;

    BUILD_BUG_ON(sizeof(struct vitt) != 8);

    if ( vits_get_vdevice_entry(d, devid, &dt_entry) )
    {
        printk(XENLOG_G_ERR
               "d%d: vITS: Fail to get vdevice for vdevid 0x%"PRIx32"\n",
               d->domain_id, devid);
        return -EINVAL;
    }

    /* dt_entry has been validated in vits_get_vdevice_entry */
    offset = event * sizeof(struct vitt);
    if ( offset > dt_entry.vitt_size )
    {
        printk(XENLOG_G_ERR "d%d: vITS: ITT out of range\n", d->domain_id);
        return -EINVAL;
    }

    vitt_entry = dt_entry.vitt_ipa + offset;

    return vgic_access_guest_memory(d, vitt_entry, entry,
                                   sizeof(struct vitt), set);
}

static int vits_set_vitt_entry(struct domain *d, uint32_t devid,
                               uint32_t event, struct vitt *entry)
{
    return vits_vitt_entry(d, devid, event, entry, 1);
}

int vits_get_vitt_entry(struct domain *d, uint32_t devid,
                        uint32_t event, struct vitt *entry)
{
    return vits_vitt_entry(d, devid, event, entry, 0);
}

static int vits_process_sync(struct vcpu *v, struct vgic_its *vits,
                             its_cmd_block *virt_cmd)
{
    /* Ignored */
    DPRINTK("%pv: vITS: SYNC: ta 0x%"PRIx32" \n", v, virt_cmd->sync.target_addr);

    return 0;
}

static int vits_process_mapvi(struct vcpu *v, struct vgic_its *vits,
                              its_cmd_block *virt_cmd)
{
    struct vitt entry;
    struct domain *d = v->domain;
    uint16_t vcol_id;
    uint8_t cmd;
    uint32_t vid, dev_id, event;

    vcol_id = virt_cmd->mapvi.col;
    vid = virt_cmd->mapvi.phy_id;
    cmd = virt_cmd->mapvi.cmd;
    dev_id = virt_cmd->mapvi.devid;

    DPRINTK("%pv: vITS: MAPVI: dev 0x%"PRIx32" vcol %"PRIu16" vid %"PRIu32"\n",
             v, dev_id, vcol_id, vid);

    entry.valid = true;
    entry.vcollection = vcol_id;
    entry.vlpi = vid;

    if ( cmd == GITS_CMD_MAPI )
        vits_set_vitt_entry(d, dev_id, vid, &entry);
    else
    {
        event = virt_cmd->mapvi.event;
        vits_set_vitt_entry(d, dev_id, event, &entry);
    }

    return 0;
}

static int vits_process_movi(struct vcpu *v, struct vgic_its *vits,
                             its_cmd_block *virt_cmd)
{
    struct vitt entry;
    struct domain *d = v->domain;
    uint32_t dev_id, event;
    uint16_t vcol_id;

    vcol_id = virt_cmd->movi.col;
    event = virt_cmd->movi.event;
    dev_id = virt_cmd->movi.devid;

    DPRINTK("%pv vITS: MOVI: dev_id 0x%"PRIx32" vcol %"PRIu16" event %"PRIu32"\n",
            v, dev_id, vcol_id, event);

    if ( vits_get_vitt_entry(d, dev_id, event, &entry) )
        return -EINVAL;

    entry.vcollection = vcol_id;

    if ( vits_set_vitt_entry(d, dev_id, event, &entry) )
        return -EINVAL;

    return 0;
}

static int vits_process_movall(struct vcpu *v, struct vgic_its *vits,
                               its_cmd_block *virt_cmd)
{
    /* Ignored */
    DPRINTK("%pv: vITS: MOVALL: ta1 0x%"PRIx32" ta2 0x%"PRIx32" \n",
            v, virt_cmd->movall.target_addr1, virt_cmd->movall.target_addr2);

    return 0;
}

static int vits_process_discard(struct vcpu *v, struct vgic_its *vits,
                                its_cmd_block *virt_cmd)
{
    struct vitt entry;
    struct domain *d = v->domain;
    uint32_t event, dev_id;

    event = virt_cmd->discard.event;
    dev_id = virt_cmd->discard.devid;

    DPRINTK("%pv vITS: DISCARD: dev_id 0x%"PRIx32" id %"PRIu32"\n",
            v, virt_cmd->discard.devid, event);

    if ( vits_get_vitt_entry(d, dev_id, event, &entry) )
        return -EINVAL;

    entry.valid = false;

    if ( vits_set_vitt_entry(d, dev_id, event, &entry) )
        return -EINVAL;

    return 0;
}

static int vits_process_inv(struct vcpu *v, struct vgic_its *vits,
                            its_cmd_block *virt_cmd)
{
    unsigned long flags;
    int state;

    /* Do no complete INV command until lpi property table
     * is under update by tasklet
     */
    do {
        spin_lock_irqsave(&v->domain->arch.vgic.prop_lock, flags);
        state = v->domain->arch.vgic.lpi_prop_table_state;
        spin_unlock_irqrestore(&v->domain->arch.vgic.prop_lock, flags);
        if ( state != LPI_TAB_IN_PROGRESS )
            break;
        cpu_relax();
        udelay(1);
    } while ( 1 );

    /* Ignored */
    DPRINTK("%pv vITS: INV: dev_id 0x%"PRIx32" id %"PRIu32"\n",
            v, virt_cmd->inv.devid, virt_cmd->inv.event);

    return 0;
}

static int vits_process_clear(struct vcpu *v, struct vgic_its *vits,
                              its_cmd_block *virt_cmd)
{
    /* Ignored */
    DPRINTK("%pv: vITS: CLEAR: dev_id 0x%"PRIx32" id %"PRIu32"\n",
            v, virt_cmd->clear.devid, virt_cmd->clear.event);

    return 0;
}

static int vits_process_invall(struct vcpu *v, struct vgic_its *vits,
                               its_cmd_block *virt_cmd)
{
    /* Ignored */
    DPRINTK("%pv: vITS: INVALL: vCID %"PRIu16"\n", v, virt_cmd->invall.col);

    return 0;
}

static int vits_process_int(struct vcpu *v, struct vgic_its *vits,
                            its_cmd_block *virt_cmd)
{
    uint32_t event, dev_id;

    event = virt_cmd->int_cmd.cmd;
    dev_id = virt_cmd->int_cmd.devid;

    DPRINTK("%pv: vITS: INT: Device 0x%"PRIx32" id %"PRIu32"\n",
            v, dev_id, event);

    /* TODO: Inject LPI */

    return 0;
}

static int vits_add_device(struct vcpu *v, struct vgic_its *vits,
                           its_cmd_block *virt_cmd)
{
    struct domain *d = v->domain;
    struct vdevice_table dt_entry;
    uint32_t dev_id = virt_cmd->mapd.devid;

    DPRINTK("%pv: vITS:Add dev 0x%"PRIx32" ipa = 0x%"PRIx64" size %"PRIu32"\n",
            v, dev_id, (u64)virt_cmd->mapd.itt << MAPC_ITT_IPA_SHIFT,
            virt_cmd->mapd.size);

    if ( virt_cmd->mapd.valid )
    {
        /* itt field is 40 bit. extract 48 bit address by shifting */
        dt_entry.vitt_ipa = virt_cmd->mapd.itt << MAPC_ITT_IPA_SHIFT;
        dt_entry.vitt_size = (1 << (virt_cmd->mapd.size + 1)) *
                              sizeof(struct vitt);
    }
    else
    {
        dt_entry.vitt_ipa = INVALID_PADDR;
        dt_entry.vitt_size = 0;
    }

    if ( vits_set_vdevice_entry(d, dev_id, &dt_entry) )
        return -EINVAL;

    return 0;
}

static int vits_process_mapc(struct vcpu *v, struct vgic_its *vits,
                             its_cmd_block *virt_cmd)
{
    uint16_t vcol_id = virt_cmd->mapc.col;
    uint64_t vta = virt_cmd->mapc.ta;

    DPRINTK("%pv: vITS: MAPC: vCID %"PRIu16" vTA 0x%"PRIx64" valid %"PRIu8"\n",
            v, vcol_id, vta, virt_cmd->mapc.valid);

    if ( !is_valid_collection(v->domain, vcol_id) )
        return -EINVAL;

    if ( virt_cmd->mapc.valid )
    {
        if ( vta > v->domain->max_vcpus )
            return -EINVAL;
        vits->collections[vcol_id].target_address = vta;
    }
    else
        vits->collections[vcol_id].target_address = INVALID_PADDR;

    return 0;
}

#ifdef DEBUG_ITS
char *cmd_str[] = {
        [GITS_CMD_MOVI]    = "MOVI",
        [GITS_CMD_INT]     = "INT",
        [GITS_CMD_CLEAR]   = "CLEAR",
        [GITS_CMD_SYNC]    = "SYNC",
        [GITS_CMD_MAPD]    = "MAPD",
        [GITS_CMD_MAPC]    = "MAPC",
        [GITS_CMD_MAPVI]   = "MAPVI",
        [GITS_CMD_MAPI]    = "MAPI",
        [GITS_CMD_INV]     = "INV",
        [GITS_CMD_INVALL]  = "INVALL",
        [GITS_CMD_MOVALL]  = "MOVALL",
        [GITS_CMD_DISCARD] = "DISCARD",
    };
#endif

static int vits_parse_its_command(struct vcpu *v, struct vgic_its *vits,
                                  its_cmd_block *virt_cmd)
{
    uint8_t cmd = virt_cmd->hdr.cmd;
    int ret;

    DPRINTK("%pv: vITS: Received cmd %s (0x%"PRIx8")\n", v, cmd_str[cmd], cmd);
    dump_cmd(virt_cmd);

    switch ( cmd )
    {
    case GITS_CMD_MAPD:
        ret = vits_add_device(v, vits, virt_cmd);
        break;
    case GITS_CMD_MAPC:
        ret = vits_process_mapc(v, vits, virt_cmd);
        break;
    case GITS_CMD_MAPI:
        /* MAPI is same as MAPVI */
    case GITS_CMD_MAPVI:
        ret = vits_process_mapvi(v, vits, virt_cmd);
        break;
    case GITS_CMD_MOVI:
        ret = vits_process_movi(v, vits, virt_cmd);
        break;
    case GITS_CMD_MOVALL:
        ret = vits_process_movall(v, vits, virt_cmd);
        break;
    case GITS_CMD_DISCARD:
        ret = vits_process_discard(v, vits, virt_cmd);
        break;
    case GITS_CMD_INV:
        ret = vits_process_inv(v, vits, virt_cmd);
        break;
    case GITS_CMD_INVALL:
        ret = vits_process_invall(v, vits, virt_cmd);
        break;
    case GITS_CMD_INT:
        ret = vits_process_int(v, vits, virt_cmd);
        break;
    case GITS_CMD_CLEAR:
        ret = vits_process_clear(v, vits, virt_cmd);
        break;
    case GITS_CMD_SYNC:
        ret = vits_process_sync(v, vits, virt_cmd);
        break;
    default:
       dprintk(XENLOG_G_ERR, "%pv: vITS: Unhandled command cmd %"PRIu8"\n",
               v, cmd);
       return 1;
    }

    if ( ret )
    {
       dprintk(XENLOG_G_ERR, "%pv: vITS: Failed to handle cmd %"PRIu8"\n",
               v, cmd);
       return 1;
    }

    return 0;
}

static int vits_read_virt_cmd(struct vcpu *v, struct vgic_its *vits,
                              its_cmd_block *virt_cmd)
{
    paddr_t maddr;
    struct domain *d = v->domain;
    int ret;

    ASSERT(spin_is_locked(&vits->lock));

    if ( !(vits->cmd_base & GITS_CBASER_VALID) )
    {
        dprintk(XENLOG_G_ERR, "%pv: vITS: Invalid CBASER\n", v);
        return 0;
    }

    /* Get command queue offset */
    maddr = (vits->cmd_base & GITS_CBASER_PA_MASK) +
             atomic_read(&vits->cmd_read);

    DPRINTK("%pv: vITS: Mapping CMD Q maddr 0x%"PRIx64" read 0x%"PRIx32"\n",
            v, maddr, atomic_read(&vits->cmd_read));

    ret = vgic_access_guest_memory(d, maddr, (void *)virt_cmd,
                                   sizeof(its_cmd_block), 0);
    if ( ret )
    {
        dprintk(XENLOG_G_ERR,
                "%pv: vITS: Failed to get command page @page 0x%"PRIx32"\n",
                v, atomic_read(&vits->cmd_read));
        return -EINVAL;
    }

    atomic_add(sizeof(its_cmd_block), &vits->cmd_read);
    if ( atomic_read(&vits->cmd_read) == vits->cmd_qsize )
    {
         DPRINTK("%pv: vITS: Reset read @ 0x%"PRIx32" qsize 0x%"PRIx64"\n",
                 v, atomic_read(&vits->cmd_read), vits->cmd_qsize);

         atomic_set(&vits->cmd_read, 0);
    }

    return 0;
}

static int vits_process_cmd(struct vcpu *v, struct vgic_its *vits)
{
    its_cmd_block virt_cmd;

    ASSERT(spin_is_locked(&vits->lock));

    do {
        if ( vits_read_virt_cmd(v, vits, &virt_cmd) )
            goto err;
        if ( vits_parse_its_command(v, vits, &virt_cmd) )
            goto err;
    } while ( vits->cmd_write != atomic_read(&vits->cmd_read) );

    DPRINTK("%pv: vITS: read @ 0x%"PRIx32" write @ 0x%"PRIx64"\n",
            v, atomic_read(&vits->cmd_read),
            vits->cmd_write);

    return 1;
err:
    dprintk(XENLOG_G_ERR, "%pv: vITS: Failed to process guest cmd\n", v);
    domain_crash_synchronous();

    return 0;
}

static inline void vits_spin_lock(struct vgic_its *vits)
{
    spin_lock(&vits->lock);
}

static inline void vits_spin_unlock(struct vgic_its *vits)
{
    spin_unlock(&vits->lock);
}

static int vgic_v3_gits_mmio_read(struct vcpu *v, mmio_info_t *info,
                                  register_t *r, void *priv)
{
    struct vgic_its *vits = v->domain->arch.vgic.vits;
    struct hsr_dabt dabt = info->dabt;
    uint64_t val;
    uint32_t gits_reg;

    gits_reg = info->gpa - vits->gits_base;
    DPRINTK("%pv: vITS: GITS_MMIO_READ offset 0x%"PRIx32"\n", v, gits_reg);

    switch ( gits_reg )
    {
    case VREG32(GITS_CTLR):
        if ( dabt.size != DABT_WORD ) goto bad_width;
        vits_spin_lock(vits);
        *r = vgic_reg32_extract(vits->ctlr, info);
        vits_spin_unlock(vits);
        return 1;
    case VREG32(GITS_IIDR):
        if ( dabt.size != DABT_WORD ) goto bad_width;
        *r = vgic_reg32_extract(GICV3_GICD_IIDR_VAL, info);
        return 1;
    case VREG64(GITS_TYPER):
        /*
         * GITS_TYPER.HCC = max_vcpus + 1 (max collection supported)
         * GITS_TYPER.Devbits = HW supported Devbits size
         * GITS_TYPER.IDbits = HW supported IDbits size
         * GITS_TYPER.PTA = 0 (Target addresses are linear processor numbers)
         * GITS_TYPER.ITTSize = Size of struct vitt
         * GITS_TYPER.Physical = 1
         */
        if ( !vgic_reg64_check_access(dabt) ) goto bad_width;
        val = ((vits_get_max_collections(v->domain) << GITS_TYPER_HCC_SHIFT ) |
               ((vits_hw.devID_bits - 1) << GITS_TYPER_DEVBITS_SHIFT)         |
               ((vits_hw.eventID_bits - 1) << GITS_TYPER_IDBITS_SHIFT)        |
               ((sizeof(struct vitt) - 1) << GITS_TYPER_ITT_SIZE_SHIFT)       |
                 GITS_TYPER_PHYSICAL_LPIS);
        *r = vgic_reg64_extract(val, info);
        return 1;
    case 0x0010 ... 0x007c:
    case 0xc000 ... 0xffcc:
        /* Implementation defined -- read ignored */
        goto read_as_zero;
    case VREG64(GITS_CBASER):
        if ( !vgic_reg64_check_access(dabt) ) goto bad_width;
        vits_spin_lock(vits);
        if ( vits->ctlr & GITS_CTLR_ENABLE )
            *r = vgic_reg64_extract(vits->cmd_base, info);
        else
            *r = vgic_reg64_extract(vits->cmd_base_save, info);
        vits_spin_unlock(vits);
        return 1;
    case VREG64(GITS_CWRITER):
        if ( !vgic_reg64_check_access(dabt) ) goto bad_width;
        vits_spin_lock(vits);
        *r = vgic_reg64_extract(vits->cmd_write & 0xfffe0, info);
        vits_spin_unlock(vits);
        return 1;
    case VREG64(GITS_CREADR):
        if ( !vgic_reg64_check_access(dabt) ) goto bad_width;
        *r = vgic_reg64_extract(atomic_read(&vits->cmd_read) & 0xfffe0, info);
        return 1;
    case 0x0098 ... 0x009c:
    case 0x00a0 ... 0x00fc:
    case 0x0140 ... 0xbffc:
        /* Reserved -- read ignored */
        goto read_as_zero;
    case VREG64(GITS_BASER0):
        if ( !vgic_reg64_check_access(dabt) ) goto bad_width;
        vits_spin_lock(vits);
        if ( vits->ctlr & GITS_CTLR_ENABLE )
            *r = vgic_reg64_extract(vits->baser0, info);
        else
            *r = vgic_reg64_extract(vits->baser0_save, info);
        vits_spin_unlock(vits);
        return 1;
    case VRANGE64(GITS_BASER1, GITS_BASERN):
        goto read_as_zero_64;
    case VREG32(GITS_PIDR0):
        if ( dabt.size != DABT_WORD ) goto bad_width;
        *r = vgic_reg32_extract(GITS_PIDR0_VAL, info);
        return 1;
    case VREG32(GITS_PIDR1):
        if ( dabt.size != DABT_WORD ) goto bad_width;
        *r = vgic_reg32_extract(GITS_PIDR1_VAL, info);
        return 1;
    case VREG32(GITS_PIDR2):
        if ( dabt.size != DABT_WORD ) goto bad_width;
        *r = vgic_reg32_extract(GITS_PIDR2_VAL, info);
        return 1;
    case VREG32(GITS_PIDR3):
        if ( dabt.size != DABT_WORD ) goto bad_width;
        *r = vgic_reg32_extract(GITS_PIDR3_VAL, info);
        return 1;
    case VREG32(GITS_PIDR4):
        if ( dabt.size != DABT_WORD ) goto bad_width;
        *r = vgic_reg32_extract(GITS_PIDR4_VAL, info);
        return 1;
    case VRANGE32(GITS_PIDR5 ,GITS_PIDR7):
        goto read_as_zero_32;
   default:
        dprintk(XENLOG_G_ERR,
                "%pv: vITS: unhandled read r%d offset 0x%#08"PRIx32"\n",
                v, dabt.reg, gits_reg);
        return 0;
    }

bad_width:
    dprintk(XENLOG_G_ERR,
            "%pv: vITS: bad read width %d r%d offset 0x%#08"PRIx32"\n",
            v, dabt.size, dabt.reg, gits_reg);
    domain_crash_synchronous();
    return 0;

read_as_zero_64:
    if ( !vgic_reg64_check_access(dabt) ) goto bad_width;
    *r = 0;
    return 1;
read_as_zero_32:
    if ( dabt.size != DABT_WORD ) goto bad_width;
read_as_zero:
    *r = 0;
    return 1;
}

/*
 * GITS_BASER.Type[58:56], GITS_BASER.Entry_size[55:48]
 * and GITS_BASER.Shareability[11:10] are read-only,
 * Only flat table is supported. So GITS_BASER.Indirect[62]
 * is RAZ/WI.
 * Mask those fields while emulating GITS_BASER reg.
 * Shareability is set to 0x0 (Reserved) which is fixed.
 * TODO: Support shareability as fixed value is deprecated
 */
#define GITS_BASER_MASK  (~((0x7UL << GITS_BASER_TYPE_SHIFT)       | \
                         (0x1UL << GITS_BASER_INDIRECT_SHIFT)      | \
                         (0xffUL << GITS_BASER_ENTRY_SIZE_SHIFT)   | \
                         (0x3UL << GITS_BASER_SHAREABILITY_SHIFT)))

static int vgic_v3_gits_mmio_write(struct vcpu *v, mmio_info_t *info,
                                   register_t r, void *priv)
{
    struct vgic_its *vits = v->domain->arch.vgic.vits;
    struct hsr_dabt dabt = info->dabt;
    int ret;
    uint32_t gits_reg, sz, psz;
    uint64_t val;

    gits_reg = info->gpa - vits->gits_base;

    DPRINTK("%pv: vITS: GITS_MMIO_WRITE offset 0x%"PRIx32"\n", v, gits_reg);
    switch ( gits_reg )
    {
    case VREG32(GITS_CTLR):
    {
        uint32_t gits_ctlr;

        if ( dabt.size != DABT_WORD ) goto bad_width;
        vits_spin_lock(vits);
        /*
         * vITS is always quiescent, has no translations in progress and
         * completed all operations. So set quiescent by default.
         */
        gits_ctlr = vits->ctlr;
        vgic_reg32_update(&vits->ctlr,
                         (r & GITS_CTLR_ENABLE) | GITS_CTLR_QUIESCENT, info);
        /* If no change in GITS_CTRL.Enabled. Just return */ 
        if ( !((gits_ctlr ^ vits->ctlr) && (vits->ctlr & GITS_CTLR_ENABLE)) )
        {
            vits_spin_unlock(vits);
            return 1;
        }
        /*
         * On GITS_CTLR.Enabled = 1, update cmdbaser and baser0 
         * with cmd_base_save and baser0_save, if they are changed.
         */
        if ( vits->cmd_base_save ^ vits->cmd_base )
        {
            vits->cmd_base = vits->cmd_base_save;
            if ( vits->cmd_base & GITS_BASER_VALID )
            {
                val = ((vits->cmd_base & GITS_BASER_PAGES_MASK_VAL) + 1) *
                        SZ_4K;
                vits->cmd_qsize = val;
                atomic_set(&vits->cmd_read, 0);
            }
        }
        if ( vits->baser0_save ^ vits->baser0 )
        {
            vits->baser0 = vits->baser0_save;
            vits->dt_ipa = vits->baser0 & GITS_BASER_PA_MASK;
            psz = (vits->baser0 >> GITS_BASER_PAGE_SIZE_SHIFT) &
                   GITS_BASER_PAGE_SIZE_MASK_VAL;
            if ( psz == GITS_BASER_PAGE_SIZE_4K_VAL )
                sz = 4;
            else if ( psz == GITS_BASER_PAGE_SIZE_16K_VAL )
                sz = 16;
            else
                /* 0x11 and 0x10 are treated as 64K size */
                sz = 64;

            vits->dt_size = (vits->baser0 & GITS_BASER_PAGES_MASK_VAL) * sz *
                             SZ_1K;
        }
        vits_spin_unlock(vits);
        return 1;
    }
    case VREG32(GITS_IIDR):
        /* RO -- write ignored */
        goto write_ignore;
    case VREG64(GITS_TYPER):
        /* RO -- write ignored */
        goto write_ignore;
    case 0x0010 ... 0x007c:
    case 0xc000 ... 0xffcc:
        /* Implementation defined -- write ignored */
        goto write_ignore;
    case VREG64(GITS_CBASER):
        if ( !vgic_reg64_check_access(dabt) ) goto bad_width;
        vits_spin_lock(vits);
        /* RO -- write ignored, if GITS_CTLR.Enabled = 1 */
        if ( !(vits->ctlr & GITS_CTLR_ENABLE) )
            vgic_reg64_update(&vits->cmd_base_save, r, info);
        vits_spin_unlock(vits);
        return 1;
    case VREG64(GITS_CWRITER):
        if ( !vgic_reg64_check_access(dabt) ) goto bad_width;
        vits_spin_lock(vits);
        /* Only Bits[19:5] are writable */
        vgic_reg64_update(&vits->cmd_write, (r & 0xfffe0), info);
        ret = 1;
        /* CWRITER should be within command queue range */
        if ( (vits->ctlr & GITS_CTLR_ENABLE) &&
             (vits->cmd_write < vits->cmd_qsize) )
            ret = vits_process_cmd(v, vits);
        vits_spin_unlock(vits);
        return ret;
    case VREG64(GITS_CREADR):
        /* RO -- write ignored */
        goto write_ignore_64;
    case 0x0098 ... 0x009c:
    case 0x00a0 ... 0x00fc:
    case 0x0140 ... 0xbffc:
        /* Reserved -- write ignored */
        goto write_ignore;
    case VREG64(GITS_BASER0):
        if ( !vgic_reg64_check_access(dabt) ) goto bad_width;
        vits_spin_lock(vits);
        /* RO -- write ignored if GITS_CTLR.Enable = 1 */
        if ( !(vits->ctlr & GITS_CTLR_ENABLE) )
        {
            val = GITS_BASER0_INIT_VAL | (r & GITS_BASER_MASK);
            vgic_reg64_update(&vits->baser0_save, val, info);
        }
        vits_spin_unlock(vits);
        return 1;
    case VRANGE64(GITS_BASER1, GITS_BASERN):
        goto write_ignore_64;
    case VRANGE32(GITS_PIDR7, GITS_PIDR0):
        /* RO -- write ignored */
        goto write_ignore_32;
   default:
        dprintk(XENLOG_G_ERR,
                "%pv vITS: unhandled write r%d offset 0x%#08"PRIx32"\n",
                v, dabt.reg, gits_reg);
        return 0;
    }

bad_width:
    dprintk(XENLOG_G_ERR,
            "%pv: vITS: bad write width %d r%d offset 0x%#08"PRIx32"\n",
            v, dabt.size, dabt.reg, gits_reg);
    domain_crash_synchronous();
    return 0;

write_ignore_64:
    if ( !vgic_reg64_check_access(dabt) ) goto bad_width;
    return 1;
write_ignore_32:
    if ( dabt.size != DABT_WORD ) goto bad_width;
    return 1;
write_ignore:
    return 1;
}

static const struct mmio_handler_ops vgic_gits_mmio_handler = {
    .read  = vgic_v3_gits_mmio_read,
    .write = vgic_v3_gits_mmio_write,
};

int vits_domain_init(struct domain *d)
{
    struct vgic_its *vits;
    int i;

    if ( d->max_vcpus >= 256 )
    {
        printk(XENLOG_G_ERR
               "vITS: Cannot support guest with >= 256 vCPUs for domaind %d\n",
               d->domain_id);
        return -ENXIO;
    }

    ASSERT(is_hardware_domain(d));
    ASSERT(vits_hw.enabled);

    /*
     * HW might support more number of LPIs than specified here for a domain.
     * Here we limit number of LPIs supported for domain to nr_lpis.
     */
    d->arch.vgic.nr_lpis = gic_nr_irq_ids() - FIRST_GIC_LPI;

    d->arch.vgic.vits = xzalloc(struct vgic_its);
    if ( !d->arch.vgic.vits )
        return -ENOMEM;

    vits = d->arch.vgic.vits;

    spin_lock_init(&vits->lock);

    vits->collections = xzalloc_array(struct its_collection,
                                      vits_get_max_collections(d));
    if ( !vits->collections )
        return -ENOMEM;

    for ( i = 0; i < vits_get_max_collections(d); i++ )
        vits->collections[i].target_address = INVALID_PADDR;

    vits->baser0 = GITS_BASER0_INIT_VAL;
    vits->baser0_save = GITS_BASER0_INIT_VAL;

    /*
     * Only one virtual ITS is provided to domain.
     * Assign first physical ITS address to Dom0 virtual ITS.
     */
    vits->gits_base = vits_hw.phys_base;
    vits->gits_size = vits_hw.phys_size;

    register_mmio_handler(d, &vgic_gits_mmio_handler, vits->gits_base,
                          SZ_64K, NULL);

    return 0;
}

void vits_domain_free(struct domain *d)
{
   xfree(d->arch.vgic.vits->collections);
   xfree(d->arch.vgic.vits);
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
