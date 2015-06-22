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
#include <asm/device.h>
#include <asm/mmio.h>
#include <asm/io.h>
#include <asm/atomic.h>
#include <asm/gic_v3_defs.h>
#include <asm/gic.h>
#include <asm/vgic.h>
#include <asm/gic-its.h>
#include <asm/vits.h>
#include <xen/log2.h>

//#define DEBUG_ITS

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
