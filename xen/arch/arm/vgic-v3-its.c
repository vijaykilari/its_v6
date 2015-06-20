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
#include <asm/gic_v3_defs.h>
#include <asm/gic.h>
#include <asm/vgic.h>
#include <asm/gic-its.h>
#include <xen/log2.h>

int vits_access_guest_table(struct domain *d, paddr_t entry, void *addr,
                            uint32_t size, bool_t set)
{
    struct page_info *page;
    uint64_t offset;
    p2m_type_t p2mt;
    void *p;

    page = get_page_from_gfn(d, paddr_to_pfn(entry), &p2mt, P2M_ALLOC);
    if ( !page )
    {
        printk(XENLOG_G_ERR "d%"PRIu16": vITS: Failed to get table entry\n",
               d->domain_id);
        return -EINVAL;
    }

    if ( !p2m_is_ram(p2mt) )
    {
        put_page(page);
        printk(XENLOG_G_ERR "d%"PRIu16": vITS: with wrong attributes\n",
               d->domain_id);
        return -EINVAL;
    }

    p = __map_domain_page(page);
    /* Offset within the mapped page */
    offset = entry & ~PAGE_MASK;

    if ( set )
        memcpy(p + offset, addr, size);
    else
        memcpy(addr, p + offset, size);

    unmap_domain_page(p);
    put_page(page);

    return 0;
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
               "d%"PRIu16":vITS:Out of range off 0x%"PRIx64" id 0x%"PRIx32"\n",
               d->domain_id, offset, dev_id);
        return -EINVAL;
    }

    dt_entry = vits->dt_ipa + offset;

    return vits_access_guest_table(d, dt_entry, entry,
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
               "d%"PRIu16": vITS: Fail to get vdevice for vdevid 0x%"PRIx32"\n",
               d->domain_id, devid);
        return -EINVAL;
    }

    /* dt_entry has been validated in vits_get_vdevice_entry */
    offset = event * sizeof(struct vitt);
    if ( offset > dt_entry.vitt_size )
    {
        printk(XENLOG_G_ERR "d%"PRIu16": vITS: ITT out of range\n",
               d->domain_id);
        return -EINVAL;
    }

    vitt_entry = dt_entry.vitt_ipa + offset;

    return vits_access_guest_table(d, vitt_entry, entry,
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

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
