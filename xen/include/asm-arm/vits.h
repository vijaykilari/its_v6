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

#ifndef __ASM_ARM_VITS_H__
#define __ASM_ARM_VITS_H__

/*
 * Per domain virtual ITS structure.
 */
struct vgic_its
{
   /* vITT device table ipa */
   paddr_t dt_ipa;
   /* vITT device table size */
   uint64_t dt_size;
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

int vgic_access_guest_memory(struct domain *d, paddr_t gipa, void *addr,
                             uint32_t size, bool_t set);
int vits_get_vitt_entry(struct domain *d, uint32_t devid, uint32_t event,
                        struct vitt *entry);
int vits_get_vdevice_entry(struct domain *d, uint32_t devid,
                           struct vdevice_table *entry);

#endif /* __ASM_ARM_VITS_H__ */
/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
