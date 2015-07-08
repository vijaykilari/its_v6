/*
 * xen/arch/arm/platforms/thunderx.c
 *
 * Cavium Thunder specific settings
 *
 * Vijaya Kumar K <Vijaya.Kumar@caviumnetworks.com>
 * Copyright (c) 2015 Cavium Inc.
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

#include <xen/config.h>
#include <asm/platform.h>
#include <asm/gic-its.h>

struct pci_dev_list 
{
   uint32_t seg;
   uint32_t bus;
   uint32_t dev;
   uint32_t func;
};

static struct pci_dev_list bdf[] =
{
    {0, 0, 1, 0},
    {0, 0, 2, 0},
    {0, 0, 6, 0},
    {0, 0, 7, 0},
    {0, 0, 10, 0},
    {0, 0, 11, 0},
    {0, 0, 14, 0},
    {0, 0, 15, 0},
    {0, 0, 16, 0},
    {0, 1, 0, 0},
    {0, 1, 0, 1},
    {0, 1, 0, 5},
    {0, 1, 1, 4},
    {0, 1, 9, 0},
    {0, 1, 9, 1},
    {0, 1, 9, 2},
    {0, 1, 9, 3},
    {0, 1, 9, 4},
    {0, 1, 9, 5},
    {0, 1, 10, 0},
    {0, 1, 10, 1},
    {0, 1, 10, 2},
    {0, 1, 10, 3},
    {0, 1, 14, 0},
    {0, 1, 14, 2},
    {0, 1, 14, 4},
    {0, 1, 16, 0},
    {0, 1, 16, 1},
    {0, 2, 0, 0},
    {0, 3, 0, 0},
    {0, 4, 0, 0},
    {1, 0, 1, 0},
    {1, 0, 4, 0},
    {1, 0, 5, 0},
    {1, 0, 6, 0},
    {1, 0, 7, 0},
    {1, 0, 8, 0},
    {1, 0, 9, 0},
    {1, 0, 10, 0},
    {1, 0, 11, 0},
    {2, 0, 1, 0},
    {2, 0, 2, 0},
    {2, 0, 3, 0},
    {2, 1, 0, 0},
    {2, 1, 0, 1},
    {2, 1, 0, 2},
    {2, 1, 0, 3},
    {2, 1, 0, 4},
    {2, 1, 0, 5},
    {2, 1, 0, 6},
    {2, 1, 0, 7},
    {2, 1, 1, 0},
    {2, 1, 1, 1},
    {2, 1, 1, 2},
    {2, 1, 1, 3},
    {2, 1, 1, 4},
    {2, 1, 1, 5},
    {2, 1, 1, 6},
    {2, 1, 1, 7},
    {2, 1, 2, 0},
    {2, 1, 2, 1},
    {2, 1, 2, 2},
    {2, 1, 2, 3},
    {2, 1, 2, 4},
    {2, 1, 2, 5},
    {2, 1, 2, 6},
    {2, 1, 2, 7},
    {2, 1, 3, 0},
    {2, 1, 3, 1},
    {2, 1, 3, 2},
    {2, 1, 3, 3},
    {2, 1, 3, 4},
    {2, 1, 3, 5},
    {2, 1, 3, 6},
    {2, 1, 3, 7},
    {2, 1, 4, 0},
    {2, 1, 4, 1},
    {2, 1, 4, 2},
    {2, 1, 4, 3},
    {2, 1, 4, 4},
    {2, 1, 4, 5},
    {2, 1, 4, 6},
    {2, 1, 4, 7},
    {2, 1, 5, 0},
    {2, 1, 5, 1},
    {2, 1, 5, 2},
    {2, 1, 5, 3},
    {2, 1, 5, 4},
    {2, 1, 5, 5},
    {2, 1, 5, 6},
    {2, 1, 5, 7},
    {2, 1, 6, 0},
    {2, 1, 6, 1},
    {2, 1, 6, 2},
    {2, 1, 6, 3},
    {2, 1, 6, 4},
    {2, 1, 6, 5},
    {2, 1, 6, 6},
    {2, 1, 6, 7},
    {2, 1, 7, 0},
    {2, 1, 7, 1},
    {2, 1, 7, 2},
    {2, 1, 7, 3},
    {2, 1, 7, 4},
    {3, 0, 1, 0},
};

#define BDF_TO_DEVID(seg, bus, dev, func) (seg << 16 | bus << 8 | dev << 3| func)

/* TODO: add and assign devices using PCI framework */
static int thunderx_specific_mapping(struct domain *d)
{
    struct dt_device_node *dt_its;
    uint32_t devid, i;
    int res;

    static const struct dt_device_match its_device_ids[] __initconst =
    {
        DT_MATCH_GIC_ITS,
        { /* sentinel */ },
    };

    for (dt_its = dt_find_matching_node(NULL, its_device_ids); dt_its;
           dt_its = dt_find_matching_node(dt_its, its_device_ids))
    {
        break;
    }

    if ( dt_its == NULL )
    {
        dprintk(XENLOG_ERR, "ThunderX: ITS node not found to add device\n");
        return 0;
    }

    for ( i = 0; i < ARRAY_SIZE(bdf); i++ )
    {
        devid = BDF_TO_DEVID(bdf[i].seg, bdf[i].bus,bdf[i].dev, bdf[i].func);
        res = its_add_device(devid, 32, dt_its);
        if ( res )
            return res;
        res = its_assign_device(d, devid, devid);
        if ( res )
            return res;
    }

    return 0;
}

static const char * const thunderx_dt_compat[] __initconst =
{
    "cavium,thunder-88xx",
    NULL
};

PLATFORM_START(thunderx, "THUNDERX")
    .compatible = thunderx_dt_compat,
    .specific_mapping = thunderx_specific_mapping,
PLATFORM_END

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
