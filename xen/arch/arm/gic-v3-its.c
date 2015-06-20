/*
 * Copyright (C) 2013, 2014 ARM Limited, All Rights Reserved.
 * Author: Marc Zyngier <marc.zyngier@arm.com>
 *
 * Xen changes:
 * Vijaya Kumar K <Vijaya.Kumar@caviumnetworks.com>
 * Copyright (C) 2014, 2015 Cavium Inc.
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

#include <xen/config.h>
#include <xen/bitops.h>
#include <xen/init.h>
#include <xen/mm.h>
#include <xen/irq.h>
#include <xen/sched.h>
#include <xen/errno.h>
#include <xen/delay.h>
#include <xen/list.h>
#include <xen/sizes.h>
#include <xen/vmap.h>
#include <asm/p2m.h>
#include <asm/domain.h>
#include <asm/io.h>
#include <asm/device.h>
#include <asm/gic.h>
#include <asm/gic_v3_defs.h>
#include <asm/gic-its.h>
#include <xen/log2.h>

#define its_print(lvl, fmt, ...)                                      \
    printk(lvl "GIC-ITS:" fmt, ## __VA_ARGS__)

#define its_err(fmt, ...) its_print(XENLOG_ERR, fmt, ## __VA_ARGS__)
/* TODO: ratelimit for Xen messages */
#define its_err_ratelimited(fmt, ...)                                 \
    its_print(XENLOG_G_ERR, fmt, ## __VA_ARGS__)

#define its_dbg(fmt, ...)                                             \
    its_print(XENLOG_DEBUG, fmt, ## __VA_ARGS__)

#define its_info(fmt, ...)                                            \
    its_print(XENLOG_INFO, fmt, ## __VA_ARGS__)

#define its_warn(fmt, ...)                                            \
    its_print(XENLOG_WARNING, fmt, ## __VA_ARGS__)

//#define DEBUG_GIC_ITS

#ifdef DEBUG_GIC_ITS
# define DPRINTK(fmt, args...) printk(XENLOG_DEBUG fmt, ##args)
#else
# define DPRINTK(fmt, args...) do {} while ( 0 )
#endif

#define ITS_FLAGS_CMDQ_NEEDS_FLUSHING         (1 << 0)
#define RDIST_FLAGS_PROPBASE_NEEDS_FLUSHING   (1 << 0)

/*
 * The ITS structure - contains most of the infrastructure, with the
 * msi_controller, the command queue, the collections, and the list of
 * devices writing to it.
 */
struct its_node {
    spinlock_t              lock;
    struct list_head        entry;
    void __iomem            *base;
    paddr_t                 phys_base;
    paddr_t                 phys_size;
    its_cmd_block           *cmd_base;
    its_cmd_block           *cmd_write;
    void                    *tables[GITS_BASER_NR_REGS];
    u32                     order[GITS_BASER_NR_REGS];
    struct its_collection   *collections;
    u64                     flags;
    u32                     ite_size;
    struct dt_device_node   *dt_node;
};

#define ITS_ITT_ALIGN    SZ_256

static LIST_HEAD(its_nodes);
static DEFINE_SPINLOCK(its_lock);
static struct rdist_prop  *gic_rdists;
static struct rb_root rb_its_dev;
static DEFINE_SPINLOCK(rb_its_dev_lock);

#define gic_data_rdist()    (this_cpu(rdist))

#ifdef DEBUG_GIC_ITS
static void dump_cmd(const its_cmd_block *cmd)
{
    printk(XENLOG_DEBUG
           "ITS: CMD[0] = 0x%lx CMD[1] = 0x%lx CMD[2] = 0x%lx CMD[3] = 0x%lx\n",
           cmd->bits[0], cmd->bits[1], cmd->bits[2], cmd->bits[3]);
}
#else
static void dump_cmd(const its_cmd_block *cmd) { }
#endif

static struct its_collection *dev_event_to_col(struct its_device *dev,
                                               u32 event)
{
    struct its_node *its = dev->its;

    return its->collections + dev->event_map.col_map[event];
}

/* RB-tree helpers for its_device */
static struct its_device *its_find_device(u32 devid)
{
    struct rb_node *node = rb_its_dev.rb_node;

    ASSERT(spin_is_locked(&rb_its_dev_lock));
    while ( node )
    {
        struct its_device *dev;

        dev = container_of(node, struct its_device, node);
        if ( devid < dev->device_id )
            node = node->rb_left;
        else if ( devid > dev->device_id )
            node = node->rb_right;
        else
            return dev;
    }

    return NULL;
}

static int its_insert_device(struct its_device *dev)
{
    struct rb_node **new, *parent;

    ASSERT(spin_is_locked(&rb_its_dev_lock));
    new = &rb_its_dev.rb_node;
    parent = NULL;
    while ( *new )
    {
        struct its_device *this;

        this  = container_of(*new, struct its_device, node);
        parent = *new;
        if ( dev->device_id < this->device_id )
            new = &((*new)->rb_left);
        else if ( dev->device_id > this->device_id )
            new = &((*new)->rb_right);
        else
            return -EEXIST;
    }

    rb_link_node(&dev->node, parent, new);
    rb_insert_color(&dev->node, &rb_its_dev);

    return 0;
}

#define ITS_CMD_QUEUE_SZ            SZ_64K
#define ITS_CMD_QUEUE_NR_ENTRIES    (ITS_CMD_QUEUE_SZ / sizeof(its_cmd_block))

static u64 its_cmd_ptr_to_offset(struct its_node *its, its_cmd_block *ptr)
{
    return (ptr - its->cmd_base) * sizeof(*ptr);
}

static int its_queue_full(struct its_node *its)
{
    int widx;
    int ridx;

    widx = its->cmd_write - its->cmd_base;
    ridx = readl_relaxed(its->base + GITS_CREADR) / sizeof(its_cmd_block);

    /* This is incredibly unlikely to happen, unless the ITS locks up. */
    if ( ((widx + 1) % ITS_CMD_QUEUE_NR_ENTRIES) == ridx )
        return 1;

    return 0;
}

static its_cmd_block *its_allocate_entry(struct its_node *its)
{
    its_cmd_block *cmd;
    u32 count = 1000000;    /* 1s! */

    while ( its_queue_full(its) )
    {
        count--;
        if ( !count )
        {
            its_err_ratelimited("ITS queue not draining\n");
            return NULL;
        }
        cpu_relax();
        udelay(1);
    }

    cmd = its->cmd_write++;

    /* Handle queue wrapping */
    if (its->cmd_write == (its->cmd_base + ITS_CMD_QUEUE_NR_ENTRIES))
        its->cmd_write = its->cmd_base;

    return cmd;
}

static its_cmd_block *its_post_commands(struct its_node *its)
{
    u64 wr = its_cmd_ptr_to_offset(its, its->cmd_write);

    writel_relaxed(wr, its->base + GITS_CWRITER);

    return its->cmd_write;
}

static void its_flush_cmd(struct its_node *its, its_cmd_block *cmd)
{
    /*
     * Make sure the commands written to memory are observable by
     * the ITS.
     */
    if ( its->flags & ITS_FLAGS_CMDQ_NEEDS_FLUSHING )
        clean_and_invalidate_dcache_va_range(cmd, sizeof(*cmd));
    else
        dsb(ishst);

    dump_cmd(cmd);
}

static void its_wait_for_range_completion(struct its_node *its,
                                          its_cmd_block *from,
                                          its_cmd_block *to)
{
    u64 rd_idx, from_idx, to_idx;
    u32 count = 1000000;    /* 1s! */

    from_idx = its_cmd_ptr_to_offset(its, from);
    to_idx = its_cmd_ptr_to_offset(its, to);

    while ( 1 )
    {
        rd_idx = readl_relaxed(its->base + GITS_CREADR);
        if ( rd_idx >= to_idx || rd_idx < from_idx )
            break;

        count--;
        if ( !count )
        {
            its_err_ratelimited("ITS queue timeout\n");
            return;
        }
        cpu_relax();
        udelay(1);
    }
}

static void its_send_single_command(struct its_node *its,
                                    its_cmd_block *src,
                                    struct its_collection *sync_col)
{
    its_cmd_block *cmd, *sync_cmd, *next_cmd;
    unsigned long flags;

    BUILD_BUG_ON(sizeof(its_cmd_block) != 32);

    spin_lock_irqsave(&its->lock, flags);

    cmd = its_allocate_entry(its);
    if ( !cmd )
    {   /* We're soooooo screewed... */
        its_err_ratelimited("ITS can't allocate, dropping command\n");
        spin_unlock_irqrestore(&its->lock, flags);
        return;
    }

    memcpy(cmd, src, sizeof(its_cmd_block));
    its_flush_cmd(its, cmd);

    if ( sync_col )
    {
        sync_cmd = its_allocate_entry(its);
        if ( !sync_cmd )
        {
            its_err_ratelimited("ITS can't SYNC, skipping\n");
            goto post;
        }
        sync_cmd->sync.cmd = GITS_CMD_SYNC;
        sync_cmd->sync.ta = sync_col->target_address;

        its_flush_cmd(its, sync_cmd);
    }

post:
    next_cmd = its_post_commands(its);
    spin_unlock_irqrestore(&its->lock, flags);

    its_wait_for_range_completion(its, cmd, next_cmd);
}

static void its_send_mapd(struct its_device *dev, int valid)
{
    its_cmd_block cmd;
    unsigned long itt_addr;
    u8 size;

    size = ilog2(dev->event_map.nr_lpis);
    itt_addr = __pa(dev->itt_addr);
    itt_addr = ROUNDUP(itt_addr, ITS_ITT_ALIGN);

    memset(&cmd, 0x0, sizeof(its_cmd_block));
    cmd.mapd.cmd = GITS_CMD_MAPD;
    cmd.mapd.devid = dev->device_id;
    cmd.mapd.size = size - 1;
    /*
     * ITT address field of MAPD command holds bit[48:8] of
     * itt address. Hence shift by 8.
     */
    cmd.mapd.itt = itt_addr >> 8;
    cmd.mapd.valid =  !!valid;

    its_send_single_command(dev->its, &cmd, NULL);
}

static void its_send_mapc(struct its_node *its, struct its_collection *col,
                          int valid)
{
    its_cmd_block cmd;

    memset(&cmd, 0x0, sizeof(its_cmd_block));
    cmd.mapc.cmd = GITS_CMD_MAPC;
    cmd.mapc.col = col->col_id;
    cmd.mapc.ta = col->target_address;
    cmd.mapc.valid = !!valid;

    its_send_single_command(its, &cmd, col);
}

static void its_send_mapvi(struct its_device *dev, u32 phys_id, u32 event)
{
    its_cmd_block cmd;
    struct its_collection *col = dev_event_to_col(dev, event);

    memset(&cmd, 0x0, sizeof(its_cmd_block));
    cmd.mapvi.cmd = GITS_CMD_MAPVI;
    cmd.mapvi.devid = dev->device_id;
    cmd.mapvi.event = event;
    cmd.mapvi.phy_id = phys_id;
    cmd.mapvi.col = col->col_id;

    its_send_single_command(dev->its, &cmd, col);
}

static void its_send_invall(struct its_node *its, struct its_collection *col)
{
    its_cmd_block cmd;

    memset(&cmd, 0x0, sizeof(its_cmd_block));
    cmd.invall.cmd = GITS_CMD_INVALL;
    cmd.invall.col = col->col_id;

    its_send_single_command(its, &cmd, NULL);
}

static void its_send_discard(struct its_device *dev, u32 event)
{
    its_cmd_block cmd;
    struct its_collection *col = dev_event_to_col(dev, event);

    memset(&cmd, 0x0, sizeof(its_cmd_block));
    cmd.discard.devid = dev->device_id;
    cmd.discard.event = event;

    its_send_single_command(dev->its, &cmd, col);
}

/*
 * How we allocate LPIs:
 *
 * The GIC has id_bits bits for interrupt identifiers. From there, we
 * must subtract 8192 which are reserved for SGIs/PPIs/SPIs. Then, as
 * we allocate LPIs by chunks of 32, we can shift the whole thing by 5
 * bits to the right.
 *
 * This gives us (((1UL << id_bits) - 8192) >> 5) possible allocations.
 */
#define IRQS_PER_CHUNK_SHIFT    5
#define IRQS_PER_CHUNK         (1 << IRQS_PER_CHUNK_SHIFT)

static unsigned long *lpi_bitmap;
static u32 lpi_chunks;
static DEFINE_SPINLOCK(lpi_lock);

static int its_lpi_to_chunk(int lpi)
{
    return (lpi - 8192) >> IRQS_PER_CHUNK_SHIFT;
}

static int its_chunk_to_lpi(int chunk)
{
    return (chunk << IRQS_PER_CHUNK_SHIFT) + 8192;
}

static int its_lpi_init(u32 id_bits)
{
    lpi_chunks = its_lpi_to_chunk(1UL << id_bits);

    lpi_bitmap = xzalloc_bytes(BITS_TO_LONGS(lpi_chunks) * sizeof(long));
    if ( !lpi_bitmap )
    {
        lpi_chunks = 0;
        return -ENOMEM;
    }

    its_info("ITS: Allocated %d chunks for LPIs\n", (int)lpi_chunks);

    return 0;
}

static unsigned long *its_lpi_alloc_chunks(int nirqs, int *base)
{
    unsigned long *bitmap = NULL;
    int chunk_id, nr_chunks, nr_ids, i;

    nr_chunks = DIV_ROUND_UP(nirqs, IRQS_PER_CHUNK);

    spin_lock(&lpi_lock);

    do {
        chunk_id = bitmap_find_next_zero_area(lpi_bitmap, lpi_chunks,
                                              0, nr_chunks, 0);
        if ( chunk_id < lpi_chunks )
            break;

        nr_chunks--;
    } while ( nr_chunks > 0 );

    if ( !nr_chunks )
        goto out;

    bitmap = xzalloc_bytes(BITS_TO_LONGS(nr_chunks * IRQS_PER_CHUNK) *
                           sizeof (long));
    if ( !bitmap )
        goto out;

    for ( i = 0; i < nr_chunks; i++ )
        set_bit(chunk_id + i, lpi_bitmap);

    *base = its_chunk_to_lpi(chunk_id);
    nr_ids = nr_chunks * IRQS_PER_CHUNK;

    if ( nr_ids < nirqs )
    {
        xfree(bitmap);
        bitmap = NULL;
    }

out:
    spin_unlock(&lpi_lock);

    return bitmap;
}

static void its_lpi_free(struct its_device *dev)
{
    int lpi;

    spin_lock(&lpi_lock);

    for ( lpi = dev->event_map.lpi_base;
        lpi < (dev->event_map.lpi_base + dev->event_map.nr_lpis);
        lpi += IRQS_PER_CHUNK )
    {
        int chunk = its_lpi_to_chunk(lpi);

        if (chunk > lpi_chunks)
            its_err("Bad LPI chunk %d\n", chunk);
        if ( test_bit(chunk, lpi_bitmap) )
            clear_bit(chunk, lpi_bitmap);
    }

    spin_unlock(&lpi_lock);

    xfree(dev->event_map.lpi_map);
}

/*
 * We allocate 64kB for PROPBASE. That gives us at most 64K LPIs to
 * deal with (one configuration byte per interrupt). PENDBASE has to
 * be 64kB aligned (one bit per LPI, plus 8192 bits for SPI/PPI/SGI).
 */
#define LPI_PROPBASE_SZ    SZ_64K
#define LPI_PENDBASE_SZ    (LPI_PROPBASE_SZ / 8 + SZ_1K)

/*
 * This is how many bits of ID we need, including the useless ones.
 */
#define LPI_NRBITS    ilog2(LPI_PROPBASE_SZ + SZ_8K)

static int __init its_alloc_lpi_tables(void)
{
    paddr_t paddr;

    gic_rdists->prop_page =
           alloc_xenheap_pages(get_order_from_bytes(LPI_PROPBASE_SZ), 0);

    if ( !gic_rdists->prop_page )
    {
        its_err("Failed to allocate PROPBASE\n");
        return -ENOMEM;
    }

    paddr = __pa(gic_rdists->prop_page);
    its_info("GIC: using LPI property table @%pa\n", &paddr);

    /* Set LPI priority and Group-1, but disabled */
    memset(gic_rdists->prop_page,
           GIC_PRI_IRQ | LPI_PROP_GROUP1,
           LPI_PROPBASE_SZ);

    /* Make sure the GIC will observe the written configuration */
    clean_and_invalidate_dcache_va_range(gic_rdists->prop_page,
                                         LPI_PROPBASE_SZ);

    return 0;
}

static const char *its_base_type_string[] = {
    [GITS_BASER_TYPE_DEVICE]       = "Devices",
    [GITS_BASER_TYPE_VCPU]         = "Virtual CPUs",
    [GITS_BASER_TYPE_CPU]          = "Physical CPUs",
    [GITS_BASER_TYPE_COLLECTION]   = "Interrupt Collections",
    [GITS_BASER_TYPE_RESERVED5]    = "Reserved (5)",
    [GITS_BASER_TYPE_RESERVED6]    = "Reserved (6)",
    [GITS_BASER_TYPE_RESERVED7]    = "Reserved (7)",
};

static void its_free_tables(struct its_node *its)
{
    int i;

    for ( i = 0; i < GITS_BASER_NR_REGS; i++ )
    {
        if ( its->tables[i] )
        {
            free_xenheap_pages(its->tables[i], its->order[i]);
            its->tables[i] = NULL;
            its->order[i] = 0;
        }
    }
}

static int its_alloc_tables(struct its_node *its)
{
    int err;
    int i;
    int psz = SZ_64K;
    u64 shr = GITS_BASER_InnerShareable;
    u64 cache = GITS_BASER_WaWb;

    for ( i = 0; i < GITS_BASER_NR_REGS; i++ )
    {
        u64 val = readq_relaxed(its->base + GITS_BASER + i * 8);
        u64 type = GITS_BASER_TYPE(val);
        u64 entry_size = GITS_BASER_ENTRY_SIZE(val);
        unsigned int order = get_order_from_bytes(psz);
        int alloc_size;
        u64 tmp;
        void *base;

        if ( type == GITS_BASER_TYPE_NONE )
            continue;

        /*
         * Allocate as many entries as required to fit the
         * range of device IDs that the ITS can grok... The ID
         * space being incredibly sparse, this results in a
         * massive waste of memory.
         *
         * For other tables, only allocate a single page.
         */
        if ( type == GITS_BASER_TYPE_DEVICE )
        {
            u64 typer = readq_relaxed(its->base + GITS_TYPER);
            u32 ids = GITS_TYPER_DEVBITS(typer);

            order = max(get_order_from_bytes((1UL << ids) * entry_size), order);
            if (order >= MAX_ORDER)
            {
                order = MAX_ORDER - 1;
                its_warn("Device Table too large,reduce its page order to %u\n",
                         order);
            }
        }

        alloc_size = (1 << order) * PAGE_SIZE;
        base = alloc_xenheap_pages(order, 0);
        if ( !base )
        {
            err = -ENOMEM;
            goto out_free;
        }
        memset(base, 0, alloc_size);
        its->tables[i] = base;
        its->order[i] = order;

retry_baser:
        val = (__pa(base)                                        |
               (type << GITS_BASER_TYPE_SHIFT)                   |
               ((entry_size - 1) << GITS_BASER_ENTRY_SIZE_SHIFT) |
               cache                                             |
               shr                                               |
               GITS_BASER_VALID);

        switch (psz) {
        case SZ_4K:
            val |= GITS_BASER_PAGE_SIZE_4K;
            break;
        case SZ_16K:
            val |= GITS_BASER_PAGE_SIZE_16K;
            break;
        case SZ_64K:
            val |= GITS_BASER_PAGE_SIZE_64K;
            break;
        }

        val |= (alloc_size / psz) - 1;

        writeq_relaxed(val, its->base + GITS_BASER + i * 8);
        tmp = readq_relaxed(its->base + GITS_BASER + i * 8);

        if ( (val ^ tmp) & GITS_BASER_SHAREABILITY_MASK )
        {
            /*
             * Shareability didn't stick. Just use
             * whatever the read reported, which is likely
             * to be the only thing this redistributor
             * supports.
             */
            shr = tmp & GITS_BASER_SHAREABILITY_MASK;
            if ( !shr )
                cache = GITS_BASER_nC;
            goto retry_baser;
        }

        if ( (val ^ tmp) & GITS_BASER_PAGE_SIZE_MASK )
        {
            /*
             * Page size didn't stick. Let's try a smaller
             * size and retry. If we reach 4K, then
             * something is horribly wrong...
             */
            switch (psz) {
            case SZ_16K:
                psz = SZ_4K;
                goto retry_baser;
            case SZ_64K:
                psz = SZ_16K;
                goto retry_baser;
            }
        }

        if ( val != tmp )
        {
            its_err("ITS: GITS_BASER%d doesn't stick: %lx %lx\n",
                    i, (unsigned long) val, (unsigned long) tmp);
            err = -ENXIO;
            goto out_free;
        }

        its_info("ITS: allocated %d %s @%lx (psz %dK, shr %d)\n",
                 (int)(alloc_size / entry_size),
                 its_base_type_string[type],
                 (unsigned long)__pa(base),
                 psz / SZ_1K, (int)shr >> GITS_BASER_SHAREABILITY_SHIFT);
    }

    return 0;

out_free:
    its_free_tables(its);

    return err;
}

static int its_alloc_collections(struct its_node *its)
{
    its->collections = xzalloc_array(struct its_collection, nr_cpu_ids);
    if ( !its->collections )
        return -ENOMEM;

    return 0;
}

static void its_cpu_init_lpis(void)
{
    void __iomem *rbase = gic_data_rdist().rbase;
    void *pend_page;
    u64 val, tmp;

    /* If we didn't allocate the pending table yet, do it now */
    pend_page = gic_data_rdist().pend_page;
    if ( !pend_page )
    {
        paddr_t paddr;
        u32 order;

        /*
         * The pending pages have to be at least 64kB aligned,
         * hence the 'max(LPI_PENDBASE_SZ, SZ_64K)' below.
         */
        order = get_order_from_bytes(max(LPI_PENDBASE_SZ, SZ_64K));
        pend_page = alloc_xenheap_pages(order, 0);
        if ( !pend_page )
        {
            its_err("Failed to allocate PENDBASE for CPU%d with order %d\n",
                    smp_processor_id(), order);
            return;
        }

        memset(pend_page, 0, max(LPI_PENDBASE_SZ, SZ_64K));
        /* Make sure the GIC will observe the zero-ed page */
        clean_and_invalidate_dcache_va_range(pend_page, LPI_PENDBASE_SZ);

        paddr = __pa(pend_page);

        its_info("CPU%d: using LPI pending table @%pa\n",
                 smp_processor_id(), &paddr);

        gic_data_rdist().pend_page = pend_page;
    }

    /* Disable LPIs */
    val = readl_relaxed(rbase + GICR_CTLR);
    val &= ~GICR_CTLR_ENABLE_LPIS;
    writel_relaxed(val, rbase + GICR_CTLR);

    /*
     * Make sure any change to the table is observable by the GIC.
     */
    dsb(sy);

    /* set PROPBASE */
    val = (__pa(gic_rdists->prop_page)   |
           GICR_PROPBASER_InnerShareable |
           GICR_PROPBASER_WaWb           |
           ((LPI_NRBITS - 1) & GICR_PROPBASER_IDBITS_MASK));

    writeq_relaxed(val, rbase + GICR_PROPBASER);
    tmp = readq_relaxed(rbase + GICR_PROPBASER);

    if ( (tmp ^ val) & GICR_PROPBASER_SHAREABILITY_MASK )
    {
        if ( !(tmp & GICR_PROPBASER_SHAREABILITY_MASK) )
        {
            /*
             * The HW reports non-shareable, we must
             * remove the cacheability attributes as well.
             */
            val &= ~(GICR_PROPBASER_SHAREABILITY_MASK |
                     GICR_PROPBASER_CACHEABILITY_MASK);
            val |= GICR_PROPBASER_nC;
            writeq_relaxed(val, rbase + GICR_PROPBASER);
        }

        its_info("GIC: using cache flushing for LPI property table\n");
        gic_rdists->flags |= RDIST_FLAGS_PROPBASE_NEEDS_FLUSHING;
    }

    /* set PENDBASE */
    val = (__pa(pend_page)               |
           GICR_PROPBASER_InnerShareable |
           GICR_PROPBASER_WaWb);

    writeq_relaxed(val, rbase + GICR_PENDBASER);
    tmp = readq_relaxed(rbase + GICR_PENDBASER);

    if ( !(tmp & GICR_PENDBASER_SHAREABILITY_MASK) )
    {
        /*
         * The HW reports non-shareable, we must remove the
         * cacheability attributes as well.
         */
        val &= ~(GICR_PENDBASER_SHAREABILITY_MASK |
                 GICR_PENDBASER_CACHEABILITY_MASK);
        val |= GICR_PENDBASER_nC;
        writeq_relaxed(val, rbase + GICR_PENDBASER);
    }

    /* Enable LPIs */
    val = readl_relaxed(rbase + GICR_CTLR);
    val |= GICR_CTLR_ENABLE_LPIS;
    writel_relaxed(val, rbase + GICR_CTLR);

    /* Make sure the GIC has seen the above */
    dsb(sy);
}

static void its_cpu_init_collection(void)
{
    struct its_node *its;
    int cpu;

    spin_lock(&its_lock);
    cpu = smp_processor_id();

    list_for_each_entry(its, &its_nodes, entry)
    {
        u64 target;

        /*
         * We now have to bind each collection to its target
         * redistributor.
         */
        if ( readq_relaxed(its->base + GITS_TYPER) & GITS_TYPER_PTA )
        {
            target = gic_data_rdist().phys_base;
            /* ITS command considers only [48:16] of the GICR address */
            target >>= 16;
        }
        else
        {
            /*
             * This ITS wants a linear CPU number.
             */
            target = readq_relaxed(gic_data_rdist().rbase + GICR_TYPER);
            target = GICR_TYPER_CPU_NUMBER(target);
        }

        /* Perform collection mapping */
        its->collections[cpu].col_id = cpu;
        its->collections[cpu].target_address = target;

        its_send_mapc(its, &its->collections[cpu], 1);
        its_send_invall(its, &its->collections[cpu]);
    }

    spin_unlock(&its_lock);
}

static int its_force_quiescent(void __iomem *base)
{
    u32 count = 1000000;   /* 1s */
    u32 val;

    val = readl_relaxed(base + GITS_CTLR);
    if ( val & GITS_CTLR_QUIESCENT )
        return 0;

    /* Disable the generation of all interrupts to this ITS */
    val &= ~GITS_CTLR_ENABLE;
    writel_relaxed(val, base + GITS_CTLR);

    /* Poll GITS_CTLR and wait until ITS becomes quiescent */
    while ( 1 )
    {
        val = readl_relaxed(base + GITS_CTLR);
        if ( val & GITS_CTLR_QUIESCENT )
            return 0;

        count--;
        if ( !count )
            return -EBUSY;

        cpu_relax();
        udelay(1);
    }
}

static int its_probe(struct dt_device_node *node)
{
    paddr_t its_addr, its_size;
    struct its_node *its;
    void __iomem *its_base;
    u32 val, typer;
    u64 baser, tmp;
    int err;

    if ( !dt_get_property(node, "msi-controller", NULL) )
    {
        its_warn("%s: not a msi-controller\n", node->full_name);
        return -ENXIO;
    }

    err = dt_device_get_address(node, 0, &its_addr, &its_size);
    if ( err )
    {
        its_warn("%s: no regs?\n", node->full_name);
        return -ENXIO;
    }

    its_base = ioremap_nocache(its_addr, its_size);
    if ( !its_base )
    {
        its_warn("%s: unable to map registers\n", node->full_name);
        return -ENOMEM;
    }

    val = readl_relaxed(its_base + GITS_PIDR2) & GIC_PIDR2_ARCH_REV_MASK;
    if ( val != 0x30 && val != 0x40 )
    {
        its_warn("%s: no ITS detected, giving up\n", node->full_name);
        err = -ENODEV;
        goto out_unmap;
    }

    err = its_force_quiescent(its_base);
    if ( err )
    {
        its_warn("%s: failed to quiesce, giving up\n",
                 node->full_name);
        goto out_unmap;
    }

    its_info("ITS: %s\n", node->full_name);

    its = xzalloc(struct its_node);
    if ( !its )
    {
        err = -ENOMEM;
        goto out_unmap;
    }

    spin_lock_init(&its->lock);
    INIT_LIST_HEAD(&its->entry);
    its->dt_node = node;
    its->base = its_base;
    its->phys_base = its_addr;
    its->phys_size = its_size;
    typer = readl_relaxed(its_base + GITS_TYPER);
    its->ite_size = ((typer >> 4) & 0xf) + 1;

    its->cmd_base = xzalloc_bytes(ITS_CMD_QUEUE_SZ);
    if ( !its->cmd_base )
    {
        err = -ENOMEM;
        goto out_free_its;
    }
    its->cmd_write = its->cmd_base;

    err = its_alloc_tables(its);
    if ( err )
        goto out_free_cmd;

    err = its_alloc_collections(its);
    if ( err )
        goto out_free_tables;

    baser = (__pa(its->cmd_base)            |
             GITS_CBASER_WaWb               |
             GITS_CBASER_InnerShareable     |
             (ITS_CMD_QUEUE_SZ / SZ_4K - 1) |
             GITS_CBASER_VALID);

    writeq_relaxed(baser, its->base + GITS_CBASER);
    tmp = readq_relaxed(its->base + GITS_CBASER);
    if ( (tmp ^ baser) & GITS_CBASER_SHAREABILITY_MASK )
    {
        if (!(tmp & GITS_CBASER_SHAREABILITY_MASK))
        {
            /*
             * The HW reports non-shareable, we must
             * remove the cacheability attributes as
             * well.
             */
            baser &= ~(GITS_CBASER_SHAREABILITY_MASK |
                       GITS_CBASER_CACHEABILITY_MASK);
            baser |= GITS_CBASER_nC;
            writeq_relaxed(baser, its->base + GITS_CBASER);
        }

        its_info("ITS: using cache flushing for cmd queue\n");
        its->flags |= ITS_FLAGS_CMDQ_NEEDS_FLUSHING;
    }

    writeq_relaxed(0, its->base + GITS_CWRITER);
    writel_relaxed(GITS_CTLR_ENABLE, its->base + GITS_CTLR);

    spin_lock(&its_lock);
    list_add(&its->entry, &its_nodes);
    spin_unlock(&its_lock);

    rb_its_dev = RB_ROOT;

    return 0;

out_free_tables:
    its_free_tables(its);
out_free_cmd:
    xfree(its->cmd_base);
out_free_its:
    xfree(its);
out_unmap:
    iounmap(its_base);
    its_err("ITS: failed probing %s (%d)\n", node->full_name, err);
    return err;
}

static bool gic_rdists_supports_plpis(void)
{
    return !!(readl_relaxed(gic_data_rdist().rbase + GICR_TYPER) & GICR_TYPER_PLPIS);
}

int its_cpu_init(void)
{
    if ( !list_empty(&its_nodes) )
    {
        if ( !gic_rdists_supports_plpis() )
        {
            its_info("CPU%d: LPIs not supported\n", smp_processor_id());
            return -ENXIO;
        }
        its_cpu_init_lpis();
        its_cpu_init_collection();
    }

    return 0;
}

int __init its_init(struct rdist_prop *rdists)
{
    struct dt_device_node *np = NULL;

    static const struct dt_device_match its_device_ids[] __initconst =
    {
        DT_MATCH_GIC_ITS,
        { /* sentinel */ },
    };

    for (np = dt_find_matching_node(NULL, its_device_ids); np;
        np = dt_find_matching_node(np, its_device_ids))
        its_probe(np);

    if ( list_empty(&its_nodes) )
    {
        its_warn("ITS: No ITS available, not enabling LPIs\n");
        return -ENXIO;
    }

    gic_rdists = rdists;
    its_alloc_lpi_tables();
    its_lpi_init(rdists->id_bits);

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
