/* nts3250.c -- Nor Flash MTD map driver for NTS3250 Board */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

static struct mtd_partition partition_info[]={
    {
	    .name = "NTS3250 Stage0",
	    .offset = 0x0,
	    .size = 0xD000
    },
    {
	    .name = "Das u-boot",
	    .offset = MTDPART_OFS_APPEND,
	    .size = 0x1F3000
    },
    {
	    .name = "Kernel",
	    .offset = MTDPART_OFS_APPEND,
	    .size = 0x500000
    },
    {
	    .name = "rootfs",
	    .offset = MTDPART_OFS_APPEND,
	    .size = 0x800000
    },
    {
	    .name = "apps",
	    .offset = MTDPART_OFS_APPEND,
	    .size = MTDPART_SIZ_FULL
    },
};

static struct map_info nts3250_norflash_map = {
	.name = "NTS3250 Flash Bank",
	.size = 0x2000000,	//Size of Nor Flash
	.bankwidth = 4,	//Bit Width
	.phys = 0xE0000000,//Base Address of Nor Flash
};

static struct mtd_info *mymtd;

static int __init init_nts3250(void)
{
	int i;

	printk(KERN_NOTICE "NTS3250 on-board Flash device: 0x%Lx at 0x%Lx\n",
			(unsigned long long)nts3250_norflash_map.size,
			(unsigned long long)nts3250_norflash_map.phys);
	nts3250_norflash_map.virt = ioremap_nocache(nts3250_norflash_map.phys, nts3250_norflash_map.size);

	if (!nts3250_norflash_map.virt) {
		printk("Failed to ioremap_nocache\n");
		return -EIO;
	}

	simple_map_init(&nts3250_norflash_map);

	mymtd = do_map_probe("cfi_probe", &nts3250_norflash_map);
	if(!mymtd)
		mymtd = do_map_probe("map_ram", &nts3250_norflash_map);
	if(!mymtd)
		mymtd = do_map_probe("map_rom", &nts3250_norflash_map);

	if (!mymtd) {
		iounmap(nts3250_norflash_map.virt);
		printk(KERN_NOTICE "Fail to map probe\n");
		return -ENXIO;
	}

	mymtd->owner = THIS_MODULE;
	add_mtd_partitions( mymtd, partition_info, ARRAY_SIZE(partition_info) );

	return 0;
}

static void __exit cleanup_nts3250(void)
{
	if (mymtd) {
		del_mtd_partitions(mymtd);
		map_destroy(mymtd);
	}
	if (nts3250_norflash_map.virt) {
		iounmap(nts3250_norflash_map.virt);
		nts3250_norflash_map.virt = NULL;
	}
}

module_init(init_nts3250);
module_exit(cleanup_nts3250);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Athurg.Feng<athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("MTD NOR Flash map for NTS3250 DTRM Board");
