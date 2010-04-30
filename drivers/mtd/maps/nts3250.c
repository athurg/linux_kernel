/* nts3250.c -- Nor Flash MTD map driver for NTS3250 Board */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#define NOR_FLASH_SEC_SIZE	0x20000

static struct mtd_partition partition_info[]={
    {
	    .name = "Stage0",
	    .offset = 0x0,
	    .size = (NOR_FLASH_SEC_SIZE * 1)
    },
    {
	    .name = "U-boot",
	    .offset = MTDPART_OFS_APPEND,
	    .size = (NOR_FLASH_SEC_SIZE * 2)
    },
    {
	    .name = "Param",
	    .offset = MTDPART_OFS_APPEND,
	    .size = (NOR_FLASH_SEC_SIZE * 1)
    },
    {
	    .name = "Kernel",
	    .offset = MTDPART_OFS_APPEND,
	    .size = (NOR_FLASH_SEC_SIZE * 16)
    },
    {
	    .name = "Apps",
	    .offset = MTDPART_OFS_APPEND,
	    .size = MTDPART_SIZ_FULL
    },
};

static struct map_info nts3250_norflash_map = {
	.name = "NTS3250 NorFlash Bank",
	.size = 0x2000000,	//Size of Nor Flash
	.bankwidth = 2,	//Bit Width
	.phys = 0xE0000000,//Base Address of Nor Flash
};

static struct mtd_info *mymtd;

static int __init init_nts3250(void)
{
	int i;

	printk(KERN_NOTICE "NorFlash : 0x%Lx Bytes,Base Addr 0x%Lx\n",
			(unsigned long long)nts3250_norflash_map.size,
			(unsigned long long)nts3250_norflash_map.phys);
	nts3250_norflash_map.virt = ioremap_nocache(nts3250_norflash_map.phys, nts3250_norflash_map.size);

	if (!nts3250_norflash_map.virt) {
		printk("Failed to ioremap_nocache\n");
		return -EIO;
	}

	simple_map_init(&nts3250_norflash_map);

	mymtd = do_map_probe("cfi_probe", &nts3250_norflash_map);

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
MODULE_DESCRIPTION("NOR Flash map for NTS3250 DTRM Board");
