/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Athurg.Feng
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : lmk03000.c
 ::   :: ::       ::             ::     Generate   : 2009.05.31
 ::    ::::       ::       ::      ::   Update     : 2010.06.22
::::    :::     ::::::      ::::::::    Version    : v0.1

Description
	None
*/
//------------------------------------------------------------------------------
// include
//------------------------------------------------------------------------------
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/semaphore.h>	//semaphore Define
#include <mach/platform.h>	//GPIO Operate Define
#include <mach/lpc32xx_gpio.h>	//GPIO Operate Define

#include "hardware.h"	//Hardware Regs Addr Define
#include "lmk03000.h"

struct lmk03000_st
{
	struct cdev cdev;
	struct semaphore sem;
	volatile unsigned int __iomem *regp;
};

struct lmk03000_st *lmk03000_stp;

static int lmk03000_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	if (down_interruptible(&lmk03000_stp->sem))
		return - ERESTARTSYS;

	switch(cmd){
		case CMD_LMK03000_DATA :	//写数据，32位
			ret = __raw_writel(arg, &lmk03000_stp->regp[OFFSET_LMK03000_DATA]);
			break;
		case CMD_LMK03000_SYNC :	//写SYNC开关，argv=0/1
			ret = __raw_writeb(arg, &lmk03000_stp->regp[OFFSET_LMK03000_SYNC]);
			break;
		case CMD_LMK03000_LD :	//读LD状态，1锁定、0未锁定
			ret = __raw_readb(&lmk03000_stp->regp[OFFSET_LMK03000_LD]);
			break;
		case CMD_LMK03000_GOE :	//写GOE状态，1打开、0关闭
			ret = __raw_writeb(arg, &lmk03000_stp->regp[OFFSET_LMK03000_GOE]);
			break;
		default:
			ret = -ENOTTY;

	}
	
	return ret;
}

//------------------------------------------------------------------------------
// register module
//------------------------------------------------------------------------------
static const struct file_operations lmk03000_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = NULL,
	.write  = NULL,
	.ioctl  = lmk03000_ioctl,
};

static int __init lmk03000_init(void)
{
	int ret = 0, err = 0;
	dev_t devno;

	// register chrdev
	devno = MKDEV(MAJ_LED, MIN_LED);
	ret = register_chrdev_region(devno, 1, "lmk03000");
	if (ret<0){
		printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
		return ret;
	}

	// alloc dev
	lmk03000_stp = kmalloc(sizeof(struct lmk03000_st), GFP_KERNEL);
	if (!lmk03000_stp){
		ret = - ENOMEM;
		goto fail_malloc;
	}

	memset(lmk03000_stp, 0, sizeof(struct lmk03000_st));
	init_MUTEX(&lmk03000_stp->sem);
	
	// add cdev
	cdev_init(&lmk03000_stp->cdev, &lmk03000_fops);
	lmk03000_stp->cdev.owner = THIS_MODULE;
	lmk03000_stp->cdev.ops = &lmk03000_fops;
	err = cdev_add(&lmk03000_stp->cdev, devno, 1);
	if (err){
		printk("BSP: %s fail cdev_add\n", __FUNCTION__);
		goto fail_remap;
	}

	// ioremap
	lmk03000_stp->regp = ioremap(LMK03000_BASE, CPLD_RMSIZE);
	if (lmk03000_stp->regp==NULL)
	{
		printk("BSP: %s fail ioremap\n", __FUNCTION__);
		goto fail_remap;
	}

	printk("G200WO LMK03000 Driver installmk03000\n");
	return 0;

fail_remap:
	kfree(lmk03000_stp);

fail_malloc:
	unregister_chrdev_region(devno, 1);
	printk("Fail to install G200WO LMK03000 driver\n");
	return ret;
}

static void __exit lmk03000_exit(void)
{
	dev_t devno;
	iounmap(lmk03000_stp->regp);
	cdev_del(&lmk03000_stp->cdev);
	kfree(lmk03000_stp);
	devno = MKDEV(MAJ_LED, MIN_LED);
	unregister_chrdev_region(devno, 1);
	printk("G200WO LMK03000 Driver removed\n");
}

module_init(lmk03000_init);
module_exit(lmk03000_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO LMK03000");
MODULE_LICENSE("GPL");
