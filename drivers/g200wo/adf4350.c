/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Athurg.Feng
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : adf4350.c
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
#include "adf4350.h"

struct adf4350_st
{
	struct cdev cdev;
	struct semaphore sem;
	volatile unsigned int __iomem *regp;
};

struct adf4350_st *adf4350_stp;

static int adf4350_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	if (down_interruptible(&adf4350_stp->sem))
		return - ERESTARTSYS;

	switch(cmd){
		case CMD_ADF4350_TXA_DATA:
			ret = __raw_writel(arg, &adf4350_stp->regp[OFFSET_ADF4350_TXA_DATA]);
			break;
		case CMD_ADF4350_TXA_LD :
			ret = __raw_readb(&adf4350_stp->regp[OFFSET_ADF4350_TXA_LD]);
			break;
		case CMD_ADF4350_RXA_DATA:
			ret = __raw_writel(arg, &adf4350_stp->regp[OFFSET_ADF4350_RXA_DATA]);
			break;
		case CMD_ADF4350_RXA_LD:
			ret = __raw_readb(&adf4350_stp->regp[OFFSET_ADF4350_RXA_LD]);
			break;
		case CMD_ADF4350_B_DATA:
			ret = __raw_writel(arg, &adf4350_stp->regp[OFFSET_ADF4350_B_DATA]);
			break;
		case CMD_ADF4350_B_LD:	
			ret = __raw_readb(&adf4350_stp->regp[OFFSET_ADF4350_B_LD]);
			break;
		default:
			ret = -ENOTTY;

	}
	
	return ret;
}

//------------------------------------------------------------------------------
// register module
//------------------------------------------------------------------------------
static const struct file_operations adf4350_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = NULL,
	.write  = NULL,
	.ioctl  = adf4350_ioctl,
};

static int __init adf4350_init(void)
{
	int ret = 0, err = 0;
	dev_t devno;

	// register chrdev
	devno = MKDEV(MAJ_LED, MIN_LED);
	ret = register_chrdev_region(devno, 1, "adf4350");
	if (ret<0){
		printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
		return ret;
	}

	// alloc dev
	adf4350_stp = kmalloc(sizeof(struct adf4350_st), GFP_KERNEL);
	if (!adf4350_stp){
		ret = - ENOMEM;
		goto fail_malloc;
	}

	memset(adf4350_stp, 0, sizeof(struct adf4350_st));
	init_MUTEX(&adf4350_stp->sem);
	
	// add cdev
	cdev_init(&adf4350_stp->cdev, &adf4350_fops);
	adf4350_stp->cdev.owner = THIS_MODULE;
	adf4350_stp->cdev.ops = &adf4350_fops;
	err = cdev_add(&adf4350_stp->cdev, devno, 1);
	if (err){
		printk("BSP: %s fail cdev_add\n", __FUNCTION__);
		goto fail_remap;
	}

	// ioremap
	adf4350_stp->regp = ioremap(ADF4350_BASE, CPLD_RMSIZE);
	if (adf4350_stp->regp==NULL)
	{
		printk("BSP: %s fail ioremap\n", __FUNCTION__);
		goto fail_remap;
	}

	printk("G200WO ADF4350 Driver instaladf4350\n");
	return 0;

fail_remap:
	kfree(adf4350_stp);

fail_malloc:
	unregister_chrdev_region(devno, 1);
	printk("Fail to install G200WO ADF4350 driver\n");
	return ret;
}

static void __exit adf4350_exit(void)
{
	dev_t devno;
	iounmap(adf4350_stp->regp);
	cdev_del(&adf4350_stp->cdev);
	kfree(adf4350_stp);
	devno = MKDEV(MAJ_LED, MIN_LED);
	unregister_chrdev_region(devno, 1);
	printk("G200WO ADF4350 Driver removed\n");
}

module_init(adf4350_init);
module_exit(adf4350_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO ADF4350");
MODULE_LICENSE("GPL");
