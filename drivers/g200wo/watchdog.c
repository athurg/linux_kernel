/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Athurg.Feng
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : watchdog.c
 ::   :: ::       ::             ::     Generate   : 2010.06.24
 ::    ::::       ::       ::      ::   Update     : 2010.06.24
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
#include <mach/lpc32xx_gpio.h>	//GPIO Operate Define

#include "hardware.h"	//Hardware Regs Addr Define

#define WATCHDOG_VALID	0x5A

struct watchdog_st
{
	struct cdev cdev;
	struct semaphore sem;
};

struct watchdog_st *watchdog_stp;

static int watchdog_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	if (down_interruptible(&watchdog_stp->sem))
		return - ERESTARTSYS;
	
	__raw_writel(WATCHDOG_VALID, WATCHDOG_BASE);
	
	return ret;
}

//------------------------------------------------------------------------------
// register module
//------------------------------------------------------------------------------
static const struct file_operations watchdog_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = NULL,
	.write  = NULL,
	.ioctl  = watchdog_ioctl,
};

static int __init watchdog_init(void)
{
	int ret = 0, err = 0;
	dev_t devno;

	// register chrdev
	devno = MKDEV(MAJ_WATCHDOG, MIN_WATCHDOG);
	ret = register_chrdev_region(devno, 0, "g200wo_watchdog");
	if (ret<0){
		printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
		return ret;
	}

	// alloc dev
	watchdog_stp = kmalloc(sizeof(struct watchdog_st), GFP_KERNEL);
	if (!watchdog_stp){
		ret = - ENOMEM;
		goto fail_malloc;
	}

	memset(watchdog_stp, 0, sizeof(struct watchdog_st));
	init_MUTEX(&watchdog_stp->sem);
	
	// add cdev
	cdev_init(&watchdog_stp->cdev, &watchdog_fops);
	watchdog_stp->cdev.owner = THIS_MODULE;
	watchdog_stp->cdev.ops = &watchdog_fops;
	err = cdev_add(&watchdog_stp->cdev, devno, 1);
	if (err){
		printk("BSP: %s fail cdev_add\n", __FUNCTION__);
		goto fail_remap;
	}

	printk("G200WO WATCHDOG Driver instalwatchdog\n");
	return 0;

fail_remap:
	kfree(watchdog_stp);

fail_malloc:
	unregister_chrdev_region(devno, 1);
	printk("Fail to install G200WO WATCHDOG driver\n");
	return ret;
}

static void __exit watchdog_exit(void)
{
	dev_t devno;
	cdev_del(&watchdog_stp->cdev);
	kfree(watchdog_stp);
	devno = MKDEV(MAJ_WATCHDOG, MIN_WATCHDOG);
	unregister_chrdev_region(devno, 1);
	printk("G200WO WATCHDOG Driver removed\n");
}

module_init(watchdog_init);
module_exit(watchdog_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO WatchDog");
MODULE_LICENSE("GPL");
