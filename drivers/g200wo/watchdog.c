/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Athurg.Feng
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : watchdog.c
 ::   :: ::       ::             ::     Generate   : 2010.06.24
 ::    ::::       ::       ::      ::   Update     : 2010-07-01 10:14:42
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	None

Changelog
	v0.2
		move watchdog_ioctl to watchdog_write
	v0.1
		initial
*/
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <asm/io.h>
#include <linux/semaphore.h>

#include "g200wo_hw.h"

#define WATCHDOG_FEED_VALUE	0x5A

struct watchdog_st
{
	struct cdev cdev;
	struct semaphore sem;
};

struct watchdog_st *watchdog_stp;

static ssize_t watchdog_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos);

static ssize_t watchdog_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	if (down_interruptible(&watchdog_stp->sem))
		return - ERESTARTSYS;
	
	__raw_writel(WATCHDOG_FEED_VALUE, WATCHDOG_BASE);
	
	up(&watchdog_stp->sem);

	return size;
}

static const struct file_operations watchdog_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = NULL,
	.write  = watchdog_write,
	.ioctl  = NULL,
};

static int __init watchdog_init(void)
{
	int ret = 0;
	dev_t devno;

	// register chrdev number
	devno = MKDEV(MAJ_WATCHDOG, MIN_WATCHDOG);
	ret = register_chrdev_region(devno, 1, "g200wo_watchdog");
	if (ret<0){
		printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
		return ret;
	}

	// alloc and initial memory
	watchdog_stp = kmalloc(sizeof(struct watchdog_st), GFP_KERNEL);
	if (!watchdog_stp){
		ret = - ENOMEM;
		goto fail_malloc;
	}

	memset(watchdog_stp, 0, sizeof(struct watchdog_st));
	init_MUTEX(&watchdog_stp->sem);

	cdev_init(&watchdog_stp->cdev, &watchdog_fops);
	watchdog_stp->cdev.owner = THIS_MODULE;
	watchdog_stp->cdev.ops = &watchdog_fops;
	
	// add cdev
	ret = cdev_add(&watchdog_stp->cdev, devno, 1);
	if (ret){
		printk("BSP: %s fail cdev_add\n", __FUNCTION__);
		goto fail_add;
	}

	printk("G200WO WATCHDOG Driver installed\n");
	return 0;

fail_add:
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
