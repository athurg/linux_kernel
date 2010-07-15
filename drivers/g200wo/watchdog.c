/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Athurg.Feng
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : watchdog.c
 ::   :: ::       ::             ::     Generate   : 2010.06.24
 ::    ::::       ::       ::      ::   Update     : 2010-07-07 15:40:46
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	2010-07-07	Change cdev to miscdevices
	None

Changelog
	v0.2
		move watchdog_ioctl to watchdog_write
	v0.1
		initial
*/
#include <linux/fs.h>
#include <linux/kernel.h>


#include <asm/io.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>

#include <g200wo/g200wo_hw.h>

#define WATCHDOG_FEED_VALUE	0x5A

struct watchdog_st
{
	struct miscdevice dev;
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

	// malloc and initial memory
	watchdog_stp = kmalloc(sizeof(struct watchdog_st), GFP_KERNEL);
	if (!watchdog_stp){
		ret = - ENOMEM;
		goto fail_malloc;
	}

	memset(watchdog_stp, 0, sizeof(struct watchdog_st));
	init_MUTEX(&watchdog_stp->sem);
	watchdog_stp->dev.minor = MISC_DYNAMIC_MINOR;
	watchdog_stp->dev.name = "g200wo_watchdog";
	watchdog_stp->dev.fops = &watchdog_fops;

	// register device
	ret = misc_register(&watchdog_stp->dev);
	if (ret){
		printk("BSP: %s fail to register device\n", __FUNCTION__);
		goto fail_registe;
	}

	printk("G200WO WATCHDOG Driver installed\n");
	return 0;

fail_registe:
	kfree(watchdog_stp);

fail_malloc:

	printk("Fail to install G200WO WATCHDOG driver\n");
	return ret;
}

static void __exit watchdog_exit(void)
{
	misc_deregister(&watchdog_stp->dev);
	kfree(watchdog_stp);
	printk("G200WO WATCHDOG Driver removed\n");
}

module_init(watchdog_init);
module_exit(watchdog_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO WatchDog");
MODULE_LICENSE("GPL");
