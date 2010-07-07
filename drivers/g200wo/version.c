/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : version.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-07-07 15:36:53
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	2010-07-07	Change cdev to miscdevices
	None
*/

#include <linux/fs.h>
#include <linux/kernel.h>


#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>

#include "g200wo_hw.h"
#include "version.h"

#define KERNEL_VERSION      0x03

struct version_st
{
	struct miscdevice dev;
	struct semaphore sem;
};

struct version_st *version_stp;

static ssize_t version_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	struct version_elem elem;

	if (down_interruptible(&version_stp->sem))
		return - ERESTARTSYS;
	
	elem.hard = __raw_readb(HARD_VER_BASE);
	elem.cpld = __raw_readb(CPLD_VER_BASE);
	elem.uboot = __raw_readb(UBOOT_VER_BASE);
	elem.kernel = KERNEL_VERSION;

	if (copy_to_user(buf, &elem, sizeof(struct version_elem))) {
		printk("BSP: %s fail copy_to_user\n", __FUNCTION__);
		up(&version_stp->sem);
		return - EFAULT;
	}

	up(&version_stp->sem);
	return sizeof(struct version_elem);
}

static const struct file_operations version_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = version_read,
	.write  = NULL,
};

static int __init version_init(void)
{
	int ret = 0;

	// malloc and initial dev memory
	version_stp = kmalloc(sizeof(struct version_st), GFP_KERNEL);
	if (!version_stp) {
		ret = - ENOMEM;
		goto fail_malloc;
	}
	memset(version_stp, 0, sizeof(struct version_st));
	init_MUTEX(&version_stp->sem);

	version_stp->dev.minor = MISC_DYNAMIC_MINOR;
	version_stp->dev.name = "g200wo_version";
	version_stp->dev.fops = &version_fops;

	// register device
	ret = misc_register(&version_stp->dev);
	if (ret){
		printk("BSP: %s fail to register device\n", __FUNCTION__);
		goto fail_registe;
	}

fail_registe:
	kfree(version_stp);

fail_malloc:

	printk("Fail to install G200WO Version driver\n");
	return ret;
}

static void __exit version_exit(void)
{
	misc_deregister(&version_stp->dev);
	kfree(version_stp);
	printk("G200WO Version Driver removed\n");
}

module_init(version_init);
module_exit(version_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO VERSION");
MODULE_LICENSE("GPL");
