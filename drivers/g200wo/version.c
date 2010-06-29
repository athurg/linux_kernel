/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : version.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010.06.09                                               
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	None
*/

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/semaphore.h>

#include "hardware.h"
#include "version.h"

struct version_st
{
	struct cdev cdev;
	struct semaphore sem;
};

struct version_st *version_stp;

static ssize_t version_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	struct version_elem elem;

	if (down_interruptible(&version_stp->sem))
		return - ERESTARTSYS;
	
	elem.hard   = __raw_readb(HARD_VER_BASE);
	elem.cpld   = __raw_readb(CPLD_VER_BASE);
	elem.uboot  = __raw_readb(UBOOT_VER_BASE);
	elem.kernel = KERNEL_VERSION;

	if (copy_to_user(buf, &elem, sizeof(struct version_elem))){
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
	dev_t devno;
	int ret = 0, err = 0;

	// register chrdev
	devno = MKDEV(MAJ_VERSION, MIN_VERSION);
	ret = register_chrdev_region(devno, 1, "g200wo_version");
	if (ret<0){
		printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
		return ret;
	}

	// alloc dev
	version_stp = kmalloc(sizeof(struct version_st), GFP_KERNEL);
	if (!version_stp){
		ret = - ENOMEM;
		goto fail_malloc;
	}
	memset(version_stp, 0, sizeof(struct version_st));

	init_MUTEX(&version_stp->sem);

	// add cdev
	cdev_init(&version_stp->cdev, &version_fops);
	version_stp->cdev.owner = THIS_MODULE;
	version_stp->cdev.ops = &version_fops;
	err = cdev_add(&version_stp->cdev, devno, 1);
	if (err){
		printk("BSP: %s fail cdev_add\n", __FUNCTION__);
		goto fail_add_dev;
	}

	printk("NTS Version Driver installed\n");
	return 0;

fail_add_dev:
	kfree(version_stp);

fail_malloc:
	unregister_chrdev_region(devno, 1);
	printk("Fail to install NTS Version driver\n");
	return ret;
}

static void __exit version_exit(void)
{
	dev_t devno;

	cdev_del(&version_stp->cdev);
	
	kfree(version_stp);
	
	devno = MKDEV(MAJ_VERSION, MIN_VERSION);	
	unregister_chrdev_region(devno, 1);

	printk("NTS Version Driver removed\n");
}

module_init(version_init);
module_exit(version_exit);

MODULE_AUTHOR("Ray.Zhou, <ray.zhou@nts-intl.com>");
MODULE_DESCRIPTION("NTS VERSION");
MODULE_LICENSE("GPL");
