/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : reset.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-07-01 11:16:14
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	None
*/
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/semaphore.h>

#include "g200wo_hw.h"
#include "reset.h"

struct reset_st
{
	struct cdev cdev;
	struct semaphore sem;
};

struct reset_st *reset_stp;

static int reset_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	if (down_interruptible(&reset_stp->sem))
		return - ERESTARTSYS;
	
	if(cmd != CMD_RESET){
		ret = -ENOTTY;
	}else{
		__raw_writeb((arg & 0xFF), RESET_BASE);
	}
	
	up(&reset_stp->sem);
	return ret;
}

static const struct file_operations reset_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = NULL,
	.write  = NULL,
	.ioctl  = reset_ioctl,
};

static int __init reset_init(void)
{
	int ret = 0, err = 0;
	dev_t devno;

	// register chrdev
	devno = MKDEV(MAJ_RESET, MIN_RESET);
	ret = register_chrdev_region(devno, 1, "g200wo_reset");
	if (ret<0){
		printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
		return ret;
	}

	// alloc dev
	reset_stp = kmalloc(sizeof(struct reset_st), GFP_KERNEL);
	if (!reset_stp){
		ret = - ENOMEM;
		goto fail_malloc;
	}
	memset(reset_stp, 0, sizeof(struct reset_st));

	init_MUTEX(&reset_stp->sem);
	cdev_init(&reset_stp->cdev, &reset_fops);
	reset_stp->cdev.owner = THIS_MODULE;
	reset_stp->cdev.ops = &reset_fops;

	// add cdev
	err = cdev_add(&reset_stp->cdev, devno, 1);
	if (err){
		printk("BSP: %s fail cdev_add\n", __FUNCTION__);
		goto fail_add;
	}

	printk("G200WO RESET Driver installed\n");
	return 0;

fail_add:
	kfree(reset_stp);

fail_malloc:
	unregister_chrdev_region(devno, 1);

	printk("Fail to install G200WO RESET driver\n");
	return ret;
}

static void __exit reset_exit(void)
{
	dev_t devno;

	cdev_del(&reset_stp->cdev);
	kfree(reset_stp);

	devno = MKDEV(MAJ_RESET, MIN_RESET);
	unregister_chrdev_region(devno, 1);

	printk("G200WO RESET Driver removed\n");
}

module_init(reset_init);
module_exit(reset_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO RESET");
MODULE_LICENSE("GPL");
