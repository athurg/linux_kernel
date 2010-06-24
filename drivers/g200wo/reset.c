/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : reset.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010.06.24
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
#include "reset.h"

struct reset_st
{
	struct cdev cdev;
	struct semaphore sem;
	volatile unsigned char __iomem *regp;
};

struct reset_st *reset_stp;

static int reset_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	if (down_interruptible(&reset_stp->sem))
		return - ERESTARTSYS;
	
	if(cmd != CMD_RESET_SET){
		return -ENOTTY;
	}

	__raw_writeb((arg & 0xFF), io_p2v(ADDR_RESET));
	
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

	// add cdev
	cdev_init(&reset_stp->cdev, &reset_fops);
	reset_stp->cdev.owner = THIS_MODULE;
	reset_stp->cdev.ops = &reset_fops;
	err = cdev_add(&reset_stp->cdev, devno, 1);
	if (err){
		printk("BSP: %s fail cdev_add\n", __FUNCTION__);
		goto fail_remap;
	}

	printk("NTS RESET Driver installed\n");
	return 0;

fail_remap:
	kfree(reset_stp);

fail_malloc:
	unregister_chrdev_region(devno, 1);
	printk("Fail to install NTS RESET driver\n");
	return ret;
}

static void __exit reset_exit(void)
{
	dev_t devno;

	cdev_del(&reset_stp->cdev);
	kfree(reset_stp);
	devno = MKDEV(MAJ_RESET, MIN_RESET);
	unregister_chrdev_region(devno, 1);
	printk("NTS RESET Driver removed\n");
}

module_init(reset_init);
module_exit(reset_exit);

MODULE_AUTHOR("Ray.Zhou, <ray.zhou@nts-intl.com>");
MODULE_DESCRIPTION("NTS RESET");
MODULE_LICENSE("GPL");
