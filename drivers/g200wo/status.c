/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : status.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010.06.24
::::    :::     ::::::      ::::::::    Version    : v0.2

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
#include <linux/semaphore.h>

#include "hardware.h"
#include "status.h"

#define DETECT_AGE_MASK		0x01	//Bit 0
#define DETECT_PA_MASK		0x02	//Bit 1

struct status_st
{
	struct cdev cdev;
	struct semaphore sem;
};

struct status_st *status_stp;

static int status_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	unsigned char temp;

	if (down_interruptible(&status_stp->sem))
		return - ERESTARTSYS;
	
	temp = __raw_readb(io_p2v(ADDR_DETECT));

	switch(cmd){
		case CMD_STATUS_AGE:
			temp &= DETECT_AGE_MASK;
			break;

		case CMD_STATUS_PA:
			temp &= DETECT_PA_MASK;
			break;

		default:
			ret = -ENOTTY;
			break;
	}

	up(&status_stp->sem);
	return (temp ? 1 : 0);
}

static ssize_t status_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	unsigned char sta;

	if (down_interruptible(&status_stp->sem))
		return - ERESTARTSYS;

	if (copy_from_user(&sta, buf, sizeof(sta)))
	{
		printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
		up(&status_stp->sem);
		return  - EFAULT;
	}

	//status_out(OFFSET_STATUS, sta);
	__raw_writeb(sta, io_p2v(ADDR_STATUS));
	
	up(&status_stp->sem);
	return sizeof(sta);
}

static ssize_t status_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	unsigned char sta;

	if (down_interruptible(&status_stp->sem))
		return - ERESTARTSYS;
	
	sta = __raw_readb(io_p2v(ADDR_STATUS));
	if (copy_to_user(buf, &sta, sizeof(sta))){
		printk("BSP: %s fail copy_to_user\n", __FUNCTION__);
		up(&status_stp->sem);
		return - EFAULT;
	}

	up(&status_stp->sem);

	return sizeof(sta);
}

//------------------------------------------------------------------------------
// register module
//------------------------------------------------------------------------------
static const struct file_operations status_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = status_read,
	.write  = status_write,
	.ioctl  = status_ioctl,
};

static int __init status_init(void)
{
	int ret = 0, err = 0;
	dev_t devno;

	// register chrdev
	devno = MKDEV(MAJ_STATUS, MIN_STATUS);
	ret = register_chrdev_region(devno, 1, "g200wo_status");
	if (ret<0){
		printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
		return ret;
	}

	// alloc dev
	status_stp = kmalloc(sizeof(struct status_st), GFP_KERNEL);
	if (!status_stp)
	{
		ret = - ENOMEM;
		goto fail_malloc;
	}
	memset(status_stp, 0, sizeof(struct status_st));

	init_MUTEX(&status_stp->sem);

	// add cdev
	cdev_init(&status_stp->cdev, &status_fops);
	status_stp->cdev.owner = THIS_MODULE;
	status_stp->cdev.ops = &status_fops;
	err = cdev_add(&status_stp->cdev, devno, 1);
	if (err){
		printk("BSP: %s fail cdev_add\n", __FUNCTION__);
		goto fail_add_cdev;
	}

	printk("NTS R804XY STATUS Driver installed\n");
	return 0;

fail_add_cdev:
	kfree(status_stp);

fail_malloc:
	unregister_chrdev_region(devno, 1);
	printk("Fail to install NTS R804XY STATUS driver\n");
	return ret;
}

static void __exit status_exit(void)
{
	dev_t devno;

	cdev_del(&status_stp->cdev);
	kfree(status_stp);
	devno = MKDEV(MAJ_STATUS, MIN_STATUS);
	unregister_chrdev_region(devno, 1);

	printk("NTS R804XY STATUS Driver removed\n");
}

module_init(status_init);
module_exit(status_exit);

MODULE_AUTHOR("Ray.Zhou, <ray.zhou@nts-intl.com>");
MODULE_DESCRIPTION("NTS G200WO STATUS DETECT");
MODULE_LICENSE("GPL");
