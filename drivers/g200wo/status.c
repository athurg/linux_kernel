/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : status.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-07-07 15:38:38
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
#include "status.h"

#define DETECT_AGE_MASK		_BIT(1)
#define DETECT_PA_MASK		_BIT(2)

struct status_st
{
	struct miscdevice dev;
	struct semaphore sem;
} *status_stp;

static int status_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	if (down_interruptible(&status_stp->sem))
		return - ERESTARTSYS;
	
	ret = __raw_readb(DETECT_BASE);

	switch(cmd){
		case CMD_GET_AGE_STATUS:
			ret &= DETECT_AGE_MASK;
			break;

		case CMD_GET_PA_STATUS:
			ret &= DETECT_PA_MASK;
			break;

		default:
			up(&status_stp->sem);
			return -ENOTTY;
	}

	up(&status_stp->sem);
	return (ret ? 1 : 0);
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

	__raw_writeb(sta, STATUS_BASE);
	
	up(&status_stp->sem);
	return sizeof(sta);
}

static ssize_t status_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	unsigned char sta;

	if (down_interruptible(&status_stp->sem))
		return - ERESTARTSYS;
	
	sta = __raw_readb(STATUS_BASE);

	if (copy_to_user(buf, &sta, sizeof(sta))){
		printk("BSP: %s fail copy_to_user\n", __FUNCTION__);
		up(&status_stp->sem);
		return - EFAULT;
	}

	up(&status_stp->sem);

	return sizeof(sta);
}

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
	int ret = 0;

	// malloc and initial
	status_stp = kmalloc(sizeof(struct status_st), GFP_KERNEL);
	if (!status_stp)
	{
		ret = - ENOMEM;
		goto fail_malloc;
	}
	memset(status_stp, 0, sizeof(struct status_st));
	init_MUTEX(&status_stp->sem);
	status_stp->dev.minor = MISC_DYNAMIC_MINOR;
	status_stp->dev.name = "g200wo_status";
	status_stp->dev.fops = &status_fops;

	// register device
	ret = misc_register(&status_stp->dev);
	if (ret){
		printk("BSP: %s fail to register device\n", __FUNCTION__);
		goto fail_registe;
	}

	printk("G200WO STATUS Driver installed\n");
	return 0;

fail_registe:
	kfree(status_stp);

fail_malloc:
	printk("Fail to install G200WO STATUS driver\n");
	return ret;
}

static void __exit status_exit(void)
{
	misc_deregister(&status_stp->dev);
	kfree(status_stp);
	printk("G200WO STATUS Driver removed\n");
}

module_init(status_init);
module_exit(status_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO STATUS DETECT");
MODULE_LICENSE("GPL");
