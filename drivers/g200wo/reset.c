/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : reset.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-07-07 15:32:46
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	2010-07-07	Change cdev to miscdevices
	None
*/
#include <linux/fs.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>

#include <g200wo/g200wo_hw.h>
#include <g200wo/reset.h>

struct reset_st
{
	struct miscdevice dev;
	struct semaphore sem;
} *reset_stp;

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
	int ret = 0;

	// malloc and initial
	reset_stp = kmalloc(sizeof(struct reset_st), GFP_KERNEL);
	if (!reset_stp){
		ret = - ENOMEM;
		goto fail_malloc;
	}
	memset(reset_stp, 0, sizeof(struct reset_st));
	init_MUTEX(&reset_stp->sem);
	reset_stp->dev.minor = MISC_DYNAMIC_MINOR;
	reset_stp->dev.name = "g200wo_reset";
	reset_stp->dev.fops = &reset_fops;


	// register device
	ret = misc_register(&reset_stp->dev);
	if (ret){
		printk("BSP: %s fail to register device\n", __FUNCTION__);
		goto fail_registe;
	}

	printk("G200WO RESET Driver installed\n");
	return 0;

fail_registe:
	kfree(reset_stp);

fail_malloc:

	printk("Fail to install G200WO RESET driver\n");
	return ret;
}

static void __exit reset_exit(void)
{
	misc_deregister(&reset_stp->dev);
	kfree(reset_stp);
	printk("G200WO RESET Driver removed\n");
}

module_init(reset_init);
module_exit(reset_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO RESET");
MODULE_LICENSE("GPL");
