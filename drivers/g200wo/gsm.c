/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : gsm.c
 ::   :: ::       ::             ::     Generate   : 2009.05.31
 ::    ::::       ::       ::      ::   Update     : 2010-07-01 11:51:40
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	None
*/
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/module.h>

#include <asm/io.h>
#include <linux/semaphore.h>

#include "g200wo_hw.h"
#include "gsm.h"

// define the port as bus
#define GSM_PWR_BUS	(GSM_PWRON_N | GSM_VCHARGE)
#define GSM_ATT_BUS	(GSM_ATT0 | GSM_ATT1 | GSM_ATT2 | GSM_ATT3 | GSM_ATT4)

struct gsm_st
{
	struct cdev cdev;
	struct semaphore sem;
};

struct gsm_st *gsm_stp;

static int gsm_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	if (down_interruptible(&gsm_stp->sem))
		return - ERESTARTSYS;
	
	if(cmd == CMD_GSM_PWR){//Manage Power
		if(arg == ARG_GSM_PWR_ON){
			__raw_writel(GSM_PWR_BUS, GPIO_P3_OUTP_CLR(GPIO_IOBASE));
		}else if(arg == ARG_GSM_PWR_OFF){
			__raw_writel(GSM_PWR_BUS, GPIO_P3_OUTP_SET(GPIO_IOBASE));
		}else{
			ret = -ENOTTY;
		}
	}else if(cmd == CMD_GSM_ATT){	//manage ATT
		arg = ((~arg) & 0x1F);	//We need only 5 bits from LSB

		//First clean, then load new value
		__raw_writel(GSM_ATT_BUS, GPIO_P3_OUTP_CLR(GPIO_IOBASE));
		__raw_writel(arg<<6, GPIO_P3_OUTP_SET(GPIO_IOBASE));
	}else{
		ret = -ENOTTY;
	}

	up(&gsm_stp->sem);
	return ret;
}

static const struct file_operations gsm_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = NULL,
	.write  = NULL,
	.ioctl  = gsm_ioctl,
};

static int __init gsm_init(void)
{
	int ret = 0;
	dev_t devno;

	// register chrdev
	devno = MKDEV(MAJ_GSM, MIN_GSM);
	ret = register_chrdev_region(devno, 1, "gsm");
	if (ret<0) {
		printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
		return ret;
	}

	// alloc dev
	gsm_stp = kmalloc(sizeof(struct gsm_st), GFP_KERNEL);
	if (!gsm_stp) {
		ret = - ENOMEM;
		goto fail_malloc;
	}

	memset(gsm_stp, 0, sizeof(struct gsm_st));
	init_MUTEX(&gsm_stp->sem);

	cdev_init(&gsm_stp->cdev, &gsm_fops);
	gsm_stp->cdev.owner = THIS_MODULE;
	gsm_stp->cdev.ops = &gsm_fops;

	// add cdev
	ret = cdev_add(&gsm_stp->cdev, devno, 1);
	if (ret) {
		printk("BSP: %s fail cdev_add\n", __FUNCTION__);
		goto fail_remap;
	}
	
	//set port as GPIO
	__raw_writel(GSM_ATT_BUS | GSM_PWR_BUS, GPIO_P3_MUX_CLR(GPIO_IOBASE));

	printk("G200WO GSM Driver instalgsm\n");
	return 0;

fail_remap:
	kfree(gsm_stp);

fail_malloc:
	unregister_chrdev_region(devno, 1);

	printk("Fail to install G200WO GSM driver\n");
	return ret;
}

static void __exit gsm_exit(void)
{
	dev_t devno;

	cdev_del(&gsm_stp->cdev);
	kfree(gsm_stp);

	devno = MKDEV(MAJ_GSM, MIN_GSM);
	unregister_chrdev_region(devno, 1);

	printk("G200WO GSM Driver removed\n");
}

module_init(gsm_init);
module_exit(gsm_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO GSM control");
MODULE_LICENSE("GPL");
