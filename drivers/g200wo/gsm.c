/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : gsm.c
 ::   :: ::       ::             ::     Generate   : 2009.05.31
 ::    ::::       ::       ::      ::   Update     : 2010.06.09
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
#include <linux/semaphore.h>	//semaphore Define
#include <mach/platform.h>	//GPIO Operate Define
#include <mach/lpc32xx_gpio.h>	//GPIO Operate Define

#include "hardware.h"	//Hardware Regs Addr Define
#include "gsm.h"

struct gsm_st
{
	struct cdev cdev;
	struct semaphore sem;
	volatile unsigned int __iomem *regp;
};

struct gsm_st *gsm_stp;

static int gsm_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	arg = (~arg) & 0x1F;

	if (down_interruptible(&gsm_stp->sem))
		return - ERESTARTSYS;
	
	if(cmd == CMD_GSM_PWR){//Manage Power
		if(arg == ARG_GSM_PWR_ON){
			__raw_writel( (OUTP_STATE_GPO(12) | OUTP_STATE_GPO(13) ),
					GPIO_P3_OUTP_CLR(io_p2v(GPIO_BASE)));
		}else if(arg == ARG_GSM_PWR_OFF){
			__raw_writel( (OUTP_STATE_GPO(12) | OUTP_STATE_GPO(13) ),
					GPIO_P3_OUTP_SET(io_p2v(GPIO_BASE)));
		}else{
			ret = -ENOTTY;
		}
	}else if(cmd == CMD_GSM_ATT){	//manage ATT
		__raw_writel(arg<<6,	GPIO_P3_OUTP_CLR(io_p2v(GPIO_BASE)));
	}else{
		ret = -ENOTTY;
	}
	
	return ret;
}

//------------------------------------------------------------------------------
// register module
//------------------------------------------------------------------------------
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
	int ret = 0, err = 0;
	dev_t devno;

	// register chrdev
	devno = MKDEV(MAJ_GSM, MIN_GSM);
	ret = register_chrdev_region(devno, 1, "gsm");
	if (ret<0){
		printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
		return ret;
	}

	// alloc dev
	gsm_stp = kmalloc(sizeof(struct gsm_st), GFP_KERNEL);
	if (!gsm_stp){
		ret = - ENOMEM;
		goto fail_malloc;
	}

	memset(gsm_stp, 0, sizeof(struct gsm_st));
	init_MUTEX(&gsm_stp->sem);
	
	// add cdev
	cdev_init(&gsm_stp->cdev, &gsm_fops);
	gsm_stp->cdev.owner = THIS_MODULE;
	gsm_stp->cdev.ops = &gsm_fops;
	err = cdev_add(&gsm_stp->cdev, devno, 1);
	if (err){
		printk("BSP: %s fail cdev_add\n", __FUNCTION__);
		goto fail_remap;
	}
	
	//set port mux
	__raw_writel(_BIT(6) | _BIT(7) | _BIT(8) | _BIT(9) | _BIT(10) | _BIT(12) | _BIT(13),
			GPIO_P3_MUX_CLR(io_p2v(GPIO_BASE)));

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
	iounmap(gsm_stp->regp);
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
