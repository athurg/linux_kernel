/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : led.c
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
#include "led.h"

struct led_st
{
	struct cdev cdev;
	struct semaphore sem;
};

struct led_st *led_stp;

static int led_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	if (down_interruptible(&led_stp->sem))
		return - ERESTARTSYS;
	/*
	if((arg!=ARG_LED_RUN_OK)
		&& (arg!=ARG_LED_RUN_ERR)
		&& (arg!=ARG_LED_VSWR1)
		&& (arg!=ARG_LED_VSWR2))
		ret = -ENOTTY;
	else*/
	if(cmd == CMD_LED_ON)
		__raw_writel(OUTP_STATE_GPO(arg),
				GPIO_P3_OUTP_SET(io_p2v(GPIO_BASE)));
	else if(cmd == CMD_LED_OFF)
		__raw_writel(OUTP_STATE_GPO(arg),
				GPIO_P3_OUTP_CLR(io_p2v(GPIO_BASE)));
	else
		ret = -ENOTTY;
	
	return ret;
}

//------------------------------------------------------------------------------
// register module
//------------------------------------------------------------------------------
static const struct file_operations led_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = NULL,
	.write  = NULL,
	.ioctl  = led_ioctl,
};

static int __init led_init(void)
{
	int ret = 0, err = 0;
	dev_t devno;

	// register chrdev
	devno = MKDEV(MAJ_LED, MIN_LED);
	ret = register_chrdev_region(devno, 1, "led");
	if (ret<0){
		printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
		return ret;
	}

	// alloc dev
	led_stp = kmalloc(sizeof(struct led_st), GFP_KERNEL);
	if (!led_stp){
		ret = - ENOMEM;
		goto fail_malloc;
	}

	memset(led_stp, 0, sizeof(struct led_st));
	init_MUTEX(&led_stp->sem);
	
	// add cdev
	cdev_init(&led_stp->cdev, &led_fops);
	led_stp->cdev.owner = THIS_MODULE;
	led_stp->cdev.ops = &led_fops;
	err = cdev_add(&led_stp->cdev, devno, 1);
	if (err){
		printk("BSP: %s fail cdev_add\n", __FUNCTION__);
		goto fail_remap;
	}

	printk("G200WO LED Driver installed\n");
	return 0;

fail_remap:
	kfree(led_stp);

fail_malloc:
	unregister_chrdev_region(devno, 1);
	printk("Fail to install G200WO LED driver\n");
	return ret;
}

static void __exit led_exit(void)
{
	dev_t devno;
	cdev_del(&led_stp->cdev);
	kfree(led_stp);
	devno = MKDEV(MAJ_LED, MIN_LED);
	unregister_chrdev_region(devno, 1);
	printk("G200WO LED Driver removed\n");
}

module_init(led_init);
module_exit(led_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO Status LED");
MODULE_LICENSE("GPL");
