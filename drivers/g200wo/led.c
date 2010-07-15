/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : led.c
 ::   :: ::       ::             ::     Generate   : 2009.05.31
 ::    ::::       ::       ::      ::   Update     : 2010-07-15 18:56:57
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	2010-07-07	Change cdev to miscdevices
	None
*/
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>

#include <asm/io.h>
#include <mach/lpc32xx_gpio.h>

#include <g200wo/g200wo_hw.h>
#include <g200wo/led.h>

struct led_st
{
	struct miscdevice dev;
	struct semaphore sem;
} *led_stp;

static int led_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	if (down_interruptible(&led_stp->sem))
		return - ERESTARTSYS;
	
	if(cmd == CMD_LED_ON)
		__raw_writel(arg, GPIO_P3_OUTP_SET(GPIO_IOBASE));
	else if(cmd == CMD_LED_OFF)
		__raw_writel(arg, GPIO_P3_OUTP_CLR(GPIO_IOBASE));
	else
		ret = -ENOTTY;
	
	up(&led_stp->sem);
	return ret;
}

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
	int ret = 0;

	// malloc and intial
	led_stp = kmalloc(sizeof(struct led_st), GFP_KERNEL);
	if (!led_stp){
		ret = - ENOMEM;
		goto fail_malloc;
	}

	memset(led_stp, 0, sizeof(struct led_st));
	init_MUTEX(&led_stp->sem);
	led_stp->dev.minor = MISC_DYNAMIC_MINOR;
	led_stp->dev.name = "g200wo_led";
	led_stp->dev.fops = &led_fops;

	// register device
	ret = misc_register(&led_stp->dev);
	if (ret){
		printk("BSP: %s fail to register device\n", __FUNCTION__);
		goto fail_registe;
	}

	printk("G200WO LED Driver installed\n");
	return 0;

fail_registe:
	kfree(led_stp);

fail_malloc:
	printk("Fail to install G200WO LED driver\n");
	return ret;
}

static void __exit led_exit(void)
{
	misc_deregister(&led_stp->dev);
	kfree(led_stp);
	printk("G200WO LED Driver removed\n");
}

module_init(led_init);
module_exit(led_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO Status LED");
MODULE_LICENSE("GPL");
