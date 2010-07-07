/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : rtc.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-07-07 15:35:15
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

#include <linux/time.h>
#include <mach/platform.h>
#include <mach/lpc32xx_rtc.h>

#include "g200wo_hw.h"

struct rtc_st
{
	struct miscdevice dev;
	struct semaphore sem;
} *rtc_stp;

static ssize_t rtc_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	time_t rtc_time;

	if (down_interruptible(&rtc_stp->sem))
		return - ERESTARTSYS;

	if (copy_from_user(&rtc_time, buf, sizeof(time_t))){
		printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
		up(&rtc_stp->sem);
		return  - EFAULT;
	}

	__raw_writel(RTC_CNTR_DIS, RTC_CTRL(RTC_IOBASE));	//stop RTC
	__raw_writel(rtc_time, RTC_UCOUNT(RTC_IOBASE));		//load value
	__raw_writel(0, RTC_CTRL(RTC_IOBASE));		//start RTC

	up(&rtc_stp->sem);
	return sizeof(time_t);
}

static ssize_t rtc_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	time_t rtc_time;

	if (down_interruptible(&rtc_stp->sem))
		return - ERESTARTSYS;
	
	rtc_time = __raw_readl(RTC_UCOUNT(RTC_IOBASE));

	if (copy_to_user(buf, &rtc_time, sizeof(time_t))){
		printk("BSP: %s fail copy_to_user\n", __FUNCTION__);
		up(&rtc_stp->sem);
		return - EFAULT;
	}

	up(&rtc_stp->sem);
	return sizeof(time_t);
}


static const struct file_operations rtc_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = rtc_read,
	.write  = rtc_write,
};

static int __init rtc_init(void)
{
	int ret = 0;

	// malloc and initial
	rtc_stp = kmalloc(sizeof(struct rtc_st), GFP_KERNEL);
	if (!rtc_stp){
		ret = - ENOMEM;
		goto fail_malloc;
	}
	memset(rtc_stp, 0, sizeof(struct rtc_st));
	init_MUTEX(&rtc_stp->sem);
	rtc_stp->dev.minor = MISC_DYNAMIC_MINOR;
	rtc_stp->dev.name = "g200wo_rtc";
	rtc_stp->dev.fops = &rtc_fops;

	// register device
	ret = misc_register(&rtc_stp->dev);
	if (ret){
		printk("BSP: %s fail to register device\n", __FUNCTION__);
		goto fail_registe;
	}

	//initial (enable RTC)
	__raw_writel(0, RTC_CTRL(RTC_IOBASE));

	printk("G200WO RTC Driver installed\n");
	return 0;

fail_registe:
	kfree(rtc_stp);

fail_malloc:
	printk("Fail to install G200WO RTC driver\n");
	return ret;
}

static void __exit rtc_exit(void)
{
	misc_deregister(&rtc_stp->dev);
	kfree(rtc_stp);
	printk("G200WO RTC Driver removed\n");
}

module_init(rtc_init);
module_exit(rtc_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO RTC");
MODULE_LICENSE("GPL");
