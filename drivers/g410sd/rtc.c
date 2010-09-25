/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G410SD
 ::  ::  ::       ::           :::      FileName   : rtc.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-07-28 15:34:07
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

#include <g410sd/g410sd_hw.h>

struct{
	struct miscdevice dev;
	struct semaphore sem;
}rtc_st;

static ssize_t rtc_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	time_t rtc_time;

	if (down_interruptible(&rtc_st.sem))
		return - ERESTARTSYS;

	if (copy_from_user(&rtc_time, buf, sizeof(time_t))){
		printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
		up(&rtc_st.sem);
		return  - EFAULT;
	}

	// stop-->load-->start
	__raw_writel(RTC_CNTR_DIS, RTC_CTRL(RTC_IOBASE));
	__raw_writel(rtc_time, RTC_UCOUNT(RTC_IOBASE));	
	__raw_writel(0, RTC_CTRL(RTC_IOBASE));

	up(&rtc_st.sem);
	return sizeof(time_t);
}

static ssize_t rtc_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	time_t rtc_time;

	if (down_interruptible(&rtc_st.sem))
		return - ERESTARTSYS;

	rtc_time = __raw_readl(RTC_UCOUNT(RTC_IOBASE));

	if (copy_to_user(buf, &rtc_time, sizeof(time_t))){
		printk("BSP: %s fail copy_to_user\n", __FUNCTION__);
		up(&rtc_st.sem);
		return - EFAULT;
	}

	up(&rtc_st.sem);
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

	// Initial structure
	memset(&rtc_st, 0, sizeof(rtc_st));

	init_MUTEX(&rtc_st.sem);

	rtc_st.dev.minor = MISC_DYNAMIC_MINOR;
	rtc_st.dev.name = "g410sd_rtc";
	rtc_st.dev.fops = &rtc_fops;

	// register device
	ret = misc_register(&rtc_st.dev);
	if (ret) {
		printk("BSP: %s fail to register device\n", __FUNCTION__);
	} else {
		__raw_writel(0, RTC_CTRL(RTC_IOBASE));	//initial (enable RTC)
		printk("BSP: G410SD RTC Driver installed\n");
	}

	return ret;
}

static void __exit rtc_exit(void)
{
	misc_deregister(&rtc_st.dev);
	printk("BSP: 200WO RTC Driver removed\n");
}

module_init(rtc_init);
module_exit(rtc_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G410SD RTC");
MODULE_LICENSE("GPL");
