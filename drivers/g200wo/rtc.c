//------------------------------------------------------------------------------
// rtc.c
// MPC8313E intrenal RTC driver
// 2009-06-02 NTS Ray.Zhou
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// include
//------------------------------------------------------------------------------
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/semaphore.h>

#include <mach/platform.h>
#include <mach/lpc32xx_rtc.h>

#include "hardware.h"

struct rtc_st
{
	struct cdev cdev;
	struct semaphore sem;
};

struct rtc_st *rtc_stp;

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

	//stop rtc -> load value -> start rtc
	__raw_writel(RTC_CNTR_DIS, io_p2v(RTC_CTRL(RTC_BASE)));
	__raw_writel(rtc_time, io_p2v(RTC_UCOUNT(RTC_BASE)));
	__raw_writel(0, io_p2v(RTC_CTRL(RTC_BASE)));

	up(&rtc_stp->sem);
	return sizeof(time_t);
}

static ssize_t rtc_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	time_t rtc_time;

	if (down_interruptible(&rtc_stp->sem))
		return - ERESTARTSYS;
	
	rtc_time = __raw_readl(io_p2v(RTC_UCOUNT(RTC_BASE)));
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
	int ret = 0, err = 0;
	dev_t devno;

	// register chrdev
	devno = MKDEV(MAJ_RTC, MIN_RTC);
	ret = register_chrdev_region(devno, 1, "g200wo_rtc");
	if (ret<0){
		printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
		return ret;
	}

	// alloc dev
	rtc_stp = kmalloc(sizeof(struct rtc_st), GFP_KERNEL);
	if (!rtc_stp){
		ret = - ENOMEM;
		goto fail_malloc;
	}
	memset(rtc_stp, 0, sizeof(struct rtc_st));

	init_MUTEX(&rtc_stp->sem);

	// add cdev
	cdev_init(&rtc_stp->cdev, &rtc_fops);
	rtc_stp->cdev.owner = THIS_MODULE;
	rtc_stp->cdev.ops = &rtc_fops;
	err = cdev_add(&rtc_stp->cdev, devno, 1);
	if (err){
		printk("BSP: %s fail cdev_add\n", __FUNCTION__);
		goto fail_remap;
	}

	//initial RTC
	__raw_writel(0, io_p2v(RTC_CTRL(RTC_BASE)));

	printk("NTS RTC Driver installed\n");
	return 0;

fail_remap:
	kfree(rtc_stp);

fail_malloc:
	unregister_chrdev_region(devno, 1);
	printk("Fail to install NTS RTC driver\n");
	return ret;
}

static void __exit rtc_exit(void)
{
	dev_t devno;

	cdev_del(&rtc_stp->cdev);
	kfree(rtc_stp);
	devno = MKDEV(MAJ_RTC, MIN_RTC);
	unregister_chrdev_region(devno, 1);
	printk("NTS RTC Driver removed\n");
}

module_init(rtc_init);
module_exit(rtc_exit);

MODULE_AUTHOR("Ray.Zhou, <ray.zhou@nts-intl.com>");
MODULE_DESCRIPTION("NTS RTC");
MODULE_LICENSE("GPL");
