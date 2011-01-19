/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : g420sd
 ::  ::  ::       ::           :::      FileName   : status.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-08-06 18:23:05
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

#include <g420sd/g420sd_hw.h>

struct{
	struct miscdevice dev;
	struct semaphore sem;
}rs422_st;

static ssize_t rs422_time_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	unsigned char sta;

	if (down_interruptible(&rs422_st.sem))
		return - ERESTARTSYS;

	if (copy_from_user(&sta, buf, sizeof(sta)))
	{
		printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
		up(&rs422_st.sem);
		return  - EFAULT;
	}
	__raw_writeb(sta, RS422_BASE);

	up(&rs422_st.sem);
	return sizeof(sta);
}

static ssize_t rs422_time_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	unsigned char sta;

	if (down_interruptible(&rs422_st.sem))
		return - ERESTARTSYS;

	sta = __raw_readb(RS422_BASE);

	if (copy_to_user(buf, &sta, sizeof(sta))){
		printk("BSP: %s fail copy_to_user\n", __FUNCTION__);
		up(&rs422_st.sem);
		return - EFAULT;
	}
	up(&rs422_st.sem);
	return sizeof(sta);
}

static const struct file_operations rs422_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = rs422_time_read,
	.write  = rs422_time_write,
	.ioctl  = NULL,
};

static int __init rs422_time_init(void)
{
	int ret = 0;

	// Initial structure
	memset(&rs422_st, 0, sizeof(rs422_st));
	init_MUTEX(&rs422_st.sem);
	rs422_st.dev.minor = MISC_DYNAMIC_MINOR;
	rs422_st.dev.name = "nts_rs422_byte_time";
	rs422_st.dev.fops = &rs422_fops;
	// Registe device
	ret = misc_register(&rs422_st.dev);
	if (ret) {
		printk("BSP: %s fail to registe device\n", __FUNCTION__);
	} else {
		printk("BSP: g420sd rs422_byte_time driver installed\n");
	}
	return ret;
}

static void __exit rs422_time_exit(void)
{
	misc_deregister(&rs422_st.dev);
	printk("BSP: g420sd rs422_byte_time driver removed\n");
}

module_init(rs422_time_init);
module_exit(rs422_time_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("g420sd rs422_byte_time");
MODULE_LICENSE("GPL");
