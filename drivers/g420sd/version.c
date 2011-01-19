/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : x223WO
 ::  ::  ::       ::           :::      FileName   : version.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-07-28 15:58:44
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
#include <g420sd/version.h>

#define KERNEL_VERSION      0x03

struct{
	struct miscdevice dev;
	struct semaphore sem;
}version_st;

static ssize_t version_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	struct version_elem elem;

	if (down_interruptible(&version_st.sem))
		return - ERESTARTSYS;

	elem.hard = __raw_readb(HARD_VER_BASE);
	elem.cpld = __raw_readb(CPLD_VER_BASE);
	elem.uboot = __raw_readb(UBOOT_VER_BASE);
	elem.kernel = KERNEL_VERSION;

	if (copy_to_user(buf, &elem, sizeof(elem))) {
		printk("BSP: %s fail copy_to_user\n", __FUNCTION__);
		up(&version_st.sem);
		return - EFAULT;
	}

	up(&version_st.sem);

	return sizeof(elem);
}

static const struct file_operations version_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = version_read,
	.write  = NULL,
};

static int __init version_init(void)
{
	int ret = 0;

	// Initial structure
	memset(&version_st, 0, sizeof(version_st));

	init_MUTEX(&version_st.sem);

	version_st.dev.minor = MISC_DYNAMIC_MINOR;
	version_st.dev.name = "nts_version";
	version_st.dev.fops = &version_fops;

	// Registe device
	ret = misc_register(&version_st.dev);

	if (ret)
		printk("BSP: %s fail to registe device\n", __FUNCTION__);
	else
		printk("BSP: g420sd version driver installed\n");

	return ret;
}

static void __exit version_exit(void)
{
	misc_deregister(&version_st.dev);
	printk("BSP: g420sd Version Driver removed\n");
}

module_init(version_init);
module_exit(version_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("g420sd VERSION");
MODULE_LICENSE("GPL");
