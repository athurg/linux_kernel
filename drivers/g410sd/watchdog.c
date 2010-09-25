/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Athurg.Feng
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G410SD
 ::  ::  ::       ::           :::      FileName   : watchdog.c
 ::   :: ::       ::             ::     Generate   : 2010.06.24
 ::    ::::       ::       ::      ::   Update     : 2010-09-02 13:09:50
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	2010-07-07	Change cdev to miscdevices
	None

Changelog
	v0.2
		move watchdog_ioctl to watchdog_write
	v0.1
		initial
*/
#include <linux/fs.h>
#include <linux/kernel.h>


#include <asm/io.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>

#include <g410sd/g410sd_hw.h>

#define WATCHDOG_FEED_VALUE	0x5A

struct{
	struct miscdevice dev;
}watchdog_st;

static ssize_t watchdog_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	__raw_writeb(WATCHDOG_FEED_VALUE, WATCHDOG_BASE);
	return size;
}

static const struct file_operations watchdog_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = NULL,
	.write  = watchdog_write,
	.ioctl  = NULL,
};

static int __init watchdog_init(void)
{
	int ret = 0;

	// malloc and initial memory
	memset(&watchdog_st, 0, sizeof(watchdog_st));

	watchdog_st.dev.minor = MISC_DYNAMIC_MINOR;
	watchdog_st.dev.name = "g410sd_watchdog";
	watchdog_st.dev.fops = &watchdog_fops;

	// Registe device
	ret = misc_register(&watchdog_st.dev);
	if (ret)
		printk("BSP: %s fail to registe device\n", __FUNCTION__);
	else
		printk("BSP: G410SD WATCHDOG Driver installed\n");

	return ret;
}

static void __exit watchdog_exit(void)
{
	misc_deregister(&watchdog_st.dev);
	printk("BSP: 200WO WATCHDOG Driver removed\n");
}

module_init(watchdog_init);
module_exit(watchdog_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G410SD WatchDog");
MODULE_LICENSE("GPL");
