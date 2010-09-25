/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G410SD
 ::  ::  ::       ::           :::      FileName   : reset.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-07-28 15:30:17
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

#include <g410sd/g410sd_hw.h>
#include <g410sd/reset.h>

struct{
	struct miscdevice dev;
	struct semaphore sem;
}reset_st;

static int reset_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	if (down_interruptible(&reset_st.sem))
		return - ERESTARTSYS;

	if(cmd != CMD_RESET){
		ret = -ENOTTY;
	}else{
		__raw_writeb((arg & 0xFF), RESET_BASE);
	}

	up(&reset_st.sem);
	return ret;
}

static const struct file_operations reset_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = NULL,
	.write  = NULL,
	.ioctl  = reset_ioctl,
};

static int __init reset_init(void)
{
	int ret = 0;

	// Initial structure
	memset(&reset_st, 0, sizeof(reset_st));

	init_MUTEX(&reset_st.sem);

	reset_st.dev.minor = MISC_DYNAMIC_MINOR;
	reset_st.dev.name = "g410sd_reset";
	reset_st.dev.fops = &reset_fops;

	// register device
	ret = misc_register(&reset_st.dev);
	if (ret)
		printk("BSP: %s fail to register device\n", __FUNCTION__);
	else
		printk("BSP: G410SD RESET Driver installed\n");

	return ret;
}

static void __exit reset_exit(void)
{
	misc_deregister(&reset_st.dev);
	printk("BSP: G410SD RESET Driver removed\n");
}

module_init(reset_init);
module_exit(reset_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G410SD RESET");
MODULE_LICENSE("GPL");
