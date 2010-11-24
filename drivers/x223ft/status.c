/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : x223FT
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

#include <x223ft/x223ft_hw.h>
#include <x223ft/status.h>

//FIXME:
//	Maybe we should define the next two macro in headers file
//	to share with user-space, why not?
#define DETECT_AGE(a)		((_BIT(0) & a) & 0x01)
#define DETECT_PA(a)		(((_BIT(1) & a) >> 1 ) & 0x01)
#define DETECT_BDA_ID(a)	((((_BIT(3) | _BIT(2)) & a) >> 2 ) & 0x03)

struct{
	struct miscdevice dev;
	struct semaphore sem;
}status_st;

static int status_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	if (down_interruptible(&status_st.sem))
		return - ERESTARTSYS;

	ret = __raw_readb(DETECT_BASE);
	printk("\n\r status register value =0x%x \n",ret);
	if (cmd == CMD_GET_AGE_STATUS)
		ret = DETECT_AGE(ret);
	else if (cmd == CMD_GET_PA_BOARD_STATUS)
		ret = DETECT_PA(ret);
	else if (cmd == CMD_GET_BDA_ID_STATUS)
		ret = DETECT_BDA_ID(ret);
	else
		ret = -ENOTTY;

	up(&status_st.sem);
	return ret;
}

static ssize_t status_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	unsigned char sta;

	if (down_interruptible(&status_st.sem))
		return - ERESTARTSYS;

	if (copy_from_user(&sta, buf, sizeof(sta)))
	{
		printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
		up(&status_st.sem);
		return  - EFAULT;
	}
	__raw_writeb(sta, STATUS_BASE);

	up(&status_st.sem);
	return sizeof(sta);
}

static ssize_t status_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	unsigned char sta;

	if (down_interruptible(&status_st.sem))
		return - ERESTARTSYS;

	sta = __raw_readb(STATUS_BASE);

	if (copy_to_user(buf, &sta, sizeof(sta))){
		printk("BSP: %s fail copy_to_user\n", __FUNCTION__);
		up(&status_st.sem);
		return - EFAULT;
	}
	up(&status_st.sem);

	return sizeof(sta);
}

static const struct file_operations status_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = status_read,
	.write  = status_write,
	.ioctl  = status_ioctl,
};

static int __init status_init(void)
{
	int ret = 0;

	// Initial structure
	memset(&status_st, 0, sizeof(status_st));

	init_MUTEX(&status_st.sem);

	status_st.dev.minor = MISC_DYNAMIC_MINOR;
	status_st.dev.name = "nts_status";
	status_st.dev.fops = &status_fops;

	// Registe device
	ret = misc_register(&status_st.dev);
	if (ret) {
		printk("BSP: %s fail to registe device\n", __FUNCTION__);
	} else {
		printk("BSP: x223ft STATUS Driver installed\n");
		ret = __raw_readb(DETECT_BASE);
		printk("BSP:	PA board is %s connect\n", DETECT_PA(ret) ? "not":"");
		printk("BSP:	AGE STATUS is %s\n", DETECT_AGE(ret) ? "not":"");
		printk("BSP:	BDA_ID 0x%01x\n", DETECT_BDA_ID(ret));
		ret = 0;
	}

	return ret;
}

static void __exit status_exit(void)
{
	misc_deregister(&status_st.dev);
	printk("BSP: X223FT STATUS Driver removed\n");
}

module_init(status_init);
module_exit(status_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("X223FT STATUS and DETECT");
MODULE_LICENSE("GPL");
