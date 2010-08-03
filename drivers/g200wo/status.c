/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      FileName   : status.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-08-02 14:39:46
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

#include <g200wo/g200wo_hw.h>
#include <g200wo/status.h>

//FIXME:
//	Maybe we should define the next two macro in headers file
//	to share with user-space, why not?
#define DETECT_AGE(a)		(_BIT(0) & a)
#define DETECT_PA(a)		(_BIT(1) & a)

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

	if (cmd == CMD_GET_AGE_STATUS)
		ret = DETECT_AGE(ret);
	else if (cmd == CMD_GET_PA_STATUS)
		ret = DETECT_PA(ret);
	else
		ret = -ENOTTY;

	// We got _BIT(1) or _BIT(2) from hardware register
	// but user-space expect simple ACTIVE (1) or INACTIVE(0)
	if(ret>=0)
		ret = ret ? 1 : 0;

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
	status_st.dev.name = "g200wo_status";
	status_st.dev.fops = &status_fops;

	// Registe device
	ret = misc_register(&status_st.dev);
	if (ret)
		printk("BSP: %s fail to registe device\n", __FUNCTION__);
	else
		printk("BSP: G200WO STATUS Driver installed\n");

	return ret;
}

static void __exit status_exit(void)
{
	misc_deregister(&status_st.dev);
	printk("BSP: 200WO STATUS Driver removed\n");
}

module_init(status_init);
module_exit(status_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO STATUS and DETECT");
MODULE_LICENSE("GPL");
