/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Athurg.Feng
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : tmp125.c
 ::   :: ::       ::             ::     Generate   : 2009.05.31
 ::    ::::       ::       ::      ::   Update     : 2010-07-07 15:40:18
::::    :::     ::::::      ::::::::    Version    : v0.1

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

#include "g200wo_hw.h"

struct tmp125_st
{
	struct miscdevice dev;
	struct semaphore sem;
} *tmp125_stp;

void inline tmp125_io_write(int port, int active)
{
	if(active)
		__raw_writel(port, GPIO_P3_OUTP_SET(GPIO_IOBASE));
	else
		__raw_writel(port, GPIO_P3_OUTP_CLR(GPIO_IOBASE));
}

static ssize_t tmp125_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	int i,data;
	if (down_interruptible(&tmp125_stp->sem))
		return - ERESTARTSYS;
	
	//Chip Select active
	tmp125_io_write(TMP125_CS_N, 0);

	//Lead zero bit
	tmp125_io_write(TMP125_SCLK, 0);
	tmp125_io_write(TMP125_SCLK, 1);

	//10 bits valid data
	for(i=0; i<10; i++){
		tmp125_io_write(TMP125_SCLK, 0);
		tmp125_io_write(TMP125_SCLK, 1);

		if(TMP125_DOUT & __raw_readb(GPIO_P3_INP_STATE(GPIO_BASE)))
			data += 1;

		data <<=1;
	}

	//5 bits dummy
	for(i=0; i<5; i++){
		tmp125_io_write(TMP125_SCLK, 0);
		tmp125_io_write(TMP125_SCLK, 1);
	}

	//Chip Select inactive
	tmp125_io_write(TMP125_CS_N, 1);
	
	if (copy_to_user(buf, &data, sizeof(int))){
		printk("BSP: %s fail copy_to_user\n", __FUNCTION__);
		up(&tmp125_stp->sem);
		return - EFAULT;
	}

	up(&tmp125_stp->sem);
	
	return sizeof(int);
}

static const struct file_operations tmp125_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = tmp125_read,
	.write  = NULL,
	.ioctl  = NULL,
};

static int __init tmp125_init(void)
{
	int ret = 0;

	// malloc and initial
	tmp125_stp = kmalloc(sizeof(struct tmp125_st), GFP_KERNEL);
	if (!tmp125_stp){
		ret = - ENOMEM;
		goto fail_malloc;
	}

	memset(tmp125_stp, 0, sizeof(struct tmp125_st));
	init_MUTEX(&tmp125_stp->sem);
	tmp125_stp->dev.minor = MISC_DYNAMIC_MINOR;
	tmp125_stp->dev.name = "g200wo_tmp125";
	tmp125_stp->dev.fops = &tmp125_fops;

	// register device
	ret = misc_register(&tmp125_stp->dev);
	if (ret){
		printk("BSP: %s fail to register device\n", __FUNCTION__);
		goto fail_registe;
	}

	// set port mux
	__raw_writel((TMP125_SCLK | TMP125_DOUT | TMP125_CS_N), GPIO_P3_MUX_CLR(GPIO_IOBASE));

	printk("G200WO TMP125 Driver installed\n");
	return 0;

fail_registe:
	kfree(tmp125_stp);

fail_malloc:

	printk("Fail to install G200WO TMP125 driver\n");
	return ret;
}

static void __exit tmp125_exit(void)
{
	misc_deregister(&tmp125_stp->dev);
	kfree(tmp125_stp);
	printk("G200WO TMP125 Driver removed\n");
}

module_init(tmp125_init);
module_exit(tmp125_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO TMP125 temperature driver");
MODULE_LICENSE("GPL");
