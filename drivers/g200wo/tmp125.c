/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Athurg.Feng
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : tmp125.c
 ::   :: ::       ::             ::     Generate   : 2009.05.31
 ::    ::::       ::       ::      ::   Update     : 2010.06.22
::::    :::     ::::::      ::::::::    Version    : v0.1

Description
	None
*/
//------------------------------------------------------------------------------
// include
//------------------------------------------------------------------------------
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/semaphore.h>	//semaphore Define

#include "hardware.h"	//Hardware Regs Addr Define

struct tmp125_st
{
	struct cdev cdev;
	struct semaphore sem;
};

struct tmp125_st *tmp125_stp;

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
	tmp125_io_write(TMP125_CS_N | TMP125_SCLK, 0);

	//Lead zero bit
	tmp125_io_write(TMP125_SCLK, 1);
	tmp125_io_write(TMP125_SCLK, 0);

	//10 bits valid data
	for(i=0; i<10; i++){
		tmp125_io_write(TMP125_SCLK, 1);
		if(TMP125_DOUT & __raw_readb(GPIO_P3_INP_STATE(GPIO_BASE)))
			data += 1;
		tmp125_io_write(TMP125_SCLK, 0);
		data <<=1;
	}

	//5 bits dummy
	for(i=0; i<5; i++){
		tmp125_io_write(TMP125_SCLK, 1);
		tmp125_io_write(TMP125_SCLK, 0);
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
	int ret = 0, err = 0;
	dev_t devno;

	// register chrdev
	devno = MKDEV(MAJ_TMP125, MIN_TMP125);
	ret = register_chrdev_region(devno, 1, "tmp125");
	if (ret<0){
		printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
		return ret;
	}

	// alloc dev
	tmp125_stp = kmalloc(sizeof(struct tmp125_st), GFP_KERNEL);
	if (!tmp125_stp){
		ret = - ENOMEM;
		goto fail_malloc;
	}

	memset(tmp125_stp, 0, sizeof(struct tmp125_st));
	init_MUTEX(&tmp125_stp->sem);
	
	// add cdev
	cdev_init(&tmp125_stp->cdev, &tmp125_fops);
	tmp125_stp->cdev.owner = THIS_MODULE;
	tmp125_stp->cdev.ops = &tmp125_fops;
	err = cdev_add(&tmp125_stp->cdev, devno, 1);
	if (err){
		printk("BSP: %s fail cdev_add\n", __FUNCTION__);
		goto fail_remap;
	}

	// set port mux
	__raw_writel((_BIT(22) | _BIT(18) | _BIT(16) ),
			GPIO_P3_MUX_CLR(GPIO_BASE));

	printk("G200WO TMP125 Driver instaltmp125\n");
	return 0;

fail_remap:
	kfree(tmp125_stp);

fail_malloc:
	unregister_chrdev_region(devno, 1);
	printk("Fail to install G200WO TMP125 driver\n");
	return ret;
}

static void __exit tmp125_exit(void)
{
	dev_t devno;
	cdev_del(&tmp125_stp->cdev);
	kfree(tmp125_stp);
	devno = MKDEV(MAJ_TMP125, MIN_TMP125);
	unregister_chrdev_region(devno, 1);
	printk("G200WO TMP125 Driver removed\n");
}

module_init(tmp125_init);
module_exit(tmp125_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO Status TMP125");
MODULE_LICENSE("GPL");
