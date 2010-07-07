/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : dac.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-07-07 14:39:31
::::    :::     ::::::      ::::::::    Version    : v0.3

Description
	2010-07-07	Change cdev to miscdevices
	v0.3	Move pins define to g200wo_hw.h
		Remove some header file
*/

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include "g200wo_hw.h"
#include "dac.h"

struct dac_st
{
	struct miscdevice dev;
	struct semaphore sem;
	unsigned char data;
} *dac_stp;

void dac5682z_io_write(unsigned int base, unsigned char port, unsigned char active)
{
	dac_stp->data &= ~port;
	if(active)	dac_stp->data |= port;

	__raw_writeb(dac_stp->data, base);
}

void dac5682z_write(unsigned int base, unsigned char addr, unsigned char data)
{
	int i;

	//set r/w bit and MODE bits
	addr &= 0x1F;

	//clear all pins
	dac5682z_io_write(base, DAC5682Z_ALL, 0);

	//active SEN
	dac5682z_io_write(base, DAC5682Z_SEN, 1);

	//address
	for (i=0; i<8; i++){
		//SDATA is latched on SCLK's falledge
		dac5682z_io_write(base, DAC5682Z_SDATA, (addr & 0x80));

		dac5682z_io_write(base, DAC5682Z_SCLK, 1);
		dac5682z_io_write(base, DAC5682Z_SCLK, 0);

		addr <<= 1;
	}

	// data
	for (i=0; i<8; i++){
		//SDATA is valid in SCLK's falledge
		dac5682z_io_write(base, DAC5682Z_SDATA, (data & 0x80));

		dac5682z_io_write(base, DAC5682Z_SCLK, 1);
		dac5682z_io_write(base, DAC5682Z_SCLK, 0);

		data<<=1;
	}

	//clear all pins
	dac5682z_io_write(base, DAC5682Z_ALL, 0);
}

unsigned char dac5682z_read(unsigned int base, unsigned char addr)
{
	unsigned char data=0;
	int i;

	//set r/w bit and MODE bits
	addr &= 0x1F;
	addr |= 0x80;

	//clear all pins
	dac5682z_io_write(base, DAC5682Z_ALL, 0);

	//active SEN
	dac5682z_io_write(base, DAC5682Z_SEN, 1);

	// address
	for (i=0; i<8; i++){
		//SDATA is latched on SCLK's falledge
		dac5682z_io_write(base, DAC5682Z_SDATA, (addr & 0x80));

		dac5682z_io_write(base, DAC5682Z_SCLK, 1);
		dac5682z_io_write(base, DAC5682Z_SCLK, 0);

		addr <<= 1;
	}
	// data
	for (i=0; i<8; i++){
		//SDATA is valid in SCLK's falledge
		dac5682z_io_write(base, DAC5682Z_SCLK, 1);
		dac5682z_io_write(base, DAC5682Z_SCLK, 0);

		if(DAC5682Z_SDATA & __raw_readb(base))
			data+=1;

		data<<=1;
	}

	//clear all pins
	dac5682z_io_write(base, DAC5682Z_ALL, 0);

	return data;
}


static ssize_t dac_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	struct dac_elem elem;
	unsigned int base;

	if (down_interruptible(&dac_stp->sem))
		return - ERESTARTSYS;
	if (copy_from_user(&elem, buf, sizeof(struct dac_elem))){
		printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
		up(&dac_stp->sem);
		return  - EFAULT;
	}

	base = (elem.dev == DEV_DAC_A) ? DACA_BASE : DACB_BASE;

	dac5682z_write(base, elem.addr, elem.data);

	up(&dac_stp->sem);
	return sizeof(struct dac_elem);
}

static ssize_t dac_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	struct dac_elem elem;
	unsigned int base;

	if (down_interruptible(&dac_stp->sem))
		return - ERESTARTSYS;
	if (copy_from_user(&elem, buf, sizeof(struct dac_elem))){
		printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
		up(&dac_stp->sem);
		return  - EFAULT;
	}

	base = (elem.dev == DEV_DAC_A) ? DACA_BASE : DACB_BASE;

	elem.data = dac5682z_read(base, elem.addr);

	if (copy_to_user(buf, &elem, sizeof(struct dac_elem))){
		printk("BSP: %s fail copy_to_user\n", __FUNCTION__);
		up(&dac_stp->sem);
		return - EFAULT;
	}
	up(&dac_stp->sem);
	return sizeof(struct dac_elem);
}

static const struct file_operations dac_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = dac_read,
	.write  = dac_write,
};

static int __init dac_init(void)
{
	int ret = 0;

	// malloc and initial
	dac_stp = kmalloc(sizeof(struct dac_st), GFP_KERNEL);
	if (!dac_stp) {
		ret = - ENOMEM;
		goto fail_malloc;
	}
	memset(dac_stp, 0, sizeof(struct dac_st));
	init_MUTEX(&dac_stp->sem);
	dac_stp->dev.minor = MISC_DYNAMIC_MINOR;
	dac_stp->dev.name = "g200wo_tx_dac";
	dac_stp->dev.fops = &dac_fops;

	// register device
	ret = misc_register(&dac_stp->dev);
	if (ret) {
		printk("BSP: %s fail register device\n", __FUNCTION__);
		goto fail_register;
	}

	printk("G200WO TX_DAC Driver installed\n");
	return 0;

fail_register:
	kfree(dac_stp);

fail_malloc:
	printk("Fail to install G200WO TX_DAC driver\n");
	return ret;
}

static void __exit dac_exit(void)
{
	misc_deregister(&dac_stp->dev);
	kfree(dac_stp);
	printk("G200WO TX_DAC Driver removed\n");
}

module_init(dac_init);
module_exit(dac_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO TX_DAC");
MODULE_LICENSE("GPL");
