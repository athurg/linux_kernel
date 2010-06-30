/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : dac.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010.06.29
::::    :::     ::::::      ::::::::    Version    : v0.3

Description
	v0.3	Move pins define to hardware.h
		Remove some header file
*/

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/semaphore.h>

#include "hardware.h"
#include "dac.h"

struct dac_st
{
	struct cdev cdev;
	struct semaphore sem;
	unsigned char data;
};
struct dac_st *dac_stp;


//------------------------------------------------------------------------------
// io functions
//------------------------------------------------------------------------------
void dac5682z_io_write(unsigned int base, unsigned char port, unsigned char active)
{
	dac_stp->data &= ~port;
	if(active)	dac_stp->data |= port;

	__raw_writeb(dac_stp->data, base);
}

//------------------------------------------------------------------------------
// hardware functions
//------------------------------------------------------------------------------
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
		dac5682z_io_write(base, DAC5682Z_SCLK, 1);
		dac5682z_io_write(base, DAC5682Z_SDATA, (addr & 0x80));
		dac5682z_io_write(base, DAC5682Z_SCLK, 0);

		addr <<= 1;
	}

	// data
	for (i=0; i<8; i++){
		//SDATA is valid in SCLK's falledge
		dac5682z_io_write(base, DAC5682Z_SCLK, 1);
		dac5682z_io_write(base, DAC5682Z_SDATA, (data & 0x80));
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
		dac5682z_io_write(base, DAC5682Z_SCLK, 1);
		dac5682z_io_write(base, DAC5682Z_SDATA, (addr & 0x80));
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


//------------------------------------------------------------------------------
// module functions
//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
// register module
//------------------------------------------------------------------------------
static const struct file_operations dac_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = dac_read,
	.write  = dac_write,
};

static int __init dac_init(void)
{
	int ret = 0, err = 0;
	dev_t devno;
	// register chrdev
	devno = MKDEV(MAJ_DAC, MIN_DAC);
	ret = register_chrdev_region(devno, 1, "g200wo_dac");
	if (ret<0)
	{
		printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
		return ret;
	}
	// alloc dev
	dac_stp = kmalloc(sizeof(struct dac_st), GFP_KERNEL);
	if (!dac_stp)
	{
		ret = - ENOMEM;
		goto fail_malloc;
	}
	memset(dac_stp, 0, sizeof(struct dac_st));
	init_MUTEX(&dac_stp->sem);
	// add cdev
	cdev_init(&dac_stp->cdev, &dac_fops);
	dac_stp->cdev.owner = THIS_MODULE;
	dac_stp->cdev.ops = &dac_fops;
	err = cdev_add(&dac_stp->cdev, devno, 1);
	if (err)
	{
		printk("BSP: %s fail cdev_add\n", __FUNCTION__);
		goto fail_remap;
	}

	printk("NTS TX_DAC Driver installed\n");
	return 0;

fail_remap:
	kfree(dac_stp);

fail_malloc:
	unregister_chrdev_region(devno, 1);
	printk("Fail to install NTS TX_DAC driver\n");
	return ret;
}

static void __exit dac_exit(void)
{
	dev_t devno;

	cdev_del(&dac_stp->cdev);
	kfree(dac_stp);
	devno = MKDEV(MAJ_DAC, MIN_DAC);
	unregister_chrdev_region(devno, 1);
	printk("NTS TX_DAC Driver removed\n");
}

module_init(dac_init);
module_exit(dac_exit);

MODULE_AUTHOR("Ray.Zhou, <ray.zhou@nts-intl.com>");
MODULE_DESCRIPTION("NTS TX_DAC");
MODULE_LICENSE("GPL");
