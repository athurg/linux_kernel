/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : adc.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010.06.24
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	None
*/

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/semaphore.h>

#include "hardware.h"
#include "adc.h"

#define ADS62C17_SDOUT	(1<<3)
#define ADS62C17_SEN	(1<<2)
#define ADS62C17_SDATA	(1<<1)
#define ADS62C17_SCLK	(1<<0)
#define ADS62C17_ALL	0xF

struct adc_st
{
	struct cdev cdev;
	struct semaphore sem;
	unsigned int base;	//base address
	unsigned char data;
};

struct adc_st *adc_stp;

static void ads62c17_io_write(unsigned int base, unsigned int port, unsigned char active)
{
	adc_stp->data &= ~port;
	if(active)	adc_stp->data |= port;

	__raw_writeb(adc_stp->data, io_p2v(base));
}

static void ads62c17_write(unsigned int base, unsigned char addr, unsigned char data)
{
	int i;

	ads62c17_io_write(base, ADS62C17_SEN, 0);
	ads62c17_io_write(base, ADS62C17_SCLK, 1);

	for(i=0; i<8; i++){
		ads62c17_io_write(base, ADS62C17_SDATA, (addr & 0x80));
		ads62c17_io_write(base, ADS62C17_SCLK, 0);
		ads62c17_io_write(base, ADS62C17_SCLK, 1);
		addr <<= 1;
	}

	for(i=0; i<8; i++){
		ads62c17_io_write(base, ADS62C17_SDATA, (data & 0x80));
		ads62c17_io_write(base, ADS62C17_SCLK, 0);
		ads62c17_io_write(base, ADS62C17_SCLK, 1);
		data <<= 1;
	}
	ads62c17_io_write(base, ADS62C17_SEN, 1);
}
#define ads62c17_read_enable(base)		ads62c17_write(base,0,1)
#define ads62c17_write_enable(base)		ads62c17_write(base,0,0)

static unsigned char ads62c17_read(unsigned int base, unsigned char addr)
{
	int i, tmp=0, data=0;

	ads62c17_io_write(base, ADS62C17_SEN, 0);
	ads62c17_io_write(base, ADS62C17_SCLK, 1);

	for(i=0; i<8; i++){
		ads62c17_io_write(base, ADS62C17_SDATA, (addr & 0x80));
		ads62c17_io_write(base, ADS62C17_SCLK, 0);
		ads62c17_io_write(base, ADS62C17_SCLK, 1);
		addr <<= 1;
	}

	for(i=0; i<8; i++){
		ads62c17_io_write(base, ADS62C17_SCLK, 0);

		tmp = ADS62C17_SDATA & __raw_readb(io_p2v(base));
		if(tmp)
			data += 1;

		ads62c17_io_write(base, ADS62C17_SCLK, 1);
		data <<= 1;
	}

	ads62c17_io_write(base, ADS62C17_SEN, 1);
	return data;
}

static ssize_t adc_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	struct adc_elem elem;
	unsigned int base;

	if (down_interruptible(&adc_stp->sem))
		return - ERESTARTSYS;
	
	if (copy_from_user(&elem, buf, sizeof(struct adc_elem)))
	{
		printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
		up(&adc_stp->sem);
		return  - EFAULT;
	}

	base = (elem.dev == DEV_ADC_A) ? ADDR_ADCA : ADDR_ADCB;

	ads62c17_write_enable(base);
	ads62c17_write(base, elem.addr, elem.data);

	up(&adc_stp->sem);
	return sizeof(struct adc_elem);
}

static ssize_t adc_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	struct adc_elem elem;
	unsigned int base;


	if (down_interruptible(&adc_stp->sem))
		return - ERESTARTSYS;
	
	if (copy_from_user(&elem, buf, sizeof(struct adc_elem))){
		printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
		up(&adc_stp->sem);
		return  - EFAULT;
	}

	base = (elem.dev == DEV_ADC_A) ? ADDR_ADCA : ADDR_ADCB;

	ads62c17_read_enable(base);
	elem.data = ads62c17_read(base, elem.addr);

	if (copy_to_user(buf, &elem, sizeof(struct adc_elem))){
		printk("BSP: %s fail copy_to_user\n", __FUNCTION__);
		up(&adc_stp->sem);
		return - EFAULT;
	}

	up(&adc_stp->sem);
	return sizeof(struct adc_elem);
}

static const struct file_operations adc_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = adc_read,
	.write  = adc_write,
};

static int __init adc_init(void)
{
	int ret = 0, err = 0;
	dev_t devno;

	// register chrdev
	devno = MKDEV(MAJ_ADC, MIN_ADC);
	ret = register_chrdev_region(devno, 1, "g200wo_rx_adc");
	if (ret<0)
	{
		printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
		return ret;
	}
	// alloc dev
	adc_stp = kmalloc(sizeof(struct adc_st), GFP_KERNEL);
	if (!adc_stp)
	{
		ret = - ENOMEM;
		goto fail_malloc;
	}
	memset(adc_stp, 0, sizeof(struct adc_st));
	init_MUTEX(&adc_stp->sem);
	// add cdev
	cdev_init(&adc_stp->cdev, &adc_fops);
	adc_stp->cdev.owner = THIS_MODULE;
	adc_stp->cdev.ops = &adc_fops;
	err = cdev_add(&adc_stp->cdev, devno, 1);
	if (err)
	{
		printk("BSP: %s fail cdev_add\n", __FUNCTION__);
		goto fail_remap;
	}

	printk("NTS RX_ADC Driver installed\n");
	return 0;

fail_remap:
	kfree(adc_stp);

fail_malloc:
	unregister_chrdev_region(devno, 1);
	printk("Fail to install NTS RX_ADC driver\n");
	return ret;
}

static void __exit adc_exit(void)
{
	dev_t devno;

	cdev_del(&adc_stp->cdev);
	kfree(adc_stp);
	devno = MKDEV(MAJ_ADC, MIN_ADC);
	unregister_chrdev_region(devno, 1);
	printk("NTS RX_ADC Driver removed\n");
}

module_init(adc_init);
module_exit(adc_exit);

MODULE_AUTHOR("Ray.Zhou, <ray.zhou@nts-intl.com>");
MODULE_DESCRIPTION("NTS RX_ADC");
MODULE_LICENSE("GPL");
