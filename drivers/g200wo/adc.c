/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : adc.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-07-01 11:44:52
::::    :::     ::::::      ::::::::    Version    : v0.3

Description
	v0.3	Move pins define to hardware.h
		Remove some header file
*/

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/semaphore.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include "hardware.h"
#include "adc.h"

struct adc_st
{
	struct cdev cdev;
	struct semaphore sem;
	unsigned int base;	//base address
	unsigned char data;
};

struct adc_st *adc_stp;


/* Functions */
static void ads62c17_io_write(unsigned int base, unsigned int port, unsigned char active);
static void ads62c17_write(unsigned int base, unsigned char addr, unsigned char data);
static unsigned char ads62c17_read(unsigned int base, unsigned char addr);
static ssize_t adc_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos);
static ssize_t adc_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos);


static void ads62c17_io_write(unsigned int base, unsigned int port, unsigned char active)
{
	adc_stp->data &= ~port;
	if(active)	adc_stp->data |= port;

	__raw_writeb(adc_stp->data, base);
}

static void ads62c17_write(unsigned int base, unsigned char addr, unsigned char data)
{
	int i;

	//clear all pins
	ads62c17_io_write(base, ADS62C17_ALL, 0);
	
	//active SEN
	ads62c17_io_write(base, ADS62C17_SEN, 1);

	for(i=0; i<8; i++){
		//SDATA latched when SCLK falledge
		ads62c17_io_write(base, ADS62C17_SDATA, (addr & 0x80));

		ads62c17_io_write(base, ADS62C17_SCLK, 1);
		ads62c17_io_write(base, ADS62C17_SCLK, 0);

		addr <<= 1;
	}

	for(i=0; i<8; i++){
		ads62c17_io_write(base, ADS62C17_SDATA, (data & 0x80));

		ads62c17_io_write(base, ADS62C17_SCLK, 1);
		ads62c17_io_write(base, ADS62C17_SCLK, 0);
		data <<= 1;
	}

	//clear all pins
	ads62c17_io_write(base, ADS62C17_ALL, 0);
}
#define ads62c17_read_enable(base)		ads62c17_write(base,0,1)
#define ads62c17_write_enable(base)		ads62c17_write(base,0,0)

static unsigned char ads62c17_read(unsigned int base, unsigned char addr)
{
	int i, tmp=0, data=0;

	//clear all pins
	ads62c17_io_write(base, ADS62C17_ALL, 0);

	//active SEN
	ads62c17_io_write(base, ADS62C17_SEN, 1);

	for(i=0; i<8; i++){
		ads62c17_io_write(base, ADS62C17_SDATA, (addr & 0x80));

		ads62c17_io_write(base, ADS62C17_SCLK, 1);
		ads62c17_io_write(base, ADS62C17_SCLK, 0);

		addr <<= 1;
	}

	for(i=0; i<8; i++){
		ads62c17_io_write(base, ADS62C17_SCLK, 1);
		ads62c17_io_write(base, ADS62C17_SCLK, 0);

		tmp = ADS62C17_SDATA & __raw_readb(base);
		if(tmp)		data += 1;

		data <<= 1;
	}

	//clear all pins
	ads62c17_io_write(base, ADS62C17_ALL, 0);
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

	base = (elem.dev == DEV_ADC_A) ? ADCA_BASE : ADCB_BASE;

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

	base = (elem.dev == DEV_ADC_A) ? ADCA_BASE : ADCB_BASE;

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
	if (ret<0) {
		printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
		return ret;
	}

	// alloc dev
	adc_stp = kmalloc(sizeof(struct adc_st), GFP_KERNEL);
	if (!adc_stp) {
		ret = - ENOMEM;
		goto fail_malloc;
	}
	memset(adc_stp, 0, sizeof(struct adc_st));
	init_MUTEX(&adc_stp->sem);

	cdev_init(&adc_stp->cdev, &adc_fops);
	adc_stp->cdev.owner = THIS_MODULE;
	adc_stp->cdev.ops = &adc_fops;

	// add cdev
	ret = cdev_add(&adc_stp->cdev, devno, 1);
	if (ret) {
		printk("BSP: %s fail cdev_add\n", __FUNCTION__);
		goto fail_remap;
	}

	printk("G200WO RX_ADC Driver installed\n");
	return 0;

fail_remap:
	kfree(adc_stp);

fail_malloc:
	unregister_chrdev_region(devno, 1);

	printk("Fail to install G200WO RX_ADC driver\n");
	return ret;
}

static void __exit adc_exit(void)
{
	dev_t devno;

	cdev_del(&adc_stp->cdev);
	kfree(adc_stp);
	devno = MKDEV(MAJ_ADC, MIN_ADC);
	unregister_chrdev_region(devno, 1);
	printk("G200WO RX_ADC Driver removed\n");
}

module_init(adc_init);
module_exit(adc_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO RX_ADC");
MODULE_LICENSE("GPL");
