/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      FileName  : adc.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-07-28 12:00:18
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

#include <g200wo/g200wo_hw.h>
#include <g200wo/adc.h>

struct{
	struct miscdevice dev;
	struct semaphore sem;
	unsigned int base;	//base address
	unsigned char data;
}adc_st;


/* Functions */
static void ads62c17_io_write(unsigned int base, unsigned int port, unsigned char active);
static void ads62c17_write(unsigned int base, unsigned char addr, unsigned char data);
static unsigned char ads62c17_read(unsigned int base, unsigned char addr);
static ssize_t adc_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos);
static ssize_t adc_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos);


//IO pins write Operation
//NOTE:
//	we use a global variable 'adc_st.data' to cache the status of IO pins
static void ads62c17_io_write(unsigned int base, unsigned int port, unsigned char active)
{
	//clear the orig value
	adc_st.data &= ~port;

	//active only when new value is 'ACTIVE'
	if(active)	adc_st.data |= port;

	//update the hardware
	__raw_writeb(adc_st.data, base);
}


//Write Protocol Defined in ADS62C17's Datasheet
static void ads62c17_write(unsigned int base, unsigned char addr, unsigned char data)
{
	int i;

	//clear all pins
	ads62c17_io_write(base, ADS62C17_ALL, 0);

	//active SEN
	ads62c17_io_write(base, ADS62C17_SEN, 1);

	for(i=0; i<8; i++){
		ads62c17_io_write(base, ADS62C17_SDATA, (addr & 0x80));//From MSB to LSB

		//SDATA latched when SCLK falledge
		ads62c17_io_write(base, ADS62C17_SCLK, 1);
		ads62c17_io_write(base, ADS62C17_SCLK, 0);

		addr <<= 1;
	}

	for(i=0; i<8; i++){
		ads62c17_io_write(base, ADS62C17_SDATA, (data & 0x80));//From MSB to LSB

		ads62c17_io_write(base, ADS62C17_SCLK, 1);
		ads62c17_io_write(base, ADS62C17_SCLK, 0);
		data <<= 1;
	}

	//clear all pins
	ads62c17_io_write(base, ADS62C17_ALL, 0);
}
//NOTE:
//We should execution a SWITCH COMMAND while switch between WRITE and READ mode
//refer to DATASHEET of ADS62C17
#define ads62c17_read_enable(base)		ads62c17_write(base,0,1)
#define ads62c17_write_enable(base)		ads62c17_write(base,0,0)


//Read Protocol Defined in ADS62C17's Datasheet
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

	if (down_interruptible(&adc_st.sem))
		return - ERESTARTSYS;

	if (copy_from_user(&elem, buf, sizeof(elem)))
	{
		printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
		up(&adc_st.sem);
		return  - EFAULT;
	}

	base = (elem.dev == DEV_ADC_A) ? ADCA_BASE : ADCB_BASE;

	ads62c17_write_enable(base);
	ads62c17_write(base, elem.addr, elem.data);

	up(&adc_st.sem);
	return sizeof(elem);
}

static ssize_t adc_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	struct adc_elem elem;
	unsigned int base;


	if (down_interruptible(&adc_st.sem))
		return - ERESTARTSYS;

	if (copy_from_user(&elem, buf, sizeof(elem))){
		printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
		up(&adc_st.sem);
		return  - EFAULT;
	}

	base = (elem.dev == DEV_ADC_A) ? ADCA_BASE : ADCB_BASE;

	ads62c17_read_enable(base);
	elem.data = ads62c17_read(base, elem.addr);

	if (copy_to_user(buf, &elem, sizeof(elem))){
		printk("BSP: %s fail copy_to_user\n", __FUNCTION__);
		up(&adc_st.sem);
		return - EFAULT;
	}

	up(&adc_st.sem);
	return sizeof(elem);
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
	int ret = 0;

	// initial device structure
	memset(&adc_st, 0, sizeof(adc_st));

	init_MUTEX(&adc_st.sem);

	adc_st.dev.minor = MISC_DYNAMIC_MINOR;
	adc_st.dev.name = "g200wo_rx_adc";
	adc_st.dev.fops = &adc_fops;

	// register device
	ret = misc_register(&adc_st.dev);

	if (ret)
		printk("BSP: %s fail register device\n", __FUNCTION__);
	else
		printk("BSP: G200WO RX_ADC Driver installed\n");

	return ret;
}

static void __exit adc_exit(void)
{
	misc_deregister(&adc_st.dev);
	printk("BSP: G200WO RX_ADC Driver removed\n");
}

module_init(adc_init);
module_exit(adc_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO RX_ADC");
MODULE_LICENSE("GPL");

