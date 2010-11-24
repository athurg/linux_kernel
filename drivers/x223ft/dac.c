/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : x223ft
 ::  ::  ::       ::           :::      FileName   : dac.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-11-12 15:12:42
::::    :::     ::::::      ::::::::    Version    : v0.3

Description
	2010-07-07	Change cdev to miscdevices
	v0.3	Move pins define to x223ft_hw.h
		Remove some header file
*/

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include <x223ft/x223ft_hw.h>
#include <x223ft/adda.h>

struct{
	struct miscdevice dev;
	struct semaphore sem;
	unsigned char data;
}dac_st;

void dac5682z_io_write(unsigned int base, unsigned char port, unsigned char active)
{
	dac_st.data &= ~port;
	if(active)	dac_st.data |= port;

	__raw_writeb(dac_st.data, base);
}

void dac5682z_write(unsigned int base, unsigned char addr, unsigned char data)
{
	int i;

	//set r/w bit and MODE bits
	addr &= 0x1F;

	//clear all pins
	dac5682z_io_write(base, DAC5682Z_ALL, 0);

	//unactive SEN	
	dac5682z_io_write(base, DAC5682Z_SEN, 1);
	
	//active SEN
	dac5682z_io_write(base, DAC5682Z_SEN, 0);	

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
	//unactive SEN	
	dac5682z_io_write(base, DAC5682Z_SEN, 1);	
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
	//unactive SEN	
	dac5682z_io_write(base, DAC5682Z_SEN, 1);
	//active SEN
	dac5682z_io_write(base, DAC5682Z_SEN, 0);

	// address
	for (i=0; i<8; i++){
		//SDATA is latched on SCLK's falledge
		dac5682z_io_write(base, DAC5682Z_SDATA, (addr & 0x80));
		dac5682z_io_write(base, DAC5682Z_SCLK, 1);
		dac5682z_io_write(base, DAC5682Z_SCLK, 0);
		addr <<= 1;
	}
	// set SDATA to 1
	dac5682z_io_write(base, DAC5682Z_SDATA, 1);
	// data
	for (i=0; i<8; i++){
		//SDATA is valid in SCLK's falledge
		data<<=1;
		ndelay(100);
		if(DAC5682Z_SDOUT & __raw_readb(base))
			data+=1;
		dac5682z_io_write(base, DAC5682Z_SCLK, 1);
		dac5682z_io_write(base, DAC5682Z_SCLK, 0);
	}

	//clear all pins
	dac5682z_io_write(base, DAC5682Z_ALL, 0);
	//unactive SEN	
	dac5682z_io_write(base, DAC5682Z_SEN, 1);	
	
	return data;
}


static ssize_t dac_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	struct adda_elem elem;
	unsigned int base;

	if (down_interruptible(&dac_st.sem))
		return - ERESTARTSYS;
	if (copy_from_user(&elem, buf, sizeof(struct adda_elem))){
		printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
		up(&dac_st.sem);
		return  - EFAULT;
	}

	//base = (elem.dev == DEV_ADDA_A) ? DACA_BASE : DACB_BASE;
	base = DACA_BASE;

	dac5682z_write(base, elem.addr, elem.data);

	up(&dac_st.sem);
	return sizeof(struct adda_elem);
}

static ssize_t dac_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	struct adda_elem elem;
	unsigned int base;

	if (down_interruptible(&dac_st.sem))
		return - ERESTARTSYS;
	if (copy_from_user(&elem, buf, sizeof(struct adda_elem))){
		printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
		up(&dac_st.sem);
		return  - EFAULT;
	}

	//base = (elem.dev == DEV_ADDA_A) ? DACA_BASE : DACB_BASE;
	base = DACA_BASE;

	elem.data = dac5682z_read(base, elem.addr);

	if (copy_to_user(buf, &elem, sizeof(struct adda_elem))){
		printk("BSP: %s fail copy_to_user\n", __FUNCTION__);
		up(&dac_st.sem);
		return - EFAULT;
	}
	up(&dac_st.sem);
	return sizeof(struct adda_elem);
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
	memset(&dac_st, 0, sizeof(dac_st));

	init_MUTEX(&dac_st.sem);

	dac_st.dev.minor = MISC_DYNAMIC_MINOR;
	dac_st.dev.name = "nts_dac";
	dac_st.dev.fops = &dac_fops;

	// register device
	ret = misc_register(&dac_st.dev);
	if (ret)
		printk("BSP: %s fail register device\n", __FUNCTION__);
	else
		printk("BSP: x223ft DAC Driver installed\n");

	return ret;
}

static void __exit dac_exit(void)
{
	misc_deregister(&dac_st.dev);
	printk("BSP: X223FT DAC Driver removed\n");
}

module_init(dac_init);
module_exit(dac_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("x223ft DAC");
MODULE_LICENSE("GPL");
