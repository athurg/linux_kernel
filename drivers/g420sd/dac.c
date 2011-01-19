/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : x223WO
 ::  ::  ::       ::           :::      FileName   : dac.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-12-11 16:15:19
::::    :::     ::::::      ::::::::    Version    : v0.3

Description
	2010-07-07	Change cdev to miscdevices
	v0.3	Move pins define to g420sd_hw.h
		Remove some header file
*/

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include <g420sd/g420sd_hw.h>
#include <g420sd/dac.h>

struct{
	struct miscdevice dev;
	struct semaphore sem;
	unsigned char data;
}dac_st;

void ad9122_io_write(unsigned int base, unsigned char port, unsigned char active)
{
	dac_st.data &= ~port;
	if(active)	dac_st.data |= port;

	__raw_writeb(dac_st.data, base);		
}

void ad9122_write(unsigned int base, unsigned char addr, unsigned char data)
{
	int i;

	//set r/w bit and MODE bits
	addr &= 0x7F;  //w_n  Mode

	//clear all pins
	ad9122_io_write(base, AD9122_ALL, 0);

	//unactive SCS	
	ad9122_io_write(base, AD9122_SCS, 1);
	
	//active SCS
	ad9122_io_write(base, AD9122_SCS, 0);	

	//address
	for (i=0; i<8; i++){
		//SDATA is latched on SCLK's falledge
		ad9122_io_write(base, AD9122_SDIO, (addr & 0x80));
		ad9122_io_write(base, AD9122_SCLK, 1);
		ad9122_io_write(base, AD9122_SCLK, 0);
		addr <<= 1;
	}

	// data
	for (i=0; i<8; i++){
		//SDATA is valid in SCLK's falledge
		ad9122_io_write(base, AD9122_SDIO, (data & 0x80));
		ad9122_io_write(base, AD9122_SCLK, 1);
		ad9122_io_write(base, AD9122_SCLK, 0);
		data<<=1;
	}

	//clear all pins
	ad9122_io_write(base, AD9122_ALL, 0);
	//unactive SEN	
	ad9122_io_write(base, AD9122_SCS, 1);	

}

unsigned char ad9122_read(unsigned int base, unsigned char addr)
{
	unsigned char data=0;
	int i;

	//set r/w bit and MODE bits
	addr &= 0x7F;
	addr |= 0x80;

	//clear all pins
	ad9122_io_write(base, AD9122_ALL, 0);
	//unactive SCS	
	ad9122_io_write(base, AD9122_SCS, 1);
	//active SCS
	ad9122_io_write(base, AD9122_SCS, 0);

	// address
	for (i=0; i<8; i++){
		//SDATA is latched on SCLK's falledge
		ad9122_io_write(base, AD9122_SDIO, (addr & 0x80));
		ad9122_io_write(base, AD9122_SCLK, 1);
		ad9122_io_write(base, AD9122_SCLK, 0);
		addr <<= 1;
	}
	// set SDATA to 1
	ad9122_io_write(base, AD9122_SDIO, 1);
	// data
	for (i=0; i<8; i++){
		//SDATA is valid in SCLK's falledge
		data <<= 1;
		ndelay(100);
		if(AD9122_SDOUT & __raw_readb(base))
			data += 1;
		ad9122_io_write(base, AD9122_SCLK, 1);
		ad9122_io_write(base, AD9122_SCLK, 0);
	}

	//clear all pins
	ad9122_io_write(base, AD9122_ALL, 0);
	//unactive SEN	
	ad9122_io_write(base, AD9122_SCS, 1);		
	return data;		
}

static ssize_t dac_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	struct dac_elem elem;
	unsigned int base;

	if (down_interruptible(&dac_st.sem))
		return - ERESTARTSYS;
	if (copy_from_user(&elem, buf, sizeof(struct dac_elem))){
		printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
		up(&dac_st.sem);
		return  - EFAULT;
	}
	switch(elem.dev) 
	{
	case DEV_DAC_A:
		base = DACA_BASE;
		break;
	case DEV_DAC_B:
		base = DACB_BASE;
		break;				
	case DEV_DAC_C:
		base = DACC_BASE;
		break;		
	case DEV_DAC_D:
		base = DACD_BASE;
		break;	
	default:
		up(&dac_st.sem);
		return  - EFAULT;
	}
	ad9122_write(base, elem.addr, elem.data);
	up(&dac_st.sem);
	return sizeof(struct dac_elem);
}

static ssize_t dac_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	struct dac_elem elem;
	unsigned int base;

	if (down_interruptible(&dac_st.sem))
		return - ERESTARTSYS;
	if (copy_from_user(&elem, buf, sizeof(struct dac_elem))){
		printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
		up(&dac_st.sem);
		return  - EFAULT;
	}

	switch(elem.dev) 
	{
	case DEV_DAC_A:
		base = DACA_BASE;
		break;
	case DEV_DAC_B:
		base = DACB_BASE;
		break;				
	case DEV_DAC_C:
		base = DACC_BASE;
		break;		
	case DEV_DAC_D:
		base = DACD_BASE;
		break;	
	default:
		up(&dac_st.sem);
		return  - EFAULT;
	}

	//switch bus to read
	ad9122_write(base, 0, 0x80);
	elem.data = ad9122_read(base, elem.addr);
	if (copy_to_user(buf, &elem, sizeof(struct dac_elem))){
		printk("BSP: %s fail copy_to_user\n", __FUNCTION__);
		up(&dac_st.sem);
		return - EFAULT;
	}
	up(&dac_st.sem);
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
		printk("BSP: g420sd TX DAC Driver installed\n");

	return ret;
}

static void __exit dac_exit(void)
{
	misc_deregister(&dac_st.dev);
	printk("BSP: g420sd TX_DAC Driver removed\n");
}

module_init(dac_init);
module_exit(dac_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("g420sd TX_DAC");
MODULE_LICENSE("GPL");
