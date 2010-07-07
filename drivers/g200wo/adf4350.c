/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Athurg.Feng
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : adf4350.c
 ::   :: ::       ::             ::     Generate   : 2009.05.31
 ::    ::::       ::       ::      ::   Update     : 2010-07-07 14:25:13
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	2010-07-07	Change cdev to miscdevices
	None
*/
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>

#include <asm/io.h>

#include "g200wo_hw.h"
#include "adf4350.h"

struct adf4350_st
{
	struct miscdevice dev;
	struct semaphore sem;
	unsigned char data;
} *adf4350_stp;


static void adf4350_write(unsigned int base_addr, unsigned int port, unsigned active)
{
	adf4350_stp->data &= port;
	if(active)	adf4350_stp->data |= port;

	__raw_writeb(adf4350_stp->data, base_addr);
}

static int adf4350_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0, base_addr, i;

	if (down_interruptible(&adf4350_stp->sem))
		return - ERESTARTSYS;

	switch(cmd){
		case CMD_LO_TRXA_SET:
		case CMD_LO_TRXA_GET_LD:
			base_addr = LO_TRXA_BASE;
			break;
		case CMD_LO_RXB_SET:
		case CMD_LO_RXB_GET_LD:
			base_addr = LO_RXB_BASE;
			break;
		case CMD_LO_TXB_SET:
		case CMD_LO_TXB_GET_LD:
			base_addr = LO_TXB_BASE;
			break;
		default:
			return -ENOTTY;

	}

	if(_IOC_DIR(cmd) == IOC_OUT){
		ret = ADF4350_LD & __raw_readb(base_addr);
		ret = ret ? 1 : 0;
	}else if(_IOC_DIR(cmd) == IOC_IN){
		//clear all pins and then active LE
		adf4350_write(base_addr, ADF4350_ALL, 0);
		adf4350_write(base_addr, ADF4350_LE, 1);

		//32 bits data
		for(i=0; i<32; i++){
			adf4350_write(base_addr, ADF4350_DAT, (arg & 0x80000000));

			adf4350_write(base_addr, ADF4350_CLK, 1);
			adf4350_write(base_addr, ADF4350_CLK, 0);
			arg <<= 1;
		}
		//clear all pins and then active LE
		adf4350_write(base_addr, ADF4350_ALL, 0);
	}else{
		return -ENOTTY;
	}

	up(&adf4350_stp->sem);
	return ret;
}

static const struct file_operations adf4350_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = NULL,
	.write  = NULL,
	.ioctl  = adf4350_ioctl,
};

static int __init adf4350_init(void)
{
	int ret = 0;

	// malloc and initial
	adf4350_stp = kmalloc(sizeof(struct adf4350_st), GFP_KERNEL);
	if (!adf4350_stp){
		ret = - ENOMEM;
		goto fail_malloc;
	}
	memset(adf4350_stp, 0, sizeof(struct adf4350_st));
	init_MUTEX(&adf4350_stp->sem);

	adf4350_stp->dev.minor = MISC_DYNAMIC_MINOR;
	adf4350_stp->dev.name = "g200wo_adf4350";
	adf4350_stp->dev.fops = &adf4350_fops;

	ret = misc_register(&adf4350_stp->dev);
	if (ret){
		printk("BSP: %s fail regiter device\n", __FUNCTION__);
		goto fail_register;
	}

	printk("G200WO ADF4350 Driver installed\n");
	return 0;

fail_register:
	kfree(adf4350_stp);

fail_malloc:
	printk("Fail to install G200WO ADF4350 driver\n");
	return ret;
}

static void __exit adf4350_exit(void)
{
	misc_deregister(&adf4350_stp->dev);
	kfree(adf4350_stp);
	printk("G200WO ADF4350 Driver removed\n");
}

module_init(adf4350_init);
module_exit(adf4350_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO ADF4350");
MODULE_LICENSE("GPL");
