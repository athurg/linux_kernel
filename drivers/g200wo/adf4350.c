/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Athurg.Feng
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : adf4350.c
 ::   :: ::       ::             ::     Generate   : 2009.05.31
 ::    ::::       ::       ::      ::   Update     : 2010-07-01 11:54:06
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	None
*/
//------------------------------------------------------------------------------
// include
//------------------------------------------------------------------------------
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/module.h>

#include <asm/io.h>
#include <linux/semaphore.h>

#include "hardware.h"
#include "adf4350.h"

struct adf4350_st
{
	struct cdev cdev;
	struct semaphore sem;
	unsigned char data;
};

struct adf4350_st *adf4350_stp;


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
	int ret = 0, err = 0;
	dev_t devno;

	// register chrdev
	devno = MKDEV(MAJ_LED, MIN_LED);
	ret = register_chrdev_region(devno, 1, "adf4350");
	if (ret<0){
		printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
		return ret;
	}

	// alloc dev
	adf4350_stp = kmalloc(sizeof(struct adf4350_st), GFP_KERNEL);
	if (!adf4350_stp){
		ret = - ENOMEM;
		goto fail_malloc;
	}

	memset(adf4350_stp, 0, sizeof(struct adf4350_st));
	init_MUTEX(&adf4350_stp->sem);

	cdev_init(&adf4350_stp->cdev, &adf4350_fops);
	adf4350_stp->cdev.owner = THIS_MODULE;
	adf4350_stp->cdev.ops = &adf4350_fops;

	// add cdev
	err = cdev_add(&adf4350_stp->cdev, devno, 1);
	if (err){
		printk("BSP: %s fail cdev_add\n", __FUNCTION__);
		goto fail_remap;
	}

	printk("G200WO ADF4350 Driver installed\n");
	return 0;

fail_remap:
	kfree(adf4350_stp);

fail_malloc:
	unregister_chrdev_region(devno, 1);
	printk("Fail to install G200WO ADF4350 driver\n");
	return ret;
}

static void __exit adf4350_exit(void)
{
	dev_t devno;

	cdev_del(&adf4350_stp->cdev);
	kfree(adf4350_stp);

	devno = MKDEV(MAJ_ADF4350, MIN_ADF4350);
	unregister_chrdev_region(devno, 1);

	printk("G200WO ADF4350 Driver removed\n");
}

module_init(adf4350_init);
module_exit(adf4350_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO ADF4350");
MODULE_LICENSE("GPL");
