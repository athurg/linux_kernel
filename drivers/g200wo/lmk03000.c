/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Athurg.Feng
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : lmk03000.c
 ::   :: ::       ::             ::     Generate   : 2009.05.31
 ::    ::::       ::       ::      ::   Update     : 2010-07-01 11:37:04
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	None
*/
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/module.h>

#include <asm/io.h>
#include <linux/semaphore.h>

#include "hardware.h"
#include "lmk03000.h"

struct lmk03000_st
{
	struct cdev cdev;
	struct semaphore sem;
	char data;
};

struct lmk03000_st *lmk03000_stp;

void lmk03000_io_write(unsigned int port, unsigned int active)
{
	lmk03000_stp->data &= ~port;
	if(active)	lmk03000_stp->data |= port;

	__raw_writeb(lmk03000_stp->data, LMK03000_BASE);
}

static int lmk03000_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0, i;

	if (down_interruptible(&lmk03000_stp->sem))
		return - ERESTARTSYS;

	switch(cmd){
		case CMD_SET_LMK03000_DATA:
			//LE => LOW
			lmk03000_io_write((LMK03000_LE | LMK03000_CLK), 0);

			//32 bits data
			for(i=0; i<32; i++){
				lmk03000_io_write(LMK03000_DAT, (arg&0x80000000));
				lmk03000_io_write(LMK03000_CLK, 1);
				lmk03000_io_write(LMK03000_CLK, 0);
				arg <<= 1;
			}

			//LE => HIGH
			lmk03000_io_write(LMK03000_LE, 1);
			break;

		case CMD_SET_LMK03000_SYNC:
			lmk03000_io_write(LMK03000_SYNC, arg);
			break;

		case CMD_SET_LMK03000_GOE:
			lmk03000_io_write(LMK03000_GOE, arg);
			break;

		case CMD_GET_LMK03000_LD:
			ret = LMK03000_LD & __raw_readb(LMK03000_BASE);
			ret = ret ? 1 : 0;
			break;
		default:
			ret = -ENOTTY;

	}
	
	return ret;
}

//------------------------------------------------------------------------------
// register module
//------------------------------------------------------------------------------
static const struct file_operations lmk03000_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = NULL,
	.write  = NULL,
	.ioctl  = lmk03000_ioctl,
};

static int __init lmk03000_init(void)
{
	int ret = 0;
	dev_t devno;

	// register chrdev
	devno = MKDEV(MAJ_LED, MIN_LED);
	ret = register_chrdev_region(devno, 1, "lmk03000");
	if (ret<0){
		printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
		return ret;
	}

	// alloc dev
	lmk03000_stp = kmalloc(sizeof(struct lmk03000_st), GFP_KERNEL);
	if (!lmk03000_stp){
		ret = - ENOMEM;
		goto fail_malloc;
	}

	memset(lmk03000_stp, 0, sizeof(struct lmk03000_st));
	init_MUTEX(&lmk03000_stp->sem);

	cdev_init(&lmk03000_stp->cdev, &lmk03000_fops);
	lmk03000_stp->cdev.owner = THIS_MODULE;
	lmk03000_stp->cdev.ops = &lmk03000_fops;

	// add cdev
	ret = cdev_add(&lmk03000_stp->cdev, devno, 1);
	if (ret){
		printk("BSP: %s fail cdev_add\n", __FUNCTION__);
		goto fail_remap;
	}

	printk("G200WO LMK03000 Driver installed\n");
	return 0;

fail_remap:
	kfree(lmk03000_stp);

fail_malloc:
	unregister_chrdev_region(devno, 1);
	printk("Fail to install G200WO LMK03000 driver\n");
	return ret;
}

static void __exit lmk03000_exit(void)
{
	dev_t devno;

	cdev_del(&lmk03000_stp->cdev);
	kfree(lmk03000_stp);

	devno = MKDEV(MAJ_LMK03000, MIN_LMK03000);
	unregister_chrdev_region(devno, 1);

	printk("G200WO LMK03000 Driver removed\n");
}

module_init(lmk03000_init);
module_exit(lmk03000_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO LMK03000");
MODULE_LICENSE("GPL");
