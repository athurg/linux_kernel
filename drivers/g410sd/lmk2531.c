/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Athurg.Feng
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G410SD
 ::  ::  ::       ::           :::      FileName   : lmk2531.c
 ::   :: ::       ::             ::     Generate   : 2009.05.31
 ::    ::::       ::       ::      ::   Update     : 2010-09-25 16:43:22
::::    :::     ::::::      ::::::::    Version    : v0.2

Description

	This Driver is based on adf4350.c

	2010-07-07	Change cdev to miscdevices
*/
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>

#include <asm/io.h>

#include <g410sd/g410sd_hw.h>
#include <g410sd/lmk2531.h>

struct{
	struct miscdevice dev;
	struct semaphore sem;
	unsigned char data;
}lmk2531_st;


static void lmk2531_iowrite(unsigned int base_addr, unsigned int port, unsigned active)
{
	lmk2531_st.data &= ~port;
	if(active)	lmk2531_st.data |= port;

	__raw_writeb(lmk2531_st.data, base_addr);
}

static int lmk2531_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0, base_addr=0, i=0;

	if (down_interruptible(&lmk2531_st.sem))
		return - ERESTARTSYS;

	switch (cmd) {
		case CMD_TRX_LO_SET:
		case CMD_TRX_LO_GET_LD:
			base_addr = TRX_LO_BASE;
			break;
		case CMD_DET_LO_SET:
		case CMD_DET_LO_GET_LD:
			base_addr = DET_LO_BASE;
			break;
		default:
			up(&lmk2531_st.sem);
			return -ENOTTY;

	}

	if (_IOC_DIR(cmd) == _IOC_READ) {
		ret = LMK2531_LD & __raw_readb(base_addr);
		ret = ret ? 1 : 0;
	} else if (_IOC_DIR(cmd) == _IOC_WRITE) {
		//clear all pins and then active LE
		lmk2531_iowrite(base_addr, LMK2531_ALL, 0);

		//24 bits data
		for(i=0; i<24; i++){
			lmk2531_iowrite(base_addr, LMK2531_DAT, (arg & 0x80000000));

			lmk2531_iowrite(base_addr, LMK2531_CLK, 1);
			lmk2531_iowrite(base_addr, LMK2531_CLK, 0);
			arg <<= 1;
		}
		//clear all pins and then active LE
		lmk2531_iowrite(base_addr, LMK2531_LE, 0);
		lmk2531_iowrite(base_addr, LMK2531_LE, 1);
		lmk2531_iowrite(base_addr, LMK2531_ALL, 0);
	} else {
		up(&lmk2531_st.sem);
		return -ENOTTY;
	}

	up(&lmk2531_st.sem);
	return ret;
}

static const struct file_operations lmk2531_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = NULL,
	.write  = NULL,
	.ioctl  = lmk2531_ioctl,
};

static int __init lmk2531_init(void)
{
	int ret = 0;

	// initial structure
	memset(&lmk2531_st, 0, sizeof(lmk2531_st));

	init_MUTEX(&lmk2531_st.sem);

	lmk2531_st.dev.minor = MISC_DYNAMIC_MINOR;
	lmk2531_st.dev.name = "g410sd_lmk2531";
	lmk2531_st.dev.fops = &lmk2531_fops;

	// register device
	ret = misc_register(&lmk2531_st.dev);

	if (ret)
		printk("BSP: %s fail to regite device\n", __FUNCTION__);
	else
		printk("BSP: LMK2531 Driver installed\n");

	return ret;
}

static void __exit lmk2531_exit(void)
{
	misc_deregister(&lmk2531_st.dev);
	printk("BSP: LMK2531 Driver removed\n");
}

module_init(lmk2531_init);
module_exit(lmk2531_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("LMK2531");
MODULE_LICENSE("GPL");

