/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Athurg.Feng
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      FileName  : lmk03000.c
 ::   :: ::       ::             ::     Generate   : 2009.05.31
 ::    ::::       ::       ::      ::   Update     : 2010-07-28 15:18:42
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	2010-07-07	Change cdev to miscdevices
	None
*/
#include <linux/fs.h>
#include <linux/miscdevice.h>

#include <asm/io.h>
#include <linux/semaphore.h>

#include <g200wo/g200wo_hw.h>
#include <g200wo/lmk03000.h>

struct{
	struct miscdevice dev;
	struct semaphore sem;
	char data;
}lmk03000_st;

void lmk03000_io_write(unsigned int port, unsigned int active)
{
	lmk03000_st.data &= ~port;
	if(active)	lmk03000_st.data |= port;

	__raw_writeb(lmk03000_st.data, LMK03000_BASE);
}

static int lmk03000_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0, i;

	if (down_interruptible(&lmk03000_st.sem))
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

	// Initial Structure
	memset(&lmk03000_st, 0, sizeof(lmk03000_st));

	init_MUTEX(&lmk03000_st.sem);

	lmk03000_st.dev.minor = MISC_DYNAMIC_MINOR;
	lmk03000_st.dev.name = "g200wo_lmk03000";
	lmk03000_st.dev.fops = &lmk03000_fops;

	// Registe device
	ret = misc_register(&lmk03000_st.dev);

	if (ret)
		printk("BSP: %s fail to registe device\n", __FUNCTION__);
	else
		printk("BSP: G200WO LMK03000 Driver installed\n");

	return ret;
}

static void __exit lmk03000_exit(void)
{
	misc_deregister(&lmk03000_st.dev);
	printk("BSP: G200WO LMK03000 Driver removed\n");
}

module_init(lmk03000_init);
module_exit(lmk03000_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO LMK03000");
MODULE_LICENSE("GPL");
