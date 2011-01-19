/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Athurg.Feng
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : g420sd
 ::  ::  ::       ::           :::      FileName   : lmk04031.c
 ::   :: ::       ::             ::     Generate   : 2009.05.31
 ::    ::::       ::       ::      ::   Update     : 2010-08-04 16:39:59
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	2010-07-07	Change cdev to miscdevices
	None
*/
#include <linux/fs.h>
#include <linux/miscdevice.h>

#include <asm/io.h>
#include <linux/semaphore.h>

#include <g420sd/g420sd_hw.h>
#include <g420sd/lmk04031.h>

struct{
	struct miscdevice dev;
	struct semaphore sem;
	char data;
}lmk04031_st;

void lmk04031_io_write(unsigned int port, unsigned int active)
{
	lmk04031_st.data &= ~port;
	if(active)	lmk04031_st.data |= port;

	__raw_writeb(lmk04031_st.data, LMK04031_BASE);
}

static int lmk04031_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0, i;

	if (down_interruptible(&lmk04031_st.sem))
		return - ERESTARTSYS;

	switch(cmd){
		case CMD_SET_LMK04031_DATA:
			//LE => LOW
			lmk04031_io_write((LMK04031_LE | LMK04031_CLK), 0);

			//32 bits data
			for(i=0; i<32; i++){
				lmk04031_io_write(LMK04031_DAT, (arg&0x80000000));
				lmk04031_io_write(LMK04031_CLK, 1);
				lmk04031_io_write(LMK04031_CLK, 0);
				arg <<= 1;
			}

			//LE => HIGH
			lmk04031_io_write(LMK04031_LE | LMK04031_DAT, 0);
			lmk04031_io_write(LMK04031_LE, 1);
			lmk04031_io_write(LMK04031_LE, 0);
			break;

		case CMD_SET_LMK04031_SYNC:
			lmk04031_io_write(LMK04031_SYNC, arg);
			break;

		case CMD_SET_LMK04031_GOE:
			lmk04031_io_write(LMK04031_GOE, arg);
			break;

		case CMD_GET_LMK04031_LD:
			ret = LMK04031_LD & __raw_readb(LMK04031_BASE);
			ret = ret ? 1 : 0;
			break;
		default:
			ret = -ENOTTY;

	}

	up(&lmk04031_st.sem);
	return ret;
}

static const struct file_operations lmk04031_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = NULL,
	.write  = NULL,
	.ioctl  = lmk04031_ioctl,
};

static int __init lmk04031_init(void)
{
	int ret = 0;

	// Initial Structure
	memset(&lmk04031_st, 0, sizeof(lmk04031_st));
	init_MUTEX(&lmk04031_st.sem);
	lmk04031_st.dev.minor = MISC_DYNAMIC_MINOR;
	lmk04031_st.dev.name = "nts_lmk04031";
	lmk04031_st.dev.fops = &lmk04031_fops;
	// Registe device
	ret = misc_register(&lmk04031_st.dev);
	if (ret)
		printk("BSP: %s fail to registe device\n", __FUNCTION__);
	else
		printk("BSP: g420sd lmk04031 driver installed\n");

	return ret;
}

static void __exit lmk04031_exit(void)
{
	misc_deregister(&lmk04031_st.dev);
	printk("BSP: g420sd lmk04031 driver removed\n");
}

module_init(lmk04031_init);
module_exit(lmk04031_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("g420sd lmk04031");
MODULE_LICENSE("GPL");
