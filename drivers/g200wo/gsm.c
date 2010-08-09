/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      FileName   : gsm.c
 ::   :: ::       ::             ::     Generate   : 2009.05.31
 ::    ::::       ::       ::      ::   Update     : 2010-08-09 11:47:25
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	2010-07-07	Change cdev to miscdevices
	None
*/
#include <linux/fs.h>

#include <asm/io.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>

#include <g200wo/g200wo_hw.h>
#include <g200wo/gsm.h>

// define the port as bus
#define GSM_PWR_BUS	(GSM_PWRON_N | GSM_VCHARGE)
#define GSM_ATT_BUS	(GSM_ATT0 | GSM_ATT1 | GSM_ATT2 | GSM_ATT3 | GSM_ATT4)

struct{
	struct miscdevice dev;
	struct semaphore sem;
}gsm_st;

static int gsm_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	if (down_interruptible(&gsm_st.sem))
		return - ERESTARTSYS;

	if(cmd == CMD_GSM_PWR){//Manage Power
		if(arg == ARG_GSM_PWR_ON){
			__raw_writel(GSM_PWRON_N, GPIO_P3_OUTP_CLR(GPIO_IOBASE));
		}else if(arg == ARG_GSM_PWR_OFF){
			__raw_writel(GSM_PWRON_N, GPIO_P3_OUTP_SET(GPIO_IOBASE));
		}else{
			ret = -ENOTTY;
		}
	}else if(cmd == CMD_GSM_VCHARGE) {
		if(arg == ARG_GSM_VCHARGE_ON){
			__raw_writel(GSM_VCHARGE, GPIO_P3_OUTP_CLR(GPIO_IOBASE));
		}else if(arg == ARG_GSM_VCHARGE_OFF){
			__raw_writel(GSM_VCHARGE, GPIO_P3_OUTP_SET(GPIO_IOBASE));
		}else{
			ret = -ENOTTY;
		}

	}else if(cmd == CMD_GSM_ATT){	//manage ATT
		arg = ((~arg) & 0x1F);	//We need only 5 bits from LSB

		//First clean, then load new value
		__raw_writel(GSM_ATT_BUS, GPIO_P3_OUTP_CLR(GPIO_IOBASE));
		__raw_writel(arg<<6, GPIO_P3_OUTP_SET(GPIO_IOBASE));
	}else{
		ret = -ENOTTY;
	}

	up(&gsm_st.sem);
	return ret;
}

static const struct file_operations gsm_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = NULL,
	.write  = NULL,
	.ioctl  = gsm_ioctl,
};

static int __init gsm_init(void)
{
	int ret = 0;

	// malloc and initial
	memset(&gsm_st, 0, sizeof(gsm_st));

	init_MUTEX(&gsm_st.sem);

	gsm_st.dev.minor = MISC_DYNAMIC_MINOR;
	gsm_st.dev.name = "g200wo_gsm_config";
	gsm_st.dev.fops = &gsm_fops;

	// register device
	ret = misc_register(&gsm_st.dev);

	if (ret) {
		printk("BSP: %s fail register device\n", __FUNCTION__);
	} else {
		//set port as GPIO
		__raw_writel(GSM_ATT_BUS | GSM_PWR_BUS, GPIO_P3_MUX_CLR(GPIO_IOBASE));
		// default value
		__raw_writel(GSM_ATT_BUS | GSM_PWR_BUS, GPIO_P3_OUTP_SET(GPIO_IOBASE));
		printk("BSP: G200WO GSM Driver installed\n");
	}

	return ret;
}

static void __exit gsm_exit(void)
{
	misc_deregister(&gsm_st.dev);
	printk("BSP: G200WO GSM Driver removed\n");
}

module_init(gsm_init);
module_exit(gsm_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO GSM control");
MODULE_LICENSE("GPL");
