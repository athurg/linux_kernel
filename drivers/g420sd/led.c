/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Andy.wu
 :: ::   ::       ::         ::         Project    : g420sd
 ::  ::  ::       ::           :::      File Name  : led.c
 ::   :: ::       ::             ::     Generate   : 2009.05.31
 ::    ::::       ::       ::      ::   Update     : 2010-07-26 18:56:57
::::    :::     ::::::      ::::::::    Version    : v0.3

Description
	2010-07-07	Change cdev to miscdevices
	None
*/
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>

#include <asm/io.h>
#include <mach/lpc32xx_gpio.h>

#include <g420sd/g420sd_hw.h>
#include <g420sd/led.h>

struct led_st
{
	struct miscdevice dev;
	struct semaphore sem;
} *led_stp;

#define  LED_RUN_OK             _BIT(1)   
#define  LED_RUN_ERR            _BIT(4) 
#define  LED_DPD_OK             _BIT(5)  
#define  LED_DPD_ERR            _BIT(11)  
#define  LED_VSWR_OK	    	_BIT(14) 
#define  LED_VSWR_ERR	    	_BIT(17) 

static int led_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	unsigned long led_in_dat;
	

	if (down_interruptible(&led_stp->sem))
		return - ERESTARTSYS;
		
	led_in_dat = __raw_readl(GPIO_P3_OUTP_STATE(GPIO_IOBASE));
	
	if(cmd == CMD_LED_SET)
	{
		switch (arg){
		case AGE_LED_RUN_OK:
			__raw_writel(led_in_dat |  LED_RUN_OK, GPIO_P3_OUTP_SET(GPIO_IOBASE));		
			__raw_writel(!led_in_dat |  LED_RUN_ERR, GPIO_P3_OUTP_CLR(GPIO_IOBASE));
			break;
		case AGE_LED_RUN_ERR:
			__raw_writel(led_in_dat |  LED_RUN_ERR, GPIO_P3_OUTP_SET(GPIO_IOBASE));		
			__raw_writel(!led_in_dat |  LED_RUN_OK, GPIO_P3_OUTP_CLR(GPIO_IOBASE));
			break;			
		case AGE_LED_DPD_OK:
			__raw_writel(led_in_dat |  LED_DPD_OK, GPIO_P3_OUTP_SET(GPIO_IOBASE));		
			__raw_writel(!led_in_dat |  LED_DPD_ERR, GPIO_P3_OUTP_CLR(GPIO_IOBASE));
			break;
		case AGE_LED_DPD_ERR:
			__raw_writel(led_in_dat |  LED_DPD_ERR, GPIO_P3_OUTP_SET(GPIO_IOBASE));		
			__raw_writel(!led_in_dat |  LED_DPD_OK, GPIO_P3_OUTP_CLR(GPIO_IOBASE));
			break;
		case AGE_LED_DPD_OFF:
			__raw_writel(led_in_dat |  LED_DPD_ERR, GPIO_P3_OUTP_SET(GPIO_IOBASE));
			__raw_writel(led_in_dat |  LED_DPD_OK, GPIO_P3_OUTP_SET(GPIO_IOBASE));
			break;
		case AGE_LED_VSWR_OK:
			__raw_writel(led_in_dat |  LED_VSWR_OK, GPIO_P3_OUTP_SET(GPIO_IOBASE));		
			__raw_writel(!led_in_dat | LED_VSWR_ERR, GPIO_P3_OUTP_CLR(GPIO_IOBASE));
			break;
		case AGE_LED_VSWR_ERR:
			__raw_writel(led_in_dat |  LED_VSWR_ERR, GPIO_P3_OUTP_SET(GPIO_IOBASE));		
			__raw_writel(!led_in_dat | LED_VSWR_OK, GPIO_P3_OUTP_CLR(GPIO_IOBASE));
			break;						
		default:
			ret = -ENOTTY;
			break;
			}	
	}
	else
		ret = -ENOTTY;
	
	up(&led_stp->sem);
	return ret;
}

static const struct file_operations led_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = NULL,
	.write  = NULL,
	.ioctl  = led_ioctl,
};

static int __init led_init(void)
{
	int ret = 0;

	// malloc and intial
	led_stp = kmalloc(sizeof(struct led_st), GFP_KERNEL);
	if (!led_stp){
		ret = - ENOMEM;
		goto fail_malloc;
	}

	memset(led_stp, 0, sizeof(struct led_st));
	init_MUTEX(&led_stp->sem);
	led_stp->dev.minor = MISC_DYNAMIC_MINOR;
	led_stp->dev.name = "nts_led";
	led_stp->dev.fops = &led_fops;

	// register device
	ret = misc_register(&led_stp->dev);
	if (ret){
		printk("BSP: %s fail to register device\n", __FUNCTION__);
		goto fail_registe;
	}

	printk("g420sd LED Driver installed\n");
	return 0;

fail_registe:
	kfree(led_stp);

fail_malloc:
	printk("Fail to install g420sd LED driver\n");
	return ret;
}

static void __exit led_exit(void)
{
	misc_deregister(&led_stp->dev);
	kfree(led_stp);
	printk("g420sd LED Driver removed\n");
}

module_init(led_init);
module_exit(led_exit);

MODULE_AUTHOR("Andy.wu, <andy.wu@nts-intl.com>");
MODULE_DESCRIPTION("g420sd LED");
MODULE_LICENSE("GPL");
