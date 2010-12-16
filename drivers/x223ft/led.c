/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Andy.wu
 :: ::   ::       ::         ::         Project    : x223ft
 ::  ::  ::       ::           :::      File Name  : led.c
 ::   :: ::       ::             ::     Generate   : 2009.05.31
 ::    ::::       ::       ::      ::   Update     : 2010-11-10 10:52:28
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

#include <x223ft/led.h>

struct{
	struct miscdevice dev;
	struct semaphore sem;
}led_st;

static int led_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	if (down_interruptible(&led_st.sem))
		return - ERESTARTSYS;

	if(cmd == LED_IOC_ON)
		__raw_writel(arg, GPIO_P3_OUTP_SET(GPIO_IOBASE));
	else if(cmd == LED_IOC_OFF)
		__raw_writel(arg, GPIO_P3_OUTP_CLR(GPIO_IOBASE));
	else
		ret = -ENOTTY;

	up(&led_st.sem);
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

	// Intial structure
	memset(&led_st, 0, sizeof(led_st));

	init_MUTEX(&led_st.sem);

	led_st.dev.minor = MISC_DYNAMIC_MINOR;
	led_st.dev.name = "nts_led";
	led_st.dev.fops = &led_fops;

	// register device
	ret = misc_register(&led_st.dev);
	if (ret){
		printk("BSP: %s fail to register device\n", __FUNCTION__);
	}else{
		printk("BSP: LED Driver installed\n");
	}

	return ret;
}

static void __exit led_exit(void)
{
	misc_deregister(&led_st.dev);
	printk("BSP: LED Driver removed\n");
}

module_init(led_init);
module_exit(led_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("Status LEDS");
MODULE_LICENSE("GPL");
