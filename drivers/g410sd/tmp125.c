/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Athurg.Feng
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G410SD
 ::  ::  ::       ::           :::      FileName   : tmp125.c
 ::   :: ::       ::             ::     Generate   : 2009.05.31
 ::    ::::       ::       ::      ::   Update     : 2010-08-09 18:09:31
::::    :::     ::::::      ::::::::    Version    : v0.1

Description
	2010-07-07	Change cdev to miscdevices
	None
*/
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>

#include <g410sd/g410sd_hw.h>

struct{
	struct miscdevice dev;
	struct semaphore sem;
}tmp125_st;

void inline tmp125_io_write(int port, int active)
{
	if(active)
		__raw_writel(port, GPIO_P3_OUTP_SET(GPIO_IOBASE));
	else
		__raw_writel(port, GPIO_P3_OUTP_CLR(GPIO_IOBASE));
	// we should wait for data setup
	ndelay(50);
}

static ssize_t tmp125_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	int i,data=0;
	if (down_interruptible(&tmp125_st.sem))
		return - ERESTARTSYS;

	tmp125_io_write(TMP125_CS_N | TMP125_SCLK, 1);
	//Chip Select active
	tmp125_io_write(TMP125_CS_N, 0);

	//Lead zero bit
	tmp125_io_write(TMP125_SCLK, 0);
	tmp125_io_write(TMP125_SCLK, 1);

	//10 bits valid data
	for(i=0; i<10; i++){
		//data should be read after falledge
		tmp125_io_write(TMP125_SCLK, 0);
		data <<=1;
		if(TMP125_DOUT & __raw_readl(GPIO_P3_INP_STATE(GPIO_IOBASE)))
			data += 1;
		tmp125_io_write(TMP125_SCLK, 1);

	}

	//5 bits dummy
	for(i=0; i<5; i++){
		tmp125_io_write(TMP125_SCLK, 0);
		tmp125_io_write(TMP125_SCLK, 1);
	}

	//Chip Select inactive
	tmp125_io_write(TMP125_CS_N | TMP125_SCLK, 1);

	// mask data bits
	data &= 0x3FF;

	if (copy_to_user(buf, &data, sizeof(int))){
		printk("BSP: %s fail copy_to_user\n", __FUNCTION__);
		up(&tmp125_st.sem);
		return - EFAULT;
	}

	up(&tmp125_st.sem);

	return sizeof(int);
}

static const struct file_operations tmp125_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = tmp125_read,
	.write  = NULL,
	.ioctl  = NULL,
};

static int __init tmp125_init(void)
{
	int ret = 0;

	// Initial structure
	memset(&tmp125_st, 0, sizeof(tmp125_st));

	init_MUTEX(&tmp125_st.sem);

	tmp125_st.dev.minor = MISC_DYNAMIC_MINOR;
	tmp125_st.dev.name = "g410sd_tmp125";
	tmp125_st.dev.fops = &tmp125_fops;

	// registe device
	ret = misc_register(&tmp125_st.dev);
	if (ret) {
		printk("BSP: %s fail to registe device\n", __FUNCTION__);
	} else {
		// set port mux
		__raw_writel(TMP125_SCLK | TMP125_DOUT | TMP125_CS_N, GPIO_P3_MUX_CLR(GPIO_IOBASE));

		// output default value
		__raw_writel(TMP125_SCLK | TMP125_CS_N, GPIO_P3_OUTP_SET(GPIO_IOBASE));
		printk("BSP: G410SD TMP125 Driver installed\n");
	}

	return ret;
}

static void __exit tmp125_exit(void)
{
	misc_deregister(&tmp125_st.dev);
	printk("BSP: 200WO TMP125 Driver removed\n");
}

module_init(tmp125_init);
module_exit(tmp125_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G410SD TMP125 temperature driver");
MODULE_LICENSE("GPL");
