/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : g420sd
 ::  ::  ::       ::           :::      FileName   : rtc.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-12-06 12:51:07
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	2010-07-07	Change cdev to miscdevices
	2010-12-06	修改ADC读流程
*/
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>

#include <linux/time.h>
#include <mach/platform.h>
#include <linux/delay.h>
#include <g420sd/g420sd_hw.h>

struct{
	struct miscdevice dev;
	struct semaphore sem;
}adc_28v_st;


/*
 * NOTE:
 * 	ADC读流程（参考LPC3250数据手册）：启动转换=>等待终端=>读转换值
 */
static ssize_t adc_28v_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	unsigned int value;

	if (down_interruptible(&adc_28v_st.sem))
		return - ERESTARTSYS;

	//__raw_writel(0x00000284, ADC_SELECT);  //ADC通道及参考电压选择
	__raw_writel(0x00000006, ADC_CTRL);	//启动转换
	udelay(1000);
	value = __raw_readl(ADC_VALUE) & 0x000003ff;	//读取转换值（硬件会自动清中断）

	if (copy_to_user(buf, &value, sizeof(unsigned int))){
		printk("BSP: %s fail copy_to_user\n", __FUNCTION__);
		up(&adc_28v_st.sem);
		return - EFAULT;
	}

	up(&adc_28v_st.sem);
	return sizeof(unsigned int);
}


static const struct file_operations adc_28v_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = adc_28v_read,
	.write  = NULL,
};

static int __init adc_28v_init(void)
{
	int ret = 0;

	// Initial structure
	memset(&adc_28v_st, 0, sizeof(adc_28v_st));

	init_MUTEX(&adc_28v_st.sem);

	adc_28v_st.dev.minor = MISC_DYNAMIC_MINOR;
	adc_28v_st.dev.name = "nts_adc28v";
	adc_28v_st.dev.fops = &adc_28v_fops;

	// register device
	ret = misc_register(&adc_28v_st.dev);
	if (ret) {
		printk("BSP: %s fail to register device\n", __FUNCTION__);
	} else {
		__raw_writel(0x0, ADC_CTRL);    //上电并启动转换
		__raw_writel(0x01, ADCLK_CTRL);	  //enable clk
		__raw_writel(0x180, ADCLK_CTRL1);  //select frequency
		__raw_writel(0x284, ADC_SELECT);  //ADC通道及参考电压选择
		__raw_writel(0x4, ADC_CTRL);    //上电并启动转换
		printk("BSP: nts adc28v Driver installed\n");
	}

	return ret;
}

static void __exit adc_28v_exit(void)
{
	misc_deregister(&adc_28v_st.dev);
	printk("BSP: nts adc28v Driver removed\n");
}

module_init(adc_28v_init);
module_exit(adc_28v_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("nts adc28v");
MODULE_LICENSE("GPL");
