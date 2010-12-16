/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      FileName   : fpga_ram.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-12-16 13:46:37
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	It's a driver to read/write fpga inner ram, such as dpd/cfr module.
*/

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include <mach/irqs.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include <x223ft/x223ft_hw.h>
#include <x223ft/fpga_ram.h>

struct{
	struct miscdevice dev;
	struct semaphore sem;
	unsigned short *kbuf;
}fpga_ram_st;

static ssize_t fpga_ram_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	unsigned int i;
	int reg_addrh, reg_addrl, reg_datal, reg_datah;
	struct fpga_ram_elem elem;

	if(down_interruptible(&fpga_ram_st.sem))
		return - ERESTARTSYS;

	if(copy_from_user(&elem, buf, sizeof(elem))) {
		printk("FPGA_RAM: fail to copy elem from userspace\n");
		up(&fpga_ram_st.sem);
		return - EFAULT;
	}

	if(copy_from_user(fpga_ram_st.kbuf, elem.buf, elem.group_len*8)){
		printk("FPGA_RAM: fail to copy data from userspace\n");
		up(&fpga_ram_st.sem);
		return - EFAULT;
	}

	reg_addrh = IF_FPGA_BASE + (elem.reg_addrh)*2;
	reg_addrl = IF_FPGA_BASE + (elem.reg_addrl)*2;
	reg_datah = IF_FPGA_BASE + (elem.reg_datah)*2;
	reg_datal = IF_FPGA_BASE + (elem.reg_datal)*2;

	if (elem.reg_addrh != FPGA_RAM_INVALID_ADDR) {	//32bit address
		if (elem.reg_datah != FPGA_RAM_INVALID_ADDR) {	//32bit data
			for (i=0; i<elem.group_len; i++) {
				__raw_writew(fpga_ram_st.kbuf[4*i+0], reg_addrl);
				__raw_writew(fpga_ram_st.kbuf[4*i+1], reg_addrh);
				__raw_writew(fpga_ram_st.kbuf[4*i+2], reg_datal);
				__raw_writew(fpga_ram_st.kbuf[4*i+3], reg_datah);
			}
		} else {	//16bit data
			for (i=0; i<elem.group_len; i++) {
				__raw_writew(fpga_ram_st.kbuf[4*i+0], reg_addrl);
				__raw_writew(fpga_ram_st.kbuf[4*i+1], reg_addrh);
				__raw_writew(fpga_ram_st.kbuf[4*i+2], reg_datal);
			}
		}
	} else {	//16bit address
		if (elem.reg_datah != FPGA_RAM_INVALID_ADDR) {	//32bit data
			for (i=0; i<elem.group_len; i++) {
				__raw_writew(fpga_ram_st.kbuf[4*i+0], reg_addrl);
				__raw_writew(fpga_ram_st.kbuf[4*i+2], reg_datal);
				__raw_writew(fpga_ram_st.kbuf[4*i+3], reg_datah);
			}
		} else {	//16bit data
			for (i=0; i<elem.group_len; i++) {
				__raw_writew(fpga_ram_st.kbuf[4*i+0], reg_addrl);
				__raw_writew(fpga_ram_st.kbuf[4*i+2], reg_datal);
			}
		}
	}


	up(&fpga_ram_st.sem);

	return (elem.group_len*8);
}

static ssize_t fpga_ram_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	unsigned int i;
	int reg_addrh, reg_addrl, reg_datal, reg_datah;
	struct fpga_ram_elem elem;

	if (down_interruptible(&fpga_ram_st.sem))
		return - ERESTARTSYS;

	// copy elem
	if(copy_from_user(&elem, buf, sizeof(elem))){
		printk("FPGA_RAM: fail to copy elem from userspace\n");
		up(&fpga_ram_st.sem);
		return - EFAULT;
	}

	if(copy_from_user(fpga_ram_st.kbuf, elem.buf, elem.group_len*8)){
		printk("FPGA_RAM: fail to copy data from userspace\n");
		up(&fpga_ram_st.sem);
		return - EFAULT;
	}

	reg_addrh = IF_FPGA_BASE + (elem.reg_addrh)*2;
	reg_addrl = IF_FPGA_BASE + (elem.reg_addrl)*2;
	reg_datah = IF_FPGA_BASE + (elem.reg_datah)*2;
	reg_datal = IF_FPGA_BASE + (elem.reg_datal)*2;

	if (elem.reg_addrh != FPGA_RAM_INVALID_ADDR) {	//32bit address
		if (elem.reg_datah != FPGA_RAM_INVALID_ADDR) {	//32bit data
			for (i=0; i<elem.group_len; i++) {
				__raw_writew(fpga_ram_st.kbuf[4*i+0], reg_addrl);
				__raw_writew(fpga_ram_st.kbuf[4*i+1], reg_addrh);
				fpga_ram_st.kbuf[4*i+2] = __raw_readw(reg_datal);
				fpga_ram_st.kbuf[4*i+3] = __raw_readw(reg_datah);
			}
		} else {	//16bit data
			for (i=0; i<elem.group_len; i++) {
				__raw_writew(fpga_ram_st.kbuf[4*i+0], reg_addrl);
				__raw_writew(fpga_ram_st.kbuf[4*i+1], reg_addrh);
				fpga_ram_st.kbuf[4*i+2] = __raw_readw(reg_datal);
			}
		}
	} else {	//16bit address
		if (elem.reg_datah != FPGA_RAM_INVALID_ADDR) {	//32bit data
			for (i=0; i<elem.group_len; i++) {
				__raw_writew(fpga_ram_st.kbuf[4*i+0], reg_addrl);
				fpga_ram_st.kbuf[4*i+2] = __raw_readw(reg_datal);
				fpga_ram_st.kbuf[4*i+3] = __raw_readw(reg_datah);
			}
		} else {	//16bit data
			for (i=0; i<elem.group_len; i++) {
				__raw_writew(fpga_ram_st.kbuf[4*i+0], reg_addrl);
				fpga_ram_st.kbuf[4*i+2] = __raw_readw(reg_datal);
			}
		}
	}


	// copy data back to user-space
	if (copy_to_user(elem.buf, fpga_ram_st.kbuf, elem.group_len*8)){
		printk("FPGA_RAM: fail to copy data to user-space fail\n");
		up(&fpga_ram_st.sem);
		return - EFAULT;
	}

	up(&fpga_ram_st.sem);

	return (elem.group_len*8);
}

static const struct file_operations fpga_ram_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.ioctl  = NULL,
	.read   = fpga_ram_read,
	.write  = fpga_ram_write,
};

static int __init fpga_ram_init(void)
{
	int ret = 0;
	void *kbuf;

	// malloc for data buffer
	kbuf = kzalloc(MAX_GROUP_LEN*8, GFP_KERNEL);
	if (!kbuf) {
		ret = - ENOMEM;
		goto fail_malloc;
	}

	// initial structure
	memset(&fpga_ram_st, 0, sizeof(fpga_ram_st));

	fpga_ram_st.kbuf = kbuf;
	init_MUTEX(&fpga_ram_st.sem);

	fpga_ram_st.dev.minor = MISC_DYNAMIC_MINOR;
	fpga_ram_st.dev.name = "nts_fpga_ram";
	fpga_ram_st.dev.fops = &fpga_ram_fops;

	// registe device
	ret = misc_register(&fpga_ram_st.dev);
	if (ret) {
		printk("FPGA_RAM: %s fail to registe device\n", __FUNCTION__);
		goto fail_register;
	}


	printk("FPGA_RAM: Driver installed!\n");

	return 0;

fail_register:
	kfree(kbuf);

fail_malloc:
	printk("FPGA_RAM: Fail to install driver\n");

	return ret;
}

static void __exit fpga_ram_exit(void)
{
	misc_deregister(&fpga_ram_st.dev);
	kfree(fpga_ram_st.kbuf);
	printk("FPGA_RAM: Driver removed\n");
}

module_init(fpga_ram_init);
module_exit(fpga_ram_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("FPGA_RAM Module in IF FPGA");
MODULE_LICENSE("GPL");
