/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : if_fpga.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-07-01 12:04:00
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	FIXME:
		maybe we should use little-endian. check line 161,168,231,238.
*/

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/semaphore.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include <mach/irqs.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include "g200wo_hw.h"
#include "if_fpga.h"

#define OFFSET_CFR_ADDR		0x0
#define OFFSET_CFR_DATAL	0x1
#define OFFSET_CFR_DATAH	0x2
#define OFFSET_DPD_ADDR		0x0
#define OFFSET_DPD_DATAL_WR	0x1
#define OFFSET_DPD_DATAH_WR	0x2
#define OFFSET_DPD_DATAL_RD	0x3
#define OFFSET_DPD_DATAH_RD	0x4

struct if_fpga_st
{
	struct cdev cdev;
	struct semaphore sem;
	unsigned short *kbuf;
	pid_t pid;
	int irq_agc;
};
struct if_fpga_st *if_fpga_stp;

static int if_send_sig(int signo)
{
	siginfo_t info;
	struct task_struct *p;

	info.si_signo = signo;
	if (if_fpga_stp->pid<=0)
		goto find_none;
	
	read_lock(&tasklist_lock);
	for_each_process(p){
		if (p->pid == if_fpga_stp->pid){
			read_unlock(&tasklist_lock);
			goto find_ps;
		}
	}
	read_unlock(&tasklist_lock);

find_none:
	printk("BSP IF: No process to send.\n");
	return -1;

find_ps:
	send_sig_info(signo, &info, p);
	return 0;
}

irqreturn_t if_agc_irq(int irq, void *context_data)
{
	if_send_sig(SIG_IF_AGC); // send signal
	return IRQ_HANDLED;
}


static int if_fpga_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned short rtn=0;

	if (down_interruptible(&if_fpga_stp->sem))
		return - ERESTARTSYS;
	
	switch(cmd){
		case CMD_IF_SET_PID:
			if_fpga_stp->pid = (pid_t)arg;
			break;

		case CMD_IF_FPGA_READ_WORD:
			rtn = __raw_readw(IF_FPGA_BASE + (arg & 0xFFFF));
			break;

		case CMD_IF_FPGA_WRITE_WORD:
			__raw_writew(((arg>>16) & 0xFFFF), (IF_FPGA_BASE + (arg & 0xFFFF)));
			break;

		default:
			rtn = -ENOTTY;
	}
	
	up(&if_fpga_stp->sem);

	return rtn;
}

static ssize_t if_fpga_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	unsigned int i,len;
	struct if_fpga_elem elem;

	if(down_interruptible(&if_fpga_stp->sem))
		return - ERESTARTSYS;

	// Get elem
	if(copy_from_user(&elem, buf, sizeof(struct if_fpga_elem))){
		printk("BSP: %s copy_from_user elem\n", __FUNCTION__);
		up(&if_fpga_stp->sem);
		return - EFAULT;
	}

	if((elem.type != TYPE_IF_FPGA_FIFO) && (elem.type != TYPE_IF_FPGA_FIFO))
		len = elem.wlen * 8;
	else	len = elem.wlen * 2;

	// Check if address is valid when NORMAL mode
	if((((elem.addr + len) > 0xFFFF) && (elem.type == TYPE_IF_FPGA_NORMAL))
		|| (len > MAX_IF_FPGA_LEN)){
		return -EFAULT;
	}

	//Get data and address
	if(copy_from_user(if_fpga_stp->kbuf, elem.buf, len)){
		printk("BSP: %s copy_from_user data\n", __FUNCTION__);
		up(&if_fpga_stp->sem);
		return - EFAULT;
	}

	// write data
	switch(elem.type){
		case TYPE_IF_FPGA_FIFO:
			for(i=0; i<elem.wlen; i++){
				__raw_writew(if_fpga_stp->kbuf[i], (elem.addr + IF_FPGA_BASE));
			}
			break;
		case TYPE_IF_FPGA_NORMAL:
			for(i=0; i<elem.wlen; i++){
				__raw_writew(if_fpga_stp->kbuf[i], (elem.addr + IF_FPGA_BASE + i));
			}
			break;
		case TYPE_IF_FPGA_CFRA:
		case TYPE_IF_FPGA_CFRB:
			for(i=0; i<elem.wlen; i++){
				__raw_writew(if_fpga_stp->kbuf[i+0], (elem.addr + IF_FPGA_BASE + OFFSET_CFR_ADDR));
				__raw_writew(if_fpga_stp->kbuf[i+2], (elem.addr + IF_FPGA_BASE + OFFSET_CFR_DATAL));
				__raw_writew(if_fpga_stp->kbuf[i+3], (elem.addr + IF_FPGA_BASE + OFFSET_CFR_DATAH));
			}
			break;
		case TYPE_IF_FPGA_DPD:
			for(i=0; i<elem.wlen; i++){
				__raw_writew(if_fpga_stp->kbuf[i+0], (elem.addr + IF_FPGA_BASE + OFFSET_DPD_ADDR));
				__raw_writew(if_fpga_stp->kbuf[i+2], (elem.addr + IF_FPGA_BASE + OFFSET_DPD_DATAL_WR));
				__raw_writew(if_fpga_stp->kbuf[i+3], (elem.addr + IF_FPGA_BASE + OFFSET_DPD_DATAH_WR));
			}
			break;
		default:
			return -EFAULT;
	}

	up(&if_fpga_stp->sem);

	return len;
}

static ssize_t if_fpga_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	int i, len;	//len means size of data in BYTES
	struct if_fpga_elem elem;

	if (down_interruptible(&if_fpga_stp->sem))
		return - ERESTARTSYS;
	
	// copy elem
	if(copy_from_user(&elem, buf, sizeof(struct if_fpga_elem))){
		printk("BSP: %s copy_from_user elem\n", __FUNCTION__);
		up(&if_fpga_stp->sem);
		return - EFAULT;
	}

	if((elem.type != TYPE_IF_FPGA_FIFO) && (elem.type != TYPE_IF_FPGA_NORMAL))
		len = elem.wlen * 8;
	else	len = elem.wlen * 2;

	// Check if address is valid when NORMAL mode
	if((((elem.addr + len) > 0xFFFF) && (elem.type == TYPE_IF_FPGA_NORMAL))
		|| (len > MAX_IF_FPGA_LEN)){
		return -EFAULT;
	}

	//Get each address while DPD and CFR mode
	if((elem.type != TYPE_IF_FPGA_FIFO) && (elem.type != TYPE_IF_FPGA_NORMAL)){
		if(copy_from_user(if_fpga_stp->kbuf, elem.buf, len)){
			printk("BSP: %s copy_from_user data\n", __FUNCTION__);
			up(&if_fpga_stp->sem);
			return - EFAULT;
		}
	}

	// read data
	switch(elem.type){
		case TYPE_IF_FPGA_FIFO:
			for (i=0; i<elem.wlen; i++){
				if_fpga_stp->kbuf[i] = __raw_readw(elem.addr + IF_FPGA_BASE);
			}
			break;
		case TYPE_IF_FPGA_NORMAL:
			for (i=0; i<elem.wlen; i++){
				if_fpga_stp->kbuf[i] = __raw_readw(elem.addr + IF_FPGA_BASE + i);
			}
			break;
		case TYPE_IF_FPGA_CFRA:
		case TYPE_IF_FPGA_CFRB:
			for (i=0; i<elem.wlen; i++){
				__raw_writew(if_fpga_stp->kbuf[i], (elem.addr + IF_FPGA_BASE + OFFSET_CFR_ADDR));
				if_fpga_stp->kbuf[i+2] = __raw_readw(elem.addr + IF_FPGA_BASE + OFFSET_CFR_DATAL);
				if_fpga_stp->kbuf[i+3] = __raw_readw(elem.addr + IF_FPGA_BASE + OFFSET_CFR_DATAH);
			}
			break;
		case TYPE_IF_FPGA_DPD:
			for (i=0; i<elem.wlen; i++){
				__raw_writew(if_fpga_stp->kbuf[i], (elem.addr + IF_FPGA_BASE + OFFSET_DPD_ADDR));
				if_fpga_stp->kbuf[i+2] = __raw_readw(elem.addr + IF_FPGA_BASE + OFFSET_DPD_DATAL_RD);
				if_fpga_stp->kbuf[i+3] = __raw_readw(elem.addr + IF_FPGA_BASE + OFFSET_DPD_DATAH_RD);
			}
			break;
		default:
			up(&if_fpga_stp->sem);
			return -EFAULT;
	}

	// copy data
	if (copy_to_user(elem.buf, if_fpga_stp->kbuf, len)){
		printk("BSP: %s copy_to_user data\n", __FUNCTION__);
		up(&if_fpga_stp->sem);
		return - EFAULT;
	}

	up(&if_fpga_stp->sem);

	return len;
}

static const struct file_operations if_fpga_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.ioctl  = if_fpga_ioctl,
	.read   = if_fpga_read,
	.write  = if_fpga_write,
};

static int __init if_fpga_init(void)
{
	int ret = 0;
	void *kbuf;
	dev_t devno;

	// register chrdev
	devno = MKDEV(MAJ_IF_FPGA, MIN_IF_FPGA);
	ret = register_chrdev_region(devno, 1, "g200_if_fpga");
	if (ret<0){
		printk("BSP: %s register_chrdev_region\n", __FUNCTION__);
		return ret;
	}

	// alloc dev
	if_fpga_stp = kmalloc(sizeof(struct if_fpga_st), GFP_KERNEL);
	if (!if_fpga_stp){
		ret = - ENOMEM;
		goto fail_malloc;
	}
	memset(if_fpga_stp, 0, sizeof(struct if_fpga_st));

	// alloc kbuf
	kbuf = kzalloc(MAX_IF_FPGA_LEN, GFP_KERNEL);
	if (!kbuf){
		ret = - ENOMEM;
		goto fail_malloc2;
	}

	if_fpga_stp->kbuf = kbuf;
	init_MUTEX(&if_fpga_stp->sem);

	cdev_init(&if_fpga_stp->cdev, &if_fpga_fops);
	if_fpga_stp->cdev.owner = THIS_MODULE;
	if_fpga_stp->cdev.ops = &if_fpga_fops;

	// add cdev
	ret = cdev_add(&if_fpga_stp->cdev, devno, 1);
	if (ret){
		printk("BSP: %s cdev_add\n", __FUNCTION__);
		goto fail_add;
	}

	// request_irq
	if_fpga_stp->irq_agc = IF_AGC_IRQ;
	set_irq_type(if_fpga_stp->irq_agc, IRQ_TYPE_EDGE_FALLING);
	ret = request_irq(if_fpga_stp->irq_agc, if_agc_irq, IRQF_DISABLED, "if_fpga_agc", if_fpga_stp);
	if(ret != 0){
		printk("BSP: %s fail request_irq\n", __FUNCTION__);
		goto fail_reqirq;
	}
	printk("G200WO IF_FPGA Driver installed\n");
	return 0;

fail_reqirq:
	cdev_del(&if_fpga_stp->cdev);

fail_add:
	kfree(kbuf);

fail_malloc2:
	kfree(if_fpga_stp);

fail_malloc:
	unregister_chrdev_region(devno, 1);
	printk("Fail to install G200WO IF_FPGA driver\n");
	return ret;
}

static void __exit if_fpga_exit(void)
{
	dev_t devno;
	
	cdev_del(&if_fpga_stp->cdev);
	kfree(if_fpga_stp->kbuf);
	kfree(if_fpga_stp);

	devno = MKDEV(MAJ_IF_FPGA, MIN_IF_FPGA);
	unregister_chrdev_region(devno, 1);

	printk("G200WO IF_FPGA Driver removed\n");
}

module_init(if_fpga_init);
module_exit(if_fpga_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO IF_FPGA");
MODULE_LICENSE("GPL");
