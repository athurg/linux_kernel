/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : power.c
 ::   :: ::       ::             ::     Generate   : 2009.06.01
 ::    ::::       ::       ::      ::   Update     : 2010.06.09
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	None
*/
//------------------------------------------------------------------------------
// include
//------------------------------------------------------------------------------
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/semaphore.h>

//------------------------------------------------------------------------------
// configuration
//------------------------------------------------------------------------------
#include "hw_cfg.h"
#include "power.h"

#define POWER_INT_EN        0x10
#define POWER_INT_ACT       0x01

//------------------------------------------------------------------------------
// structure
//------------------------------------------------------------------------------
struct power_st
{
	struct cdev cdev;
	struct semaphore sem;
	pid_t pid;
	volatile unsigned char __iomem *regp;
	int irq;
	int count;
};

//------------------------------------------------------------------------------
// global
//------------------------------------------------------------------------------
struct power_st *power_stp;

//------------------------------------------------------------------------------
// io functions
//------------------------------------------------------------------------------
static inline u8 power_in(unsigned int off)
{
	return __raw_readb(&power_stp->regp[off]);
}

static inline void power_out(unsigned int off, u8 val)
{
	__raw_writeb(val, &power_stp->regp[off]);
}

//------------------------------------------------------------------------------
// hardware functions
//------------------------------------------------------------------------------
static int power_send_sig(void)
{
	siginfo_t info;
	struct task_struct *p;
	info.si_signo = SIG_POWER;
	if (power_stp->pid<=0)
		goto find_none;
	read_lock(&tasklist_lock);
	for_each_process(p)
	{
		if (p->pid == power_stp->pid)
		{
			read_unlock(&tasklist_lock);
			goto find_ps;
		}
	}
	read_unlock(&tasklist_lock);
find_none:
	printk("BSP: No process to send.\n");
	return -1;
find_ps:
	send_sig_info(SIG_POWER, &info, p);
	return 0;
}


irqreturn_t power_irq(int irq, void *context_data)
{
	unsigned char reg;
	reg = power_in(OFFSET_POWER_INT);
	power_out(OFFSET_POWER_INT, POWER_INT_ACT);
	power_send_sig(); // send signal
	power_out(OFFSET_POWER_INT, POWER_INT_EN);
	return IRQ_HANDLED;
}

//------------------------------------------------------------------------------
// module functions
//------------------------------------------------------------------------------
static int power_open(struct inode *inode, struct file *filp)
{
	if (down_interruptible(&power_stp->sem))
		return - ERESTARTSYS;
	if (power_stp->count==0)
	{
		power_out(OFFSET_POWER_INT, POWER_INT_EN);
		// enable_irq(power_stp->irq);
	}
	power_stp->count++;
	up(&power_stp->sem);
	return 0;
}

static int power_release(struct inode *inode, struct file *filp)
{
	if (down_interruptible(&power_stp->sem))
		return - ERESTARTSYS;
	power_stp->count--;
	if (power_stp->count==0)
	{
		power_out(OFFSET_POWER_INT, 0x00);
		// disable_irq(power_stp->irq);
	}
	up(&power_stp->sem);
	return 0;
}

static ssize_t power_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	return 0;
}

static ssize_t power_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	return 0;
}

static int power_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	if (down_interruptible(&power_stp->sem))
		return - ERESTARTSYS;
	switch (cmd)
	{
		case POWER_SET_PID:
			power_stp->pid = (pid_t)arg;
			break;

		case POWER_GET_PEND:
			ret = power_in(OFFSET_POWER_PEND);
			break;

		case POWER_GET_STAT:
			ret = power_in(OFFSET_POWER_STAT);
			break;

		case POWER_P28_ON:
			power_out(OFFSET_POWER_STAT, POWER_STAT_P28ON);
			break;

		case POWER_P28_OFF:
			power_out(OFFSET_POWER_STAT, 0x00);
			break;

		default:
			ret = -ENOTTY;
			break;
	}
	up(&power_stp->sem);
	return ret;
}

//------------------------------------------------------------------------------
// register module
//------------------------------------------------------------------------------
static const struct file_operations power_fops = {
	.owner  = THIS_MODULE,
	.open   = power_open,
	.release= power_release,
	.read   = power_read,
	.write  = power_write,
	.ioctl  = power_ioctl,
};

static int __init power_init(void)
{
	dev_t devno;
	int ret = 0, err = 0;
	// register chrdev
	devno = MKDEV(MAJ_POWER, MIN_POWER);
	ret = register_chrdev_region(devno, 1, NAME_POWER);
	if (ret<0)
	{
		printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
		return ret;
	}
	// alloc dev
	power_stp = kmalloc(sizeof(struct power_st), GFP_KERNEL);
	if (!power_stp)
	{
		ret = - ENOMEM;
		goto fail_malloc;
	}
	memset(power_stp, 0, sizeof(struct power_st));
	init_MUTEX(&power_stp->sem);
	power_stp->pid = 0;
	// add cdev
	cdev_init(&power_stp->cdev, &power_fops);
	power_stp->cdev.owner = THIS_MODULE;
	power_stp->cdev.ops = &power_fops;
	err = cdev_add(&power_stp->cdev, devno, 1);
	if (err)
	{
		printk("BSP: %s fail cdev_add\n", __FUNCTION__);
		goto fail_remap;
	}
	// ioremap
	power_stp->regp = ioremap(BASE_POWER, CPLD_RMSIZE);
	if (power_stp->regp==NULL)
	{
		printk("BSP: %s fail ioremap\n", __FUNCTION__);
		goto fail_remap;
	}
	// request_irq
	set_irq_type(power_stp->irq, IRQ_TYPE_EDGE_FALLING);
	ret = request_irq(IRQ_GPI_01, power_irq, 0, "power", power_stp);
	if (ret != 0)
	{
		printk("BSP: %s fail request_irq\n", __FUNCTION__);
		goto fail_irq;
	}
	printk("NTS Power Driver installed\n");
	return 0;

fail_irq:
	iounmap(power_stp->regp);

fail_remap:
	kfree(power_stp);

fail_malloc:
	unregister_chrdev_region(devno, 1);
	printk("Fail to install NTS Power driver\n");
	return ret;
}

static void __exit power_exit(void)
{
	dev_t devno;
	iounmap(power_stp->regp);
	free_irq(power_stp->irq, power_stp);
	cdev_del(&power_stp->cdev);
	kfree(power_stp);
	devno = MKDEV(MAJ_POWER, MIN_POWER);
	unregister_chrdev_region(devno, 1);
	printk("NTS Power Driver removed\n");
}

module_init(power_init);
module_exit(power_exit);

MODULE_AUTHOR("Ray.Zhou, <ray.zhou@nts-intl.com>");
MODULE_DESCRIPTION("NTS POWER");
MODULE_LICENSE("GPL");
