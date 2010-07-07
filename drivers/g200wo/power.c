/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : power.c
 ::   :: ::       ::             ::     Generate   : 2009.06.01
 ::    ::::       ::       ::      ::   Update     : 2010-07-07 15:01:34
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	2010-07-07	Change cdev to miscdevices
	None
*/
#include <linux/fs.h>

#include <linux/irq.h>
#include <linux/interrupt.h>

#include <asm/io.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>

#include "g200wo_hw.h"
#include "power.h"

struct power_st
{
	struct miscdevice dev;
	struct semaphore sem;
	pid_t pid;
	int irq;
	int count;
} *power_stp;


static int power_send_sig(void)
{
	siginfo_t info;
	struct task_struct *p;

	info.si_signo = SIG_POWER;
	
	if (power_stp->pid<=0)
		goto find_none;
	
	read_lock(&tasklist_lock);
	
	for_each_process(p) {
		if (p->pid == power_stp->pid) {
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
	//clear irq
	__raw_writeb(POWER_INT_ACT, POWER_INT_BASE);

	power_send_sig();

	//enable irq
	__raw_writeb(POWER_INT_ENA, POWER_INT_BASE);

	return IRQ_HANDLED;
}

static int power_open(struct inode *inode, struct file *filp)
{
	if (down_interruptible(&power_stp->sem))
		return - ERESTARTSYS;

	if (power_stp->count==0) {
		__raw_writeb(POWER_INT_ENA, POWER_INT_BASE);
		enable_irq(power_stp->irq);
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

	if (power_stp->count==0) {
		__raw_writeb(0, POWER_INT_BASE);
		disable_irq(power_stp->irq);
	}

	up(&power_stp->sem);
	return 0;
}

static int power_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	if (down_interruptible(&power_stp->sem))
		return - ERESTARTSYS;
	
	switch (cmd) {
		case CMD_SET_POWER_P28:
			__raw_writeb(POWER_STAT_P28, POWER_STAT_BASE);
			break;

		case CMD_SET_POWER_PID:
			power_stp->pid = (pid_t)arg;
			break;

		case CMD_GET_POWER_STAT:
			ret = __raw_readb(POWER_STAT_BASE);
			break;

		case CMD_GET_POWER_PEND:
			ret = __raw_readb(POWER_PEND_BASE);
			break;

		default:
			ret = -ENOTTY;
			break;
	}

	up(&power_stp->sem);
	return ret;
}

static const struct file_operations power_fops = {
	.owner  = THIS_MODULE,
	.open   = power_open,
	.release= power_release,
	.read   = NULL,
	.write  = NULL,
	.ioctl  = power_ioctl,
};

static int __init power_init(void)
{
	int ret = 0;

	// malloc and initial
	power_stp = kmalloc(sizeof(struct power_st), GFP_KERNEL);
	if (!power_stp) {
		ret = - ENOMEM;
		goto fail_malloc;
	}

	memset(power_stp, 0, sizeof(struct power_st));
	init_MUTEX(&power_stp->sem);
	power_stp->pid = 0;
	power_stp->dev.minor = MISC_DYNAMIC_MINOR;
	power_stp->dev.name = "g200wo_power";
	power_stp->dev.fops = &power_fops;

	// registe device
	ret = misc_register(&power_stp->dev);
	if (ret) {
		printk("BSP: %s fail register device \n", __FUNCTION__);
		goto fail_registe;
	}

	// request_irq
	power_stp->irq = POWER_IRQ;
	set_irq_type(power_stp->irq, IRQ_TYPE_EDGE_FALLING);
	ret = request_irq(power_stp->irq, power_irq, IRQF_DISABLED, "power", power_stp);
	if (ret != 0) {
		printk("BSP: %s fail request_irq\n", __FUNCTION__);
		goto fail_reqirq;
	}

	printk("G200WO Power Driver installed\n");
	return 0;

fail_reqirq:
	misc_deregister(&power_stp->dev);

fail_registe:
	kfree(power_stp);

fail_malloc:
	printk("Fail to install G200WO Power driver\n");
	return ret;
}

static void __exit power_exit(void)
{
	free_irq(power_stp->irq, power_stp);
	misc_deregister(&power_stp->dev);
	kfree(power_stp);
	printk("G200WO Power Driver removed\n");
}

module_init(power_init);
module_exit(power_exit);

MODULE_AUTHOR("Ray.Zhou, <ray.zhou@nts-intl.com>");
MODULE_DESCRIPTION("G200WO POWER");
MODULE_LICENSE("GPL");
