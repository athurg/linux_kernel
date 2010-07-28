/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      FileName  : power.c
 ::   :: ::       ::             ::     Generate   : 2009.06.01
 ::    ::::       ::       ::      ::   Update     : 2010-07-28 15:28:32
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
//For sys_kill(pid_t pid, int sig_no)
#include <linux/syscalls.h>

#include <g200wo/g200wo_hw.h>
#include <g200wo/power.h>

struct{
	struct miscdevice dev;
	struct semaphore sem;
	pid_t pid;
	int irq;
	int count;
}power_st;

irqreturn_t power_irq(int irq, void *context_data)
{
	//clear irq
	__raw_writeb(POWER_INT_ACT, POWER_INT_BASE);

	sys_kill(power_st.pid, SIG_POWER);

	//enable irq
	__raw_writeb(POWER_INT_ENA, POWER_INT_BASE);

	return IRQ_HANDLED;
}

static int power_open(struct inode *inode, struct file *filp)
{
	if (down_interruptible(&power_st.sem))
		return - ERESTARTSYS;

	if (power_st.count==0) {
		__raw_writeb(POWER_INT_ENA, POWER_INT_BASE);
		enable_irq(power_st.irq);
	}

	power_st.count++;

	up(&power_st.sem);
	return 0;
}

static int power_release(struct inode *inode, struct file *filp)
{
	if (down_interruptible(&power_st.sem))
		return - ERESTARTSYS;
	power_st.count--;

	if (power_st.count==0) {
		__raw_writeb(0, POWER_INT_BASE);
		disable_irq(power_st.irq);
	}

	up(&power_st.sem);
	return 0;
}

static int power_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	if (down_interruptible(&power_st.sem))
		return - ERESTARTSYS;

	switch (cmd) {
		case CMD_SET_POWER_P28:
			__raw_writeb(POWER_STAT_P28, POWER_STAT_BASE);
			break;

		case CMD_SET_POWER_PID:
			power_st.pid = (pid_t)arg;
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

	up(&power_st.sem);
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

	// Initial structure
	memset(&power_st, 0, sizeof(power_st));

	init_MUTEX(&power_st.sem);

	power_st.pid = 0;

	power_st.irq = POWER_IRQ;
	set_irq_type(power_st.irq, IRQ_TYPE_EDGE_FALLING);

	power_st.dev.minor = MISC_DYNAMIC_MINOR;
	power_st.dev.name = "g200wo_power";
	power_st.dev.fops = &power_fops;

	// Registe device
	ret = misc_register(&power_st.dev);
	if (ret) {
		printk("BSP: %s fail to register device \n", __FUNCTION__);
		goto fail_registe;
	}

	// Request_irq
	ret = request_irq(power_st.irq, power_irq, IRQF_DISABLED, "power", &power_st);
	if (ret) {
		printk("BSP: %s fail request_irq\n", __FUNCTION__);
		goto fail_reqirq;
	}

	printk("BSP: G200WO Power Driver installed\n");
	return 0;

fail_reqirq:
	misc_deregister(&power_st.dev);

fail_registe:
	printk("BSP: Fail to install G200WO Power driver\n");
	return ret;
}

static void __exit power_exit(void)
{
	free_irq(power_st.irq, &power_st);
	misc_deregister(&power_st.dev);
	printk("BSP: 200WO Power Driver removed\n");
}

module_init(power_init);
module_exit(power_exit);

MODULE_AUTHOR("Ray.Zhou, <ray.zhou@nts-intl.com>");
MODULE_DESCRIPTION("G200WO POWER");
MODULE_LICENSE("GPL");
