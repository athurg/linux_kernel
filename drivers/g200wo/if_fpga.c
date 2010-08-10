/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      FileName   : if_fpga.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-08-10 13:34:11
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	2010-07-07	Change cdev to miscdevices
	FIXME:
		maybe we should use little-endian. check line 161,168,231,238.
*/

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include <mach/irqs.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
// For sys_kill(pid_t pid, int sig_no)
#include <linux/syscalls.h>

#include <g200wo/g200wo_hw.h>
#include <g200wo/if_fpga.h>

struct{
	struct miscdevice dev;
	struct semaphore sem;
	unsigned short *kbuf;
	pid_t pid;
	int irq_agc;
}if_fpga_st;


irqreturn_t if_agc_irq(int irq, void *context_data)
{
	//just send SIG_IF_AGC signal to user-space program while AGC WARNNING
	sys_kill(if_fpga_st.pid, SIG_IF_AGC);
	return IRQ_HANDLED;
}


static int if_fpga_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned short rtn=0;
	unsigned short data;
	unsigned int addr;
	data=arg&0xFFFF;

	if (down_interruptible(&if_fpga_st.sem))
		return - ERESTARTSYS;

	switch(cmd){
		case CMD_IF_SET_PID:
			if_fpga_st.pid = (pid_t)arg;
			break;

		case CMD_IF_FPGA_READ_WORD:
			// Address is the MSB 16bits
			addr =  (arg & 0xFFFF)*2 + IF_FPGA_BASE;
			rtn = __raw_readw(addr);
			break;

		case CMD_IF_FPGA_WRITE_WORD:
			// Address is the LSB 16bits, data is the MSB 16 bits
			addr = (arg & 0xFFFF)*2 + IF_FPGA_BASE;
			data = (arg >> 16)&0xFFFF;
			__raw_writew(data, addr);
			break;

		default:
			rtn = -ENOTTY;
	}

	up(&if_fpga_st.sem);

	return rtn;
}

static ssize_t if_fpga_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	unsigned int i,len,addr;	//len means size of data in BYTES
	struct if_fpga_elem elem;

	if(down_interruptible(&if_fpga_st.sem))
		return - ERESTARTSYS;

	// Get elem
	if(copy_from_user(&elem, buf, sizeof(elem))){
		printk("BSP: %s of elem fail\n", __FUNCTION__);
		up(&if_fpga_st.sem);
		return - EFAULT;
	}

	// Get a byte length from elem.wlen
	if ((elem.type != fifo) && (elem.type != normal))
		len = elem.wlen * 8;
	else	len = elem.wlen * 2;

	// Check if address and length is valid
	if((len > MAX_IF_FPGA_LEN) ||
		(((elem.addr + len) > 0xFFFF) && (elem.type == normal))){
		return -EFAULT;
	}

	//Get data and address
	if(copy_from_user(if_fpga_st.kbuf, elem.buf, len)){
		printk("BSP: %s of data fail\n", __FUNCTION__);
		up(&if_fpga_st.sem);
		return - EFAULT;
	}

	// write data
	//NOTE:
	//	elem.addr means the offset of FPGA but the EMC bus address
	addr = elem.addr*2 + IF_FPGA_BASE;
	switch(elem.type){
		case fifo:
		case normal:
			for(i=0; i<elem.wlen; i++){
				__raw_writew(if_fpga_st.kbuf[i], addr);
				if(elem.type==normal)
					addr+=2;
			}
			break;

			/*
			   Description of each group data

			   ADDR(OFFSET)	TYPE	BITS(16 bits)
			   ==================================
			   0		ADDR	15...0
			   1		DON't care
			   2		DATA	15..0
			   3		DATA	31..16
			 */
		case dpd:
		case cfr:
			for(i=0; i<elem.wlen; i++){
				__raw_writew(if_fpga_st.kbuf[4*i+0], (addr + OFFSET_DPD_ADDR));
				__raw_writew(if_fpga_st.kbuf[4*i+2], (addr + OFFSET_DPD_WR_DATAL));
				__raw_writew(if_fpga_st.kbuf[4*i+3], (addr + OFFSET_DPD_WR_DATAH));
			}
			break;
		default:
			return -EFAULT;
	}

	up(&if_fpga_st.sem);

	return len;
}

static ssize_t if_fpga_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	unsigned int i, len, addr;	//len means size of data in BYTES
	struct if_fpga_elem elem;

	if (down_interruptible(&if_fpga_st.sem))
		return - ERESTARTSYS;

	// copy elem
	if(copy_from_user(&elem, buf, sizeof(elem))){
		printk("BSP: %s of elem fail\n", __FUNCTION__);
		up(&if_fpga_st.sem);
		return - EFAULT;
	}

	if((elem.type != fifo) && (elem.type != normal))
		len = elem.wlen * 8;
	else	len = elem.wlen * 2;

	// Check address and length is valid
	if( (len > MAX_IF_FPGA_LEN) ||
			(((elem.addr + len) > 0xFFFF) && (elem.type == normal))) {
		up(&if_fpga_st.sem);
		return -EFAULT;
	}

	//Get address data while DPD and CFR mode
	if ((elem.type == dpd) || (elem.type == cfr)) {
		if(copy_from_user(if_fpga_st.kbuf, elem.buf, len)){
			printk("BSP: %s of data fail\n", __FUNCTION__);
			up(&if_fpga_st.sem);
			return - EFAULT;
		}
	}

	// read data from fpga
	//NOTE:
	//	elem.addr means the offset of FPGA but the EMC bus address
	addr = elem.addr*2 + IF_FPGA_BASE;

	switch(elem.type){
		case fifo:
		case normal:
			for (i=0; i<elem.wlen; i++){
				if_fpga_st.kbuf[i] = __raw_readw(addr);
				if (elem.type == normal)
					addr+=2;
			}
			break;

		case dpd:
		case cfr:
			for (i=0; i<elem.wlen; i++){
				/*
				   Description of each group data

				   ADDR(OFFSET)	TYPE	BITS(16 bits)
				   ==================================
				   0		ADDR	15...0
				   1		DON't care
				   2		DATA	15..0
				   3		DATA	31..16
				 */
				__raw_writew(if_fpga_st.kbuf[4*i], (addr + OFFSET_DPD_ADDR));
				if_fpga_st.kbuf[4*i+2] = __raw_readw(addr + OFFSET_DPD_RD_DATAL);
				if_fpga_st.kbuf[4*i+3] = __raw_readw(addr +  OFFSET_DPD_RD_DATAH);
			}
			break;
		default:
			up(&if_fpga_st.sem);
			return -EFAULT;
	}

	// copy data back to user-space
	if (copy_to_user(elem.buf, if_fpga_st.kbuf, len)){
		printk("BSP: %s of data to user-space fail\n", __FUNCTION__);
		up(&if_fpga_st.sem);
		return - EFAULT;
	}

	up(&if_fpga_st.sem);

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

	// malloc for data buffer
	kbuf = kzalloc(MAX_IF_FPGA_LEN, GFP_KERNEL);
	if (!kbuf) {
		ret = - ENOMEM;
		goto fail_malloc;
	}

	// initial structure
	memset(&if_fpga_st, 0, sizeof(if_fpga_st));

	if_fpga_st.kbuf = kbuf;
	init_MUTEX(&if_fpga_st.sem);

	if_fpga_st.dev.minor = MISC_DYNAMIC_MINOR;
	if_fpga_st.dev.name = "g200wo_if_fpga";
	if_fpga_st.dev.fops = &if_fpga_fops;

	// registe device
	ret = misc_register(&if_fpga_st.dev);
	if (ret) {
		printk("BSP: %s fail to registe device\n", __FUNCTION__);
		goto fail_registe;
	}

	// request_irq
	if_fpga_st.irq_agc = IF_AGC_IRQ;
	set_irq_type(if_fpga_st.irq_agc, IRQ_TYPE_EDGE_FALLING);
	ret = request_irq(if_fpga_st.irq_agc, if_agc_irq, IRQF_DISABLED, "if_fpga_agc", &if_fpga_st);
	if (ret) {
		printk("BSP: %s fail request_irq\n", __FUNCTION__);
		goto fail_reqirq;
	}

	printk("BSP: G200WO IF_FPGA Driver installed!\n");

	return 0;

fail_reqirq:
	misc_deregister(&if_fpga_st.dev);

fail_registe:
	kfree(kbuf);

fail_malloc:
	printk("BSP: Fail to install G200WO IF_FPGA driver\n");

	return ret;
}

static void __exit if_fpga_exit(void)
{
	misc_deregister(&if_fpga_st.dev);
	kfree(if_fpga_st.kbuf);
	printk("BSP: G200WO IF_FPGA Driver removed\n");
}

module_init(if_fpga_init);
module_exit(if_fpga_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO IF_FPGA");
MODULE_LICENSE("GPL");
