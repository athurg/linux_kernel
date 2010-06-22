//------------------------------------------------------------------------------
// if_fpga.c
// dpd fpga driver
// 2009-06-02 NTS Ray.Zhou
//------------------------------------------------------------------------------

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
#include "if_fpga.h"

#define CFR_ADDR_A      0x100
#define CFR_DATAL_A     0x101
#define CFR_DATAH_A     0x102

#define CFR_ADDR_B      0x200
#define CFR_DATAL_B     0x201
#define CFR_DATAH_B     0x202

#define DPD_ADDR        0x300
#define DPD_WDATAL      0x301
#define DPD_WDATAH      0x302
#define DPD_RDATAL      0x303
#define DPD_RDATAH      0x304
//------------------------------------------------------------------------------
// structure
//------------------------------------------------------------------------------
struct if_fpga_st
{
    struct cdev cdev;
    struct semaphore sem;
    volatile unsigned short __iomem *regp;
    unsigned short *kbuf;
    pid_t pid;
    int irq_agc;
};

//------------------------------------------------------------------------------
// global
//------------------------------------------------------------------------------
struct if_fpga_st *if_fpga_stp;

//------------------------------------------------------------------------------
// io functions
//------------------------------------------------------------------------------
static inline u16 if_fpga_in(unsigned int off)
{
    return __raw_readw(&if_fpga_stp->regp[off]);
}

static inline void if_fpga_out(unsigned int off, u16 val)
{
    __raw_writew(val,&if_fpga_stp->regp[off]);
}

static int if_send_sig(int signo)
{
    siginfo_t info;
    struct task_struct *p;
    info.si_signo = signo;
    if (if_fpga_stp->pid<=0)
        goto find_none;
    read_lock(&tasklist_lock);
    for_each_process(p)
    {
        if (p->pid == if_fpga_stp->pid)
        {
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

//------------------------------------------------------------------------------
// module functions
//------------------------------------------------------------------------------
static int if_fpga_open(struct inode *inode, struct file *filp)
{
    return 0;
}

static int if_fpga_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static int if_fpga_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    unsigned short data, addr;
    if (down_interruptible(&if_fpga_stp->sem))
        return - ERESTARTSYS;
    switch (cmd)
    {
        case CMD_IF_SET_PID:
            if_fpga_stp->pid = (pid_t)arg;
            up(&if_fpga_stp->sem);
            return 0;

        case CMD_IF_FPGA_READ_WORD:
            addr = arg&0xFFFF;
            data = if_fpga_in(addr);
            up(&if_fpga_stp->sem);
            return (int)data;

        case CMD_IF_FPGA_WRITE_WORD:
            addr = arg&0xFFFF;
            data = (arg>>16)&0xFFFF;
            if_fpga_out(addr, data);
            up(&if_fpga_stp->sem);
            return 0;

        default:
            up(&if_fpga_stp->sem);
            return -ENOTTY;
    }
    up(&if_fpga_stp->sem);
    return 0;
}

static ssize_t if_fpga_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
    unsigned short *datap;
    unsigned int *bufp;
    unsigned short addr, datal, datah;
    int i, len;
    struct if_fpga_elem elem;
    if (down_interruptible(&if_fpga_stp->sem))
        return - ERESTARTSYS;
    // copy elem
    if (copy_from_user(&elem, buf, sizeof(struct if_fpga_elem)))
    {
        printk("BSP: %s copy_from_user elem\n", __FUNCTION__);
        up(&if_fpga_stp->sem);
        return - EFAULT;
    }
    // copy data
    if ((elem.type == TPYE_IF_FPGA_NORMAL)||(elem.type == TPYE_IF_FPGA_FIFO))
        len = elem.wlen*2;
    else
        len = elem.wlen*8;
    if (len>MAX_IF_FPGA_BLEN)
        len = MAX_IF_FPGA_BLEN;
    if (copy_from_user(if_fpga_stp->kbuf, elem.buf, len))
    {
        printk("BSP: %s copy_from_user data\n", __FUNCTION__);
        up(&if_fpga_stp->sem);
        return - EFAULT;
    }
    // write data
    datap = if_fpga_stp->kbuf;
    bufp = (unsigned int *)if_fpga_stp->kbuf;
    if (elem.type == TPYE_IF_FPGA_FIFO)
    {
        elem.wlen = len/2;
        for (i=0; i<elem.wlen; i++)
            if_fpga_out(elem.addr, datap[i]);
    }
    else if (elem.type == TPYE_IF_FPGA_NORMAL)
    {
        elem.wlen = len/2;
        for (i=0; i<elem.wlen; i++)
            if (elem.addr+i<=0xFFFF)
                if_fpga_out(elem.addr+i, datap[i]);
    }
    else if (elem.type == TPYE_IF_FPGA_CFRA)
    {
        elem.wlen = len/8;
        for (i=0; i<elem.wlen; i++)
        {
            addr = bufp[2*i];
            datal = bufp[2*i+1];
            datah = bufp[2*i+1]>>16;
            if_fpga_out(CFR_ADDR_A, addr);
            if_fpga_out(CFR_DATAL_A, datal);
            if_fpga_out(CFR_DATAH_A, datah);
        }
    }
    else if (elem.type == TPYE_IF_FPGA_CFRB)
    {
        elem.wlen = len/8;
        for (i=0; i<elem.wlen; i++)
        {
            addr = bufp[2*i];
            datal = bufp[2*i+1];
            datah = bufp[2*i+1]>>16;
            if_fpga_out(CFR_ADDR_B, addr);
            if_fpga_out(CFR_DATAL_B, datal);
            if_fpga_out(CFR_DATAH_B, datah);
        }
    }
    else if (elem.type == TPYE_IF_FPGA_DPD)
    {
        elem.wlen = len/8;
        for (i=0; i<elem.wlen; i++)
        {
            addr = bufp[2*i];
            datal = bufp[2*i+1];
            datah = bufp[2*i+1]>>16;
            if_fpga_out(DPD_ADDR, addr);
            if_fpga_out(DPD_WDATAL, datal);
            if_fpga_out(DPD_WDATAH, datah);
        }
    }
    // finish
    up(&if_fpga_stp->sem);
    return len;
}

static ssize_t if_fpga_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
    unsigned short *datap;
    unsigned int *bufp;
    unsigned short addr, datal, datah;
    int i, len;
    struct if_fpga_elem elem;
    if (down_interruptible(&if_fpga_stp->sem))
        return - ERESTARTSYS;
    // copy elem
    if (copy_from_user(&elem, buf, sizeof(struct if_fpga_elem)))
    {
        printk("BSP: %s copy_from_user elem\n", __FUNCTION__);
        up(&if_fpga_stp->sem);
        return - EFAULT;
    }
    // get addr
    if ((elem.type == TPYE_IF_FPGA_NORMAL)||(elem.type == TPYE_IF_FPGA_FIFO))
        len = elem.wlen*2;
    else
        len = elem.wlen*8;
    if (len>MAX_IF_FPGA_BLEN)
        len = MAX_IF_FPGA_BLEN;
    if ((elem.type != TPYE_IF_FPGA_NORMAL)&&(elem.type != TPYE_IF_FPGA_FIFO))
    {
        if (copy_from_user(if_fpga_stp->kbuf, elem.buf, len))
        {
            printk("BSP: %s copy_from_user data\n", __FUNCTION__);
            up(&if_fpga_stp->sem);
            return - EFAULT;
        }
    }
    // read data
    datap = if_fpga_stp->kbuf;
    bufp = (unsigned int *)if_fpga_stp->kbuf;
    if (elem.type == TPYE_IF_FPGA_FIFO)
    {
        elem.wlen = len/2;
        for (i=0; i<elem.wlen; i++)
            datap[i] = if_fpga_in(elem.addr);
    }
    else if (elem.type == TPYE_IF_FPGA_NORMAL)
    {
        elem.wlen = len/2;
        for (i=0; i<elem.wlen; i++)
            if (elem.addr+i<=0xFFFF)
                datap[i] = if_fpga_in(elem.addr+i);
    }
    else if (elem.type == TPYE_IF_FPGA_CFRA)
    {
        elem.wlen = len/8;
        for (i=0; i<elem.wlen; i++)
        {
            addr = bufp[2*i];
            if_fpga_out(CFR_ADDR_A, addr);
            datal = if_fpga_in(CFR_DATAL_A);
            datah = if_fpga_in(CFR_DATAH_A);
            bufp[2*i+1] = (datah<<16)|datal;
        }
    }
    else if (elem.type == TPYE_IF_FPGA_CFRB)
    {
        elem.wlen = len/8;
        for (i=0; i<elem.wlen; i++)
        {
            addr = bufp[2*i];
            if_fpga_out(CFR_ADDR_B, addr);
            datal = if_fpga_in(CFR_DATAL_B);
            datah = if_fpga_in(CFR_DATAH_B);
            bufp[2*i+1] = (datah<<16)|datal;
        }
    }
    else if (elem.type == TPYE_IF_FPGA_DPD)
    {
        elem.wlen = len/8;
        for (i=0; i<elem.wlen; i++)
        {
            addr = bufp[2*i];
            if_fpga_out(DPD_ADDR, addr);
            datal = if_fpga_in(DPD_RDATAL);
            datah = if_fpga_in(DPD_RDATAH);
            bufp[2*i+1] = (datah<<16)|datal;
        }
    }
    // copy data
    if (copy_to_user(elem.buf, if_fpga_stp->kbuf, len))
    {
        printk("BSP: %s copy_to_user data\n", __FUNCTION__);
        up(&if_fpga_stp->sem);
        return - EFAULT;
    }
    // finish
    up(&if_fpga_stp->sem);
    return len;
}

//------------------------------------------------------------------------------
// register module
//------------------------------------------------------------------------------
static const struct file_operations if_fpga_fops = {
        .owner  = THIS_MODULE,
        .open   = if_fpga_open,
        .release= if_fpga_release,
        .ioctl  = if_fpga_ioctl,
        .read   = if_fpga_read,
        .write  = if_fpga_write,
};

static int __init if_fpga_init(void)
{
    int ret = 0, err = 0;
    void *kbuf;
    dev_t devno;
    // register chrdev
    devno = MKDEV(MAJ_IF_FPGA, MIN_IF_FPGA);
    ret = register_chrdev_region(devno, 1, NAME_IF_FPGA);
    if (ret<0)
    {
        printk("BSP: %s register_chrdev_region\n", __FUNCTION__);
        return ret;
    }
    // alloc dev
    if_fpga_stp = kmalloc(sizeof(struct if_fpga_st), GFP_KERNEL);
    if (!if_fpga_stp)
    {
        ret = - ENOMEM;
        goto fail_malloc;
    }
    memset(if_fpga_stp, 0, sizeof(struct if_fpga_st));
    // alloc kbuf
    kbuf = kzalloc(MAX_IF_FPGA_WLEN*2, GFP_KERNEL);
    if (!kbuf)
    {
        ret = - ENOMEM;
        goto fail_malloc2;
    }
    if_fpga_stp->kbuf = kbuf;
    init_MUTEX(&if_fpga_stp->sem);
    // add cdev
    cdev_init(&if_fpga_stp->cdev, &if_fpga_fops);
    if_fpga_stp->cdev.owner = THIS_MODULE;
    if_fpga_stp->cdev.ops = &if_fpga_fops;
    err = cdev_add(&if_fpga_stp->cdev, devno, 1);
    if (err)
    {
        printk("BSP: %s cdev_add\n", __FUNCTION__);
        goto fail_remap;
    }
    // ioremap
    if_fpga_stp->regp = ioremap(IF_FPGA_BASE, IF_FPGA_RMSIZE);
    if (if_fpga_stp->regp==NULL)
    {
        printk("BSP: %s ioremap\n", __FUNCTION__);
        goto fail_remap;
    }
    // request_irq
    set_irq_type(if_fpga_stp->irq_agc, IRQ_TYPE_EDGE_FALLING);
    ret = request_irq(IRQ_GPI_00, if_agc_irq, 0, "if_agc", if_fpga_stp);
    if (ret != 0)
    {
        printk("BSP: %s fail request_irq\n", __FUNCTION__);
        goto fail_irq;
    }
    printk("NTS IF_FPGA Driver installed\n");
    return 0;

fail_irq:
    iounmap(if_fpga_stp->regp);

fail_remap:
    kfree(kbuf);

fail_malloc2:
    kfree(if_fpga_stp);

fail_malloc:
    unregister_chrdev_region(devno, 1);
    printk("Fail to install NTS IF_FPGA driver\n");
    return ret;
}

static void __exit if_fpga_exit(void)
{
    dev_t devno;
    iounmap(if_fpga_stp->regp);
    cdev_del(&if_fpga_stp->cdev);
    kfree(if_fpga_stp->kbuf);
    kfree(if_fpga_stp);
    devno = MKDEV(MAJ_IF_FPGA, MIN_IF_FPGA);
    unregister_chrdev_region(devno, 1);
    printk("NTS IF_FPGA Driver removed\n");
}

module_init(if_fpga_init);
module_exit(if_fpga_exit);

MODULE_AUTHOR("Ray.Zhou, <ray.zhou@nts-intl.com>");
MODULE_DESCRIPTION("NTS IF_FPGA");
MODULE_LICENSE("GPL");
