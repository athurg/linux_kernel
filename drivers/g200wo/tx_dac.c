//------------------------------------------------------------------------------
// tx_dac.c
// dac5682 spi driver
// 2009-06-02 NTS Ray.Zhou
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// include
//------------------------------------------------------------------------------
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
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
#include "tx_dac.h"

#define TX_DAC_CTRL_SDOUT   0x08
#define TX_DAC_CTRL_SEN     0x04
#define TX_DAC_CTRL_SDATA   0x02
#define TX_DAC_CTRL_SCLK    0x01

//------------------------------------------------------------------------------
// structure
//------------------------------------------------------------------------------
struct tx_dac_st
{
    struct cdev cdev;
    struct semaphore sem;
    volatile unsigned char __iomem *regp;
};

//------------------------------------------------------------------------------
// global
//------------------------------------------------------------------------------
struct tx_dac_st *tx_dac_stp;


//------------------------------------------------------------------------------
// io functions
//------------------------------------------------------------------------------
static inline u8 tx_dac_in(unsigned int off)
{
    return __raw_readb(&tx_dac_stp->regp[off]);
}

static inline void tx_dac_out(unsigned int off, u8 val)
{
    __raw_writeb(val, &tx_dac_stp->regp[off]);
}

//------------------------------------------------------------------------------
// hardware functions
//------------------------------------------------------------------------------
void tx_dac_spi_write(unsigned char addr, unsigned char data)
{
    unsigned char t;
    int i;
    tx_dac_out(OFFSET_TX_DAC_CTRL, 0x00);
    tx_dac_out(OFFSET_TX_DAC_CTRL, TX_DAC_CTRL_SEN);
    // addr
    t = addr & 0x0F; // (rw=0)->write, (n1=0,n0=0)->1byte
    for (i=0; i<8; i++)
    {
        if (t&0x80) // D15 is the MSB
        {
            tx_dac_out(OFFSET_TX_DAC_CTRL, TX_DAC_CTRL_SEN | TX_DAC_CTRL_SDATA);
            tx_dac_out(OFFSET_TX_DAC_CTRL, TX_DAC_CTRL_SEN | TX_DAC_CTRL_SDATA | TX_DAC_CTRL_SCLK);
        }
        else
        {
            tx_dac_out(OFFSET_TX_DAC_CTRL, TX_DAC_CTRL_SEN);
            tx_dac_out(OFFSET_TX_DAC_CTRL, TX_DAC_CTRL_SEN | TX_DAC_CTRL_SCLK);
        }
        t = t<<1;
    }
    // data
    t = data;
    for (i=0; i<8; i++)
    {
        if (t&0x80) // D15 is the MSB
        {
            tx_dac_out(OFFSET_TX_DAC_CTRL, TX_DAC_CTRL_SEN | TX_DAC_CTRL_SDATA);
            tx_dac_out(OFFSET_TX_DAC_CTRL, TX_DAC_CTRL_SEN | TX_DAC_CTRL_SDATA | TX_DAC_CTRL_SCLK);
        }
        else
        {
            tx_dac_out(OFFSET_TX_DAC_CTRL, TX_DAC_CTRL_SEN);
            tx_dac_out(OFFSET_TX_DAC_CTRL, TX_DAC_CTRL_SEN | TX_DAC_CTRL_SCLK);
        }
        t = t<<1;
    }
    tx_dac_out(OFFSET_TX_DAC_CTRL, TX_DAC_CTRL_SEN);
    // deselect all
    tx_dac_out(OFFSET_TX_DAC_CTRL, 0x00);
}

unsigned char tx_dac_spi_read(unsigned char addr)
{
    unsigned t, c;
    int i;
    tx_dac_out(OFFSET_TX_DAC_CTRL, 0x00);
    tx_dac_out(OFFSET_TX_DAC_CTRL, TX_DAC_CTRL_SEN);
    // addr
    t = addr | 0x80; // (rw=1)->read
    t = t & 0x8F; // (n1=0,n0=0)->1byte
    for (i=0; i<8; i++)
    {
        if (t&0x80) // D15 is the MSB
        {
            tx_dac_out(OFFSET_TX_DAC_CTRL, TX_DAC_CTRL_SEN | TX_DAC_CTRL_SDATA);
            tx_dac_out(OFFSET_TX_DAC_CTRL, TX_DAC_CTRL_SEN | TX_DAC_CTRL_SDATA | TX_DAC_CTRL_SCLK);
        }
        else
        {
            tx_dac_out(OFFSET_TX_DAC_CTRL, TX_DAC_CTRL_SEN);
            tx_dac_out(OFFSET_TX_DAC_CTRL, TX_DAC_CTRL_SEN | TX_DAC_CTRL_SCLK);
        }
        t = t<<1;
    }
    // data
    t = 0x00;
    for (i=0; i<8; i++)
    {
        t = t<<1;
        tx_dac_out(OFFSET_TX_DAC_CTRL, TX_DAC_CTRL_SEN);
        tx_dac_out(OFFSET_TX_DAC_CTRL, TX_DAC_CTRL_SEN | TX_DAC_CTRL_SCLK);
        c = tx_dac_in(OFFSET_TX_DAC_CTRL);
        if (c&TX_DAC_CTRL_SDOUT)
            t = t | 0x01;
    }
    tx_dac_out(OFFSET_TX_DAC_CTRL, TX_DAC_CTRL_SEN);
    // deselect all
    tx_dac_out(OFFSET_TX_DAC_CTRL, 0x00);
    return t;
}


//------------------------------------------------------------------------------
// module functions
//------------------------------------------------------------------------------
static int tx_dac_open(struct inode *inode, struct file *filp)
{
    return 0;
}

static int tx_dac_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static ssize_t tx_dac_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
    struct tx_dac_elem elem;
    if (down_interruptible(&tx_dac_stp->sem))
        return - ERESTARTSYS;
    if (copy_from_user(&elem, buf, sizeof(struct tx_dac_elem)))
    {
        printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
        up(&tx_dac_stp->sem);
        return  - EFAULT;
    }
    tx_dac_spi_write(elem.addr, elem.data);
    up(&tx_dac_stp->sem);
    return sizeof(struct tx_dac_elem);
}

static ssize_t tx_dac_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
    struct tx_dac_elem elem;
    if (down_interruptible(&tx_dac_stp->sem))
        return - ERESTARTSYS;
    if (copy_from_user(&elem, buf, sizeof(struct tx_dac_elem)))
    {
        printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
        up(&tx_dac_stp->sem);
        return  - EFAULT;
    }
    elem.data = tx_dac_spi_read(elem.addr);
    if (copy_to_user(buf, &elem, sizeof(struct tx_dac_elem)))
    {
        printk("BSP: %s fail copy_to_user\n", __FUNCTION__);
        up(&tx_dac_stp->sem);
        return - EFAULT;
    }
    up(&tx_dac_stp->sem);
    return sizeof(struct tx_dac_elem);
}

//------------------------------------------------------------------------------
// register module
//------------------------------------------------------------------------------
static const struct file_operations tx_dac_fops = {
        .owner  = THIS_MODULE,
        .open   = tx_dac_open,
        .release= tx_dac_release,
        .read   = tx_dac_read,
        .write  = tx_dac_write,
};

static int __init tx_dac_init(void)
{
    int ret = 0, err = 0;
    dev_t devno;
    // register chrdev
    devno = MKDEV(MAJ_TX_DAC, MIN_TX_DAC);
    ret = register_chrdev_region(devno, 1, NAME_TX_DAC);
    if (ret<0)
    {
        printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
        return ret;
    }
    // alloc dev
    tx_dac_stp = kmalloc(sizeof(struct tx_dac_st), GFP_KERNEL);
    if (!tx_dac_stp)
    {
        ret = - ENOMEM;
        goto fail_malloc;
    }
    memset(tx_dac_stp, 0, sizeof(struct tx_dac_st));
    init_MUTEX(&tx_dac_stp->sem);
    // add cdev
    cdev_init(&tx_dac_stp->cdev, &tx_dac_fops);
    tx_dac_stp->cdev.owner = THIS_MODULE;
    tx_dac_stp->cdev.ops = &tx_dac_fops;
    err = cdev_add(&tx_dac_stp->cdev, devno, 1);
    if (err)
    {
        printk("BSP: %s fail cdev_add\n", __FUNCTION__);
        goto fail_remap;
    }
    // ioremap
    tx_dac_stp->regp = ioremap(BASE_TX_DAC, CPLD_RMSIZE);
    if (tx_dac_stp->regp==NULL)
    {
        printk("BSP: %s fail ioremap\n", __FUNCTION__);
        goto fail_remap;
    }
    tx_dac_out(OFFSET_TX_DAC_CTRL, 0x00);
    printk("NTS TX_DAC Driver installed\n");
    return 0;

fail_remap:
    kfree(tx_dac_stp);

fail_malloc:
    unregister_chrdev_region(devno, 1);
    printk("Fail to install NTS TX_DAC driver\n");
    return ret;
}

static void __exit tx_dac_exit(void)
{
    dev_t devno;
    iounmap(tx_dac_stp->regp);
    cdev_del(&tx_dac_stp->cdev);
    kfree(tx_dac_stp);
    devno = MKDEV(MAJ_TX_DAC, MIN_TX_DAC);
    unregister_chrdev_region(devno, 1);
    printk("NTS TX_DAC Driver removed\n");
}

module_init(tx_dac_init);
module_exit(tx_dac_exit);

MODULE_AUTHOR("Ray.Zhou, <ray.zhou@nts-intl.com>");
MODULE_DESCRIPTION("NTS TX_DAC");
MODULE_LICENSE("GPL");
