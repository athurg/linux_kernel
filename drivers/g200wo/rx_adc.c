//------------------------------------------------------------------------------
// rx_adc.c
// ads62c17 spi driver
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
#include "rx_adc.h"

#define RX_ADC_SDOUT     0x08
#define RX_ADC_SEN       0x04
#define RX_ADC_SDATA     0x02
#define RX_ADC_SCLK      0x01

//------------------------------------------------------------------------------
// structure
//------------------------------------------------------------------------------
struct rx_adc_st
{
    struct cdev cdev;
    struct semaphore sem;
    volatile unsigned char __iomem *regp;
};

//------------------------------------------------------------------------------
// global
//------------------------------------------------------------------------------
struct rx_adc_st *rx_adc_stp;

//------------------------------------------------------------------------------
// io functions
//------------------------------------------------------------------------------
static inline u8 rx_adc_in(unsigned int off)
{
    return __raw_readb(&rx_adc_stp->regp[off]);
}

static inline void rx_adc_out(unsigned int off, u8 val)
{
    __raw_writeb(val, &rx_adc_stp->regp[off]);
}

//------------------------------------------------------------------------------
// hardware functions
//------------------------------------------------------------------------------
unsigned char rx_adc_transfer(unsigned char addr, unsigned char data)
{
    unsigned char t, dout, din;
    int i;
    // select one chip
    rx_adc_out(OFFSET_RX_ADC_CTRL, 0x00);
    rx_adc_out(OFFSET_RX_ADC_CTRL, RX_ADC_SEN);
    // addr
    dout = addr;
    for (i=0; i<8; i++)
    {
        if (dout&0x80)
        {
            rx_adc_out(OFFSET_RX_ADC_CTRL, RX_ADC_SEN|RX_ADC_SDATA);
            rx_adc_out(OFFSET_RX_ADC_CTRL, RX_ADC_SEN|RX_ADC_SDATA|RX_ADC_SCLK);
        }
        else
        {
            rx_adc_out(OFFSET_RX_ADC_CTRL, RX_ADC_SEN);
            rx_adc_out(OFFSET_RX_ADC_CTRL, RX_ADC_SEN|RX_ADC_SCLK);
        }
        dout = dout<<1;
    }
    // data
    dout = data;
    din = 0x00;
    for (i=0; i<8; i++)
    {
        din = din<<1;
        if (dout&0x80)
        {
            rx_adc_out(OFFSET_RX_ADC_CTRL, RX_ADC_SEN|RX_ADC_SDATA);
            rx_adc_out(OFFSET_RX_ADC_CTRL, RX_ADC_SEN|RX_ADC_SDATA|RX_ADC_SCLK);
            t = rx_adc_in(OFFSET_RX_ADC_CTRL);
            if (t&RX_ADC_SDOUT)
                din = din | 0x01;
        }
        else
        {
            rx_adc_out(OFFSET_RX_ADC_CTRL, RX_ADC_SEN);
            rx_adc_out(OFFSET_RX_ADC_CTRL, RX_ADC_SEN|RX_ADC_SCLK);
            t = rx_adc_in(OFFSET_RX_ADC_CTRL);
            if (t&RX_ADC_SDOUT)
                din = din | 0x01;
        }
        dout = dout<<1;
    }
    rx_adc_out(OFFSET_RX_ADC_CTRL, RX_ADC_SEN);
    rx_adc_out(OFFSET_RX_ADC_CTRL, 0x00);
    return din;
}

//------------------------------------------------------------------------------
// module functions
//------------------------------------------------------------------------------
static int rx_adc_open(struct inode *inode, struct file *filp)
{
    return 0;
}

static int rx_adc_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static ssize_t rx_adc_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
    struct rx_adc_elem elem;
    if (down_interruptible(&rx_adc_stp->sem))
        return - ERESTARTSYS;
    if (copy_from_user(&elem, buf, sizeof(struct rx_adc_elem)))
    {
        printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
        up(&rx_adc_stp->sem);
        return  - EFAULT;
    }
    rx_adc_transfer(0x00, 0x00); // enable write
    rx_adc_transfer(elem.addr, elem.data);
    up(&rx_adc_stp->sem);
    return sizeof(struct rx_adc_elem);
}

static ssize_t rx_adc_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
    struct rx_adc_elem elem;
    if (down_interruptible(&rx_adc_stp->sem))
        return - ERESTARTSYS;
    if (copy_from_user(&elem, buf, sizeof(struct rx_adc_elem)))
    {
        printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
        up(&rx_adc_stp->sem);
        return  - EFAULT;
    }
    rx_adc_transfer(0x00, 0x01); // enable read
    elem.data = rx_adc_transfer(elem.addr, 0x00);
    rx_adc_transfer(0x00, 0x00); // enable write
    if (copy_to_user(buf, &elem, sizeof(struct rx_adc_elem)))
    {
        printk("BSP: %s fail copy_to_user\n", __FUNCTION__);
        up(&rx_adc_stp->sem);
        return - EFAULT;
    }
    up(&rx_adc_stp->sem);
    return sizeof(struct rx_adc_elem);
}

//------------------------------------------------------------------------------
// register module
//------------------------------------------------------------------------------
static const struct file_operations rx_adc_fops = {
        .owner  = THIS_MODULE,
        .open   = rx_adc_open,
        .release= rx_adc_release,
        .read   = rx_adc_read,
        .write  = rx_adc_write,
};

static int __init rx_adc_init(void)
{
    int ret = 0, err = 0;
    dev_t devno;
    // register chrdev
    devno = MKDEV(MAJ_RX_ADC, MIN_RX_ADC);
    ret = register_chrdev_region(devno, 1, NAME_RX_ADC);
    if (ret<0)
    {
        printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
        return ret;
    }
    // alloc dev
    rx_adc_stp = kmalloc(sizeof(struct rx_adc_st), GFP_KERNEL);
    if (!rx_adc_stp)
    {
        ret = - ENOMEM;
        goto fail_malloc;
    }
    memset(rx_adc_stp, 0, sizeof(struct rx_adc_st));
    init_MUTEX(&rx_adc_stp->sem);
    // add cdev
    cdev_init(&rx_adc_stp->cdev, &rx_adc_fops);
    rx_adc_stp->cdev.owner = THIS_MODULE;
    rx_adc_stp->cdev.ops = &rx_adc_fops;
    err = cdev_add(&rx_adc_stp->cdev, devno, 1);
    if (err)
    {
        printk("BSP: %s fail cdev_add\n", __FUNCTION__);
        goto fail_remap;
    }
    // ioremap
    rx_adc_stp->regp = ioremap(BASE_RX_ADC, CPLD_RMSIZE);
    if (rx_adc_stp->regp==NULL)
    {
        printk("BSP: %s fail ioremap\n", __FUNCTION__);
        goto fail_remap;
    }
    rx_adc_out(OFFSET_RX_ADC_CTRL, 0x00);
    printk("NTS RX_ADC Driver installed\n");
    return 0;

fail_remap:
    kfree(rx_adc_stp);

fail_malloc:
    unregister_chrdev_region(devno, 1);
    printk("Fail to install NTS RX_ADC driver\n");
    return ret;
}

static void __exit rx_adc_exit(void)
{
    dev_t devno;
    iounmap(rx_adc_stp->regp);
    cdev_del(&rx_adc_stp->cdev);
    kfree(rx_adc_stp);
    devno = MKDEV(MAJ_RX_ADC, MIN_RX_ADC);
    unregister_chrdev_region(devno, 1);
    printk("NTS RX_ADC Driver removed\n");
}

module_init(rx_adc_init);
module_exit(rx_adc_exit);

MODULE_AUTHOR("Ray.Zhou, <ray.zhou@nts-intl.com>");
MODULE_DESCRIPTION("NTS RX_ADC");
MODULE_LICENSE("GPL");
