//------------------------------------------------------------------------------
// reset.c
// reset control driver
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
#include "reset.h"

//------------------------------------------------------------------------------
// structure
//------------------------------------------------------------------------------
struct reset_st
{
    struct cdev cdev;
    struct semaphore sem;
    volatile unsigned char __iomem *regp;
};

//------------------------------------------------------------------------------
// global
//------------------------------------------------------------------------------
struct reset_st *reset_stp;

//------------------------------------------------------------------------------
// io functions
//------------------------------------------------------------------------------
static inline u8 reset_in(unsigned int off)
{
    return __raw_readb(&reset_stp->regp[off]);
}

static inline void reset_out(unsigned int off, u8 val)
{
    __raw_writeb(val, &reset_stp->regp[off]);
}

//------------------------------------------------------------------------------
// module functions
//------------------------------------------------------------------------------
static int reset_open(struct inode *inode, struct file *filp)
{
    return 0;
}

static int reset_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static ssize_t reset_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
    return 0;
}

static ssize_t reset_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
    return 0;
}

static int reset_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    unsigned char ctrl;
    if (down_interruptible(&reset_stp->sem))
        return - ERESTARTSYS;
    switch (cmd)
    {
        case CMD_RESET_SET:
            ctrl = arg&0xFF;
            reset_out(OFFSET_RESET_CTRL, ctrl);
            break;
        case CMD_RESET_ENABLE:
            reset_out(OFFSET_RESET_ENABLE, 0x01);
            break;

        case CMD_RESET_DISABLE:
            reset_out(OFFSET_RESET_ENABLE, 0x00);
            break;
        default:
            ret = -ENOTTY;
            break;
    }
    up(&reset_stp->sem);
    return ret;
}

//------------------------------------------------------------------------------
// register module
//------------------------------------------------------------------------------
static const struct file_operations reset_fops = {
        .owner  = THIS_MODULE,
        .open   = reset_open,
        .release= reset_release,
        .read   = reset_read,
        .write  = reset_write,
        .ioctl  = reset_ioctl,
};

static int __init reset_init(void)
{
    int ret = 0, err = 0;
    dev_t devno;
    // register chrdev
    devno = MKDEV(MAJ_RESET, MIN_RESET);
    ret = register_chrdev_region(devno, 1, NAME_RESET);
    if (ret<0)
    {
        printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
        return ret;
    }
    // alloc dev
    reset_stp = kmalloc(sizeof(struct reset_st), GFP_KERNEL);
    if (!reset_stp)
    {
        ret = - ENOMEM;
        goto fail_malloc;
    }
    memset(reset_stp, 0, sizeof(struct reset_st));
    init_MUTEX(&reset_stp->sem);
    // add cdev
    cdev_init(&reset_stp->cdev, &reset_fops);
    reset_stp->cdev.owner = THIS_MODULE;
    reset_stp->cdev.ops = &reset_fops;
    err = cdev_add(&reset_stp->cdev, devno, 1);
    if (err)
    {
        printk("BSP: %s fail cdev_add\n", __FUNCTION__);
        goto fail_remap;
    }
    // ioremap
    reset_stp->regp = ioremap(BASE_RESET, CPLD_RMSIZE);
    if (reset_stp->regp==NULL)
    {
        printk("BSP: %s fail ioremap\n", __FUNCTION__);
        goto fail_remap;
    }
    printk("NTS RESET Driver installed\n");
    return 0;

fail_remap:
    kfree(reset_stp);

fail_malloc:
    unregister_chrdev_region(devno, 1);
    printk("Fail to install NTS RESET driver\n");
    return ret;
}

static void __exit reset_exit(void)
{
    dev_t devno;
    iounmap(reset_stp->regp);
    cdev_del(&reset_stp->cdev);
    kfree(reset_stp);
    devno = MKDEV(MAJ_RESET, MIN_RESET);
    unregister_chrdev_region(devno, 1);
    printk("NTS RESET Driver removed\n");
}

module_init(reset_init);
module_exit(reset_exit);

MODULE_AUTHOR("Ray.Zhou, <ray.zhou@nts-intl.com>");
MODULE_DESCRIPTION("NTS RESET");
MODULE_LICENSE("GPL");
