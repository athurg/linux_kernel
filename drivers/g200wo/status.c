//------------------------------------------------------------------------------
// status.c
// status register and elpb detection driver
// 2009-05-31 NTS Ray.Zhou
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
#include "hardware.h"
#include "status.h"

#define DETECT_OLD      0x01
#define DETECT_PA       0x10
#define DETECT_DRY1     0x02
#define DETECT_DRY2     0x04
#define DETECT_DRY3     0x08

//------------------------------------------------------------------------------
// structure
//------------------------------------------------------------------------------
struct status_st
{
    struct cdev cdev;
    struct semaphore sem;
    volatile unsigned char __iomem *regp;
};

//------------------------------------------------------------------------------
// global
//------------------------------------------------------------------------------
struct status_st *status_stp;

//------------------------------------------------------------------------------
// io functions
//------------------------------------------------------------------------------
static inline u8 status_in(unsigned int off)
{
    return __raw_readb(&status_stp->regp[off]);
}

static inline void status_out(unsigned int off, u8 val)
{
    __raw_writeb(val, &status_stp->regp[off]);
}

//------------------------------------------------------------------------------
// module functions
//------------------------------------------------------------------------------
static int status_open(struct inode *inode, struct file *filp)
{
    return 0;
}

static int status_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static int status_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    unsigned char temp;
    if (down_interruptible(&status_stp->sem))
        return - ERESTARTSYS;
    switch (cmd)
    {
        case CMD_STATUS_OLD:
            temp = status_in(OFFSET_DETECT);
            ret = ((temp&DETECT_OLD)==DETECT_OLD)?1:0;
            break;

        case CMD_STATUS_PA:
            temp = status_in(OFFSET_DETECT);
            ret = ((temp&DETECT_PA)==DETECT_PA)?1:0;
            break;

        case CMD_STATUS_DRY1:
            temp = status_in(OFFSET_DETECT);
            ret = ((temp&DETECT_DRY1)==DETECT_DRY1)?1:0;
            break;

        case CMD_STATUS_DRY2:
            temp = status_in(OFFSET_DETECT);
            ret = ((temp&DETECT_DRY2)==DETECT_DRY2)?1:0;
            break;

        case CMD_STATUS_DRY3:
            temp = status_in(OFFSET_DETECT);
            ret = ((temp&DETECT_DRY3)==DETECT_DRY3)?1:0;
            break;

        default:
            ret = -ENOTTY;
            break;
    }
    up(&status_stp->sem);
    return ret;
}

static ssize_t status_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
    unsigned char sta;
    if (down_interruptible(&status_stp->sem))
        return - ERESTARTSYS;
    if (copy_from_user(&sta, buf, sizeof(sta)))
    {
        printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
        up(&status_stp->sem);
        return  - EFAULT;
    }
    status_out(OFFSET_STATUS, sta);
    up(&status_stp->sem);
    return sizeof(sta);
}

static ssize_t status_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
    unsigned char sta;
    if (down_interruptible(&status_stp->sem))
        return - ERESTARTSYS;
    sta = status_in(OFFSET_STATUS);
    if (copy_to_user(buf, &sta, sizeof(sta)))
    {
        printk("BSP: %s fail copy_to_user\n", __FUNCTION__);
        up(&status_stp->sem);
        return - EFAULT;
    }
    up(&status_stp->sem);
    return sizeof(sta);
}

//------------------------------------------------------------------------------
// register module
//------------------------------------------------------------------------------
static const struct file_operations status_fops = {
        .owner  = THIS_MODULE,
        .open   = status_open,
        .release= status_release,
        .read   = status_read,
        .write  = status_write,
        .ioctl  = status_ioctl,
};

static int __init status_init(void)
{
    int ret = 0, err = 0;
    dev_t devno;
    // register chrdev
    devno = MKDEV(MAJ_STATUS, MIN_STATUS);
    ret = register_chrdev_region(devno, 1, "g200wo_status");
    if (ret<0)
    {
        printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
        return ret;
    }
    // alloc dev
    status_stp = kmalloc(sizeof(struct status_st), GFP_KERNEL);
    if (!status_stp)
    {
        ret = - ENOMEM;
        goto fail_malloc;
    }
    memset(status_stp, 0, sizeof(struct status_st));
    init_MUTEX(&status_stp->sem);
    // add cdev
    cdev_init(&status_stp->cdev, &status_fops);
    status_stp->cdev.owner = THIS_MODULE;
    status_stp->cdev.ops = &status_fops;
    err = cdev_add(&status_stp->cdev, devno, 1);
    if (err)
    {
        printk("BSP: %s fail cdev_add\n", __FUNCTION__);
        goto fail_remap;
    }
    // ioremap
    status_stp->regp = ioremap(STATUS_BASE, CPLD_RMSIZE);
    if (status_stp->regp==NULL)
    {
        printk("BSP: %s fail ioremap\n", __FUNCTION__);
        goto fail_remap;
    }
    printk("NTS R804XY STATUS Driver installed\n");
    return 0;

fail_remap:
    kfree(status_stp);

fail_malloc:
    unregister_chrdev_region(devno, 1);
    printk("Fail to install NTS R804XY STATUS driver\n");
    return ret;
}

static void __exit status_exit(void)
{
    dev_t devno;
    iounmap(status_stp->regp);
    cdev_del(&status_stp->cdev);
    kfree(status_stp);
    devno = MKDEV(MAJ_STATUS, MIN_STATUS);
    unregister_chrdev_region(devno, 1);
    printk("NTS R804XY STATUS Driver removed\n");
}

module_init(status_init);
module_exit(status_exit);

MODULE_AUTHOR("Ray.Zhou, <ray.zhou@nts-intl.com>");
MODULE_DESCRIPTION("NTS R804XY STATUS");
MODULE_LICENSE("GPL");
