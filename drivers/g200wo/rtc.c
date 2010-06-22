//------------------------------------------------------------------------------
// rtc.c
// MPC8313E intrenal RTC driver
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
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/semaphore.h>
#include "rtc.h"

//------------------------------------------------------------------------------
// configuration
//------------------------------------------------------------------------------

//#define CLK_IN          (333330000/2) //333MHz
#define CLK_IN          (266664000/2) //266MHz
#define REG_BASE        0xE0000300
#define REG_SIZE        0x20
#define OFF_RTCNR       0x00
#define OFF_RTLDR       0x01
#define OFF_RTPSR       0x02
#define OFF_RTCTR       0x03
#define OFF_RTEVR       0x04
#define OFF_RTALR       0x05

#define DEFAULT_RTCNR   0x00000080
#define DEFAULT_RTPSR   (CLK_IN-1)

//------------------------------------------------------------------------------
// structure
//------------------------------------------------------------------------------
struct rtc_st
{
    struct cdev cdev;
    struct semaphore sem;
    volatile unsigned int __iomem *regp;
};

//------------------------------------------------------------------------------
// global
//------------------------------------------------------------------------------
struct rtc_st *rtc_stp;

//------------------------------------------------------------------------------
// io functions
//------------------------------------------------------------------------------
static inline u32 rtc_in(unsigned int off)
{
    return __raw_readl(&rtc_stp->regp[off]);
}

static inline void rtc_out(unsigned int off, u32 val)
{
    __raw_writel(val, &rtc_stp->regp[off]);
}

//------------------------------------------------------------------------------
// module functions
//------------------------------------------------------------------------------
static int rtc_open(struct inode *inode, struct file *filp)
{
    return 0;
}

static int rtc_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static ssize_t rtc_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
    time_t rtc_time;
    if (down_interruptible(&rtc_stp->sem))
        return - ERESTARTSYS;
    if (copy_from_user(&rtc_time, buf, sizeof(time_t)))
    {
        printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
        up(&rtc_stp->sem);
        return  - EFAULT;
    }
    //rtc_stp->regp[OFFSET_SYSTEM_RTC] = rtc_time;
    //
    rtc_out(OFF_RTCNR, 0x0);//stop rtc
    rtc_out(OFF_RTLDR, rtc_time);
    rtc_out(OFF_RTCNR, DEFAULT_RTCNR);//start rtc
    up(&rtc_stp->sem);
    return sizeof(time_t);
}

static ssize_t rtc_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
    time_t rtc_time;
    if (down_interruptible(&rtc_stp->sem))
        return - ERESTARTSYS;
    rtc_time = rtc_in(OFF_RTCTR);
    if (copy_to_user(buf, &rtc_time, sizeof(time_t)))
    {
        printk("BSP: %s fail copy_to_user\n", __FUNCTION__);
        up(&rtc_stp->sem);
        return - EFAULT;
    }
    up(&rtc_stp->sem);
    return sizeof(time_t);
}

//------------------------------------------------------------------------------
// register module
//------------------------------------------------------------------------------
static const struct file_operations rtc_fops = {
        .owner  = THIS_MODULE,
        .open   = rtc_open,
        .release= rtc_release,
        .read   = rtc_read,
        .write  = rtc_write,
};

static int __init rtc_init(void)
{
    int ret = 0, err = 0;
    dev_t devno;
    // register chrdev
    devno = MKDEV(MAJ_RTC, MIN_RTC);
    ret = register_chrdev_region(devno, 1, NAME_RTC);
    if (ret<0)
    {
        printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
        return ret;
    }
    // alloc dev
    rtc_stp = kmalloc(sizeof(struct rtc_st), GFP_KERNEL);
    if (!rtc_stp)
    {
        ret = - ENOMEM;
        goto fail_malloc;
    }
    memset(rtc_stp, 0, sizeof(struct rtc_st));
    init_MUTEX(&rtc_stp->sem);
    // add cdev
    cdev_init(&rtc_stp->cdev, &rtc_fops);
    rtc_stp->cdev.owner = THIS_MODULE;
    rtc_stp->cdev.ops = &rtc_fops;
    err = cdev_add(&rtc_stp->cdev, devno, 1);
    if (err)
    {
        printk("BSP: %s fail cdev_add\n", __FUNCTION__);
        goto fail_remap;
    }
    // ioremap
    rtc_stp->regp = ioremap(REG_BASE, REG_SIZE);
    if (rtc_stp->regp==NULL)
    {
        printk("BSP: %s fail ioremap\n", __FUNCTION__);
        goto fail_remap;
    }
    rtc_out(OFF_RTCNR, 0x0); //stop rtc
    rtc_out(OFF_RTLDR, 0x0); //must write a value, otherwise rtc RTCNR can not start the RTC
    rtc_out(OFF_RTPSR, DEFAULT_RTPSR);
    rtc_out(OFF_RTCNR, DEFAULT_RTCNR);//start rtc
    printk("NTS RTC Driver installed\n");
    return 0;

fail_remap:
    kfree(rtc_stp);

fail_malloc:
    unregister_chrdev_region(devno, 1);
    printk("Fail to install NTS RTC driver\n");
    return ret;
}

static void __exit rtc_exit(void)
{
    dev_t devno;
    iounmap(rtc_stp->regp);
    cdev_del(&rtc_stp->cdev);
    kfree(rtc_stp);
    devno = MKDEV(MAJ_RTC, MIN_RTC);
    unregister_chrdev_region(devno, 1);
    printk("NTS RTC Driver removed\n");
}

module_init(rtc_init);
module_exit(rtc_exit);

MODULE_AUTHOR("Ray.Zhou, <ray.zhou@nts-intl.com>");
MODULE_DESCRIPTION("NTS RTC");
MODULE_LICENSE("GPL");
