/*

	kernel/driver/nts/cpld_test.c
*/
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#include <mach/platform.h>

#define MAJ_CPLD_TEST_DEV_NUM	10
#define MIN_CPLD_TEST_DEV_NUM	0
#define NTS_CPLD_TEST_ADDR	EMC_CS2_BASE	//CPLD's Chip Select connect to EMC CS2
#define NTS_CPLD_TEST_SIZE	32
#define	NTS_CPLD_TEST_NAME	"nts cpld test module"

struct nts_cpld_test_st
{
    struct cdev cdev;
    volatile unsigned char __iomem *regp;
};

struct nts_cpld_test_st *nts_cpld_test_stp;


static ssize_t nts_cpld_test_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	unsigned int i=0;

	if(size > NTS_CPLD_TEST_SIZE){
		size = -1;
	}else{
		for(i=0; i < size; i++){
			__raw_writeb(buf[i], &nts_cpld_test_stp->regp[i]);
		}
	}
	return size;
}

static ssize_t nts_cpld_test_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	unsigned int i=0;

	if(size > NTS_CPLD_TEST_SIZE){
		size=-1;
	}else{
		for(i=0; i < size; i++){
			buf[i]=__raw_readb(&nts_cpld_test_stp->regp[i]);
		}
	}
	return size;
}



static const struct file_operations nts_cpld_test_fops = {
        .owner  = THIS_MODULE,
        .open   = NULL,
        .release= NULL,
        .read   = nts_cpld_test_read,
        .write  = nts_cpld_test_write,
        .ioctl  = NULL,
};

static int __init nts_cpld_test_init(void)
{
    int ret = 0, err = 0;
    dev_t devno;

    // register chrdev
    devno = MKDEV(MAJ_CPLD_TEST_DEV_NUM, MIN_CPLD_TEST_DEV_NUM);

    ret = register_chrdev_region(devno, 1, NTS_CPLD_TEST_NAME);
    if (ret<0){
        printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
        return ret;
    }

    // alloc dev
    nts_cpld_test_stp = kmalloc(sizeof(struct nts_cpld_test_st), GFP_KERNEL);
    if (!nts_cpld_test_stp){
        ret = - ENOMEM;
        goto fail_malloc;
    }

    memset(nts_cpld_test_stp, 0, sizeof(struct nts_cpld_test_st));
    
    // add cdev
    cdev_init(&nts_cpld_test_stp->cdev, &nts_cpld_test_fops);
    nts_cpld_test_stp->cdev.owner = THIS_MODULE;
    nts_cpld_test_stp->cdev.ops = &nts_cpld_test_fops;

    err = cdev_add(&nts_cpld_test_stp->cdev, devno, 1);
    if (err){
        printk("BSP: %s fail cdev_add\n", __FUNCTION__);
        goto fail_remap;
    }

    // ioremap
    nts_cpld_test_stp->regp = ioremap(NTS_CPLD_TEST_ADDR, NTS_CPLD_TEST_SIZE);
    if (nts_cpld_test_stp->regp==NULL){
        printk("BSP: %s fail ioremap\n", __FUNCTION__);
        goto fail_remap;
    }
    printk("NTS nts_cpld_test Driver installed\n");
    return 0;

fail_remap:
    kfree(nts_cpld_test_stp);

fail_malloc:
    unregister_chrdev_region(devno, 1);
    printk("Fail to install NTS nts_cpld_test driver\n");
    return ret;
}

static void __exit nts_cpld_test_exit(void)
{
    dev_t devno;

    iounmap(nts_cpld_test_stp->regp);
    cdev_del(&nts_cpld_test_stp->cdev);
    kfree(nts_cpld_test_stp);
    devno = MKDEV(MAJ_CPLD_TEST_DEV_NUM, MIN_CPLD_TEST_DEV_NUM);
    unregister_chrdev_region(devno, 1);
    printk("NTS CPLD TEST Modules removed\n");
}

module_init(nts_cpld_test_init);
module_exit(nts_cpld_test_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("athurg <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("Test module to check communication between CPLD and CPU");
