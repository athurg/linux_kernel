/*
 * Driver for TMP435 temperature sensor
 *
 *	kernel/drivers/nts/tmp435.c
 *
 *
 * Author: AT <athurg.feng@nts-intl.com>
 *
 */
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/semaphore.h>

#include <mach/platform.h>

/*	TMP435 Pointer define		*/
//only read of write
#define TMP435_LTEMP_MSB		0x00
#define TMP435_LTEMP_LSB		0x15
#define TMP435_RTEMP_MSB		0x01
#define TMP435_RTEMP_LSB		0x10
#define TMP435_STATUS_REG		0x02
#define TMP435_RESET			0xFC

/*	TMP435 Address	*/
#define TMP435_ADDR_PREFIX	0x90
#define TMP435_READ	0x1
#define TMP435_WRITE	0x0
#define TMP435_ADDR	0x0

/*	Address and Register offset of IIC module in LPC32xx	*/
#define TMP435_BASE		I2C1_BASE
#define TMP435_MEM_SIZE		0x8000
#define I2C_TXRX	0x0000
#define I2C_STAT	0x0004
#define I2C_CTRL	0x0008
#define I2C_CLK_HI	0x000C
#define I2C_CLK_LO	0x0010
#define I2C_ADR		0x0014
#define I2C_RXFL	0x0018
#define I2C_TXFL	0x001C
#define I2C_RXB		0x0020
#define I2C_TXB		0x0024
#define I2C_STX		0x0028
#define I2C_STXFL	0x002C


#define MAJ_TMP435	223
#define MIN_TMP435	2

typedef struct{
	unsigned char local_msb;
	unsigned char local_lsb;
	unsigned char remote_msb;
	unsigned char remote_lsb;
}TMP435_RETURN_ST;

struct TMP435_ST{
	struct cdev cdev;
	struct semaphore sem;
	volatile unsigned char __iomem *regp;
};




struct TMP435_ST * tmp435_stp;
/*
	Init IIC Module of LPC32XX

Desc:
	set the register of IIC module of LPC32XX
*/
void tmp435_probe(void)
{
	//__raw_writeb(0x1,tmp435_stp->regp[I2C_CTRL]);
	;
}

unsigned char tmp435_raw_read(unsigned char pointer)
{
	unsigned char iic_slave_addr;

	iic_slave_addr = TMP435_ADDR_PREFIX | TMP435_ADDR;

	__raw_writeb(iic_slave_addr | TMP435_WRITE, &tmp435_stp->regp[I2C_TXB]);	//IIC Slave Address Byte
	__raw_writeb(pointer, &tmp435_stp->regp[I2C_TXB]);	//Pointer Register Byte (operation code)

	__raw_writeb(iic_slave_addr | TMP435_READ, &tmp435_stp->regp[I2C_TXB]);	//IIC Slave Address Byte again
	__raw_writeb(0, &tmp435_stp->regp[I2C_TXB]);	//dummy write to generate clock for reading

	return __raw_readb(&tmp435_stp->regp[I2C_RXB]);
}

void tmp435_raw_write(unsigned char pointer, unsigned char db)
{
	unsigned char iic_slave_addr;

	iic_slave_addr = TMP435_ADDR_PREFIX | TMP435_ADDR;

	__raw_writeb(iic_slave_addr | TMP435_WRITE, &tmp435_stp->regp[I2C_TXB]);	//IIC Slave Address Byte
	__raw_writeb(pointer, &tmp435_stp->regp[I2C_TXB]);	//Pointer Register Byte (operation code)

	__raw_writeb(iic_slave_addr | TMP435_WRITE, &tmp435_stp->regp[I2C_TXB]);	//IIC Slave Address Byte again
	__raw_writeb(db, &tmp435_stp->regp[I2C_TXB]);	//dummy write to generate clock for reading
}


static ssize_t tmp435_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	TMP435_RETURN_ST rtn;
	unsigned char addr;

	addr=TMP435_ADDR_PREFIX | TMP435_ADDR | TMP435_READ;

	if(down_interruptible(&tmp435_stp->sem)){
		return - ERESTARTSYS;
	}

	rtn.local_msb  = tmp435_raw_read(TMP435_LTEMP_MSB);
	rtn.local_lsb  = tmp435_raw_read(TMP435_LTEMP_LSB);
	rtn.remote_msb = tmp435_raw_read(TMP435_RTEMP_MSB);
	rtn.remote_lsb = tmp435_raw_read(TMP435_RTEMP_LSB);

	if(copy_to_user(&rtn, buf, sizeof(rtn))){
		printk("BSP:TMP435 return temperature failed\n");
	}else{
		printk("BSP:TMP435 return temperature successful\n");
	}

	up(&tmp435_stp->sem);
	return size;
}

static const struct file_operations tmp435_fops = {
        .owner  = THIS_MODULE,
        .open   = tmp435_probe,
        .release= NULL,
        .read   = tmp435_read,
        .write  = NULL,
};

/*
	Init TMP435 Driver
NOTE:
	We just initial TMP435 driver module
	when the device is probe, we do real initial CPU Modules in LPC32xx in iic_init() function
*/
static int __init tmp435_init(void)
{
	int ret = 0, err = 0;
	dev_t devno;

	// register chrdev
	devno = MKDEV(MAJ_TMP435, MIN_TMP435);
	ret = register_chrdev_region(devno, 1, "TMP435 temperature sensor");
	if (ret<0){
		printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
		return ret;
	}

	// alloc dev
	tmp435_stp = kmalloc(sizeof(tmp435_stp), GFP_KERNEL);
	if (!tmp435_stp){
		ret = - ENOMEM;
		goto fail_malloc;
	}
	memset(tmp435_stp, 0, sizeof(tmp435_stp));
	init_MUTEX(&tmp435_stp->sem);

	// add cdev
	cdev_init(&tmp435_stp->cdev, &tmp435_fops);
	tmp435_stp->cdev.owner = THIS_MODULE;
	tmp435_stp->cdev.ops = &tmp435_fops;
	err = cdev_add(&tmp435_stp->cdev, devno, 1);
	if (err){
		printk("BSP: %s fail cdev_add\n", __FUNCTION__);
		goto fail_remap;
	}

	// ioremap
	tmp435_stp->regp = ioremap(TMP435_BASE, TMP435_MEM_SIZE);
	if (tmp435_stp->regp==NULL){
		printk("BSP: %s fail ioremap\n", __FUNCTION__);
		goto fail_remap;
	}

	printk("NTS TMP435 Driver installed\n");
	return 0;

fail_remap:
	kfree(tmp435_stp);

fail_malloc:
	unregister_chrdev_region(devno, 1);
	printk("Fail to install NTS TMP435 driver\n");
	return ret;
}

static void __exit tmp435_exit(void)
{
	dev_t devno;

	iounmap(tmp435_stp->regp);
	cdev_del(&tmp435_stp->cdev);
	kfree(tmp435_stp);

	devno = MKDEV(MAJ_TMP435, MIN_TMP435);
	unregister_chrdev_region(devno, 1);
	
	printk("TMP435 temperator sensor removed...\n");
}

module_init(tmp435_init);
module_exit(tmp435_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Athurg <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("TMP435 temperature sensor module");
MODULE_ALIAS("platform:tmp435");
