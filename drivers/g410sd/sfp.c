/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Athurg.Feng
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : 
 ::  ::  ::       ::           :::      FileName   : sfp.c
 ::   :: ::       ::             ::     Generate   : 
 ::    ::::       ::       ::      ::   Update     : 2010-09-25 14:46:14
::::    :::     ::::::      ::::::::    Version    : 0.0.1

Description
None
*/

/* IIC数据线释放检测
 * 		IIC时钟由主机占用，数据线由双方共用，且通信过程以时钟驱动。
 * 	通信开始前，需保证主机能驱动数据线和控制线。当从机输出低时，无论主
 * 	机输出高或低，数据线都会被拉低。双方无法通信。从机输高或者高阻时，
 * 	数据线等于主机的输出。这是可以完成通信，如输出起始标志位。当从机捕
 * 	获到通信起始标志时，会自动复位。
 *
 * IIC通信起始、结束标志
 * 	    ___                    ___
 * 	SDA    \____......._______/
 * 	    ______             _______
 * 	SCL       \........___/
 *            /\                 /\
 *            ||                 ||
 *           起始               结束
 */
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include <g410sd/g410sd_hw.h>
#include <g410sd/sfp.h>

#define DEFAULT_FREQ    200000 //200k

struct{
	struct semaphore sem;
	struct miscdevice dev;
	unsigned char data;
}sfp_st;

//端口操作
void sfp_iowrite(unsigned int base, unsigned int port, unsigned char active)
{
	//clear the orig value
	sfp_st.data &= ~port;

	//active only when new value is 'ACTIVE'
	if(active)	sfp_st.data |= port;

	//update the hardware
	__raw_writeb(sfp_st.data, base);

	udelay(1000000/(2*DEFAULT_FREQ));
}

unsigned char sfp_ioread(unsigned int base, unsigned int port)
{
	return (port & __raw_readb(base));
}

//IIC协议实现
void iic_start(unsigned int base)
{
	sfp_iowrite(base, SFP_SCL_OUT | SFP_SDA_OUT , 1);
	sfp_iowrite(base, SFP_SDA_OUT, 0);
}

void iic_stop(unsigned int base)
{
	sfp_iowrite(base, SFP_SCL_OUT, 0);
	sfp_iowrite(base, SFP_SDA_OUT, 0);
	sfp_iowrite(base, SFP_SCL_OUT, 1);
	sfp_iowrite(base, SFP_SDA_OUT, 1);
}

void iic_send_ack(unsigned int base, unsigned char ack)
{
	sfp_iowrite(base, SFP_SCL_OUT, 0);
	sfp_iowrite(base, SFP_SDA_OUT, ack);
	sfp_iowrite(base, SFP_SCL_OUT, 1);
}

unsigned char iic_recv_ack(unsigned int base)
{
	sfp_iowrite(base, SFP_SCL_OUT, 0);
	sfp_iowrite(base, SFP_SDA_OUT, 1);
	sfp_iowrite(base, SFP_SCL_OUT, 1);

	return sfp_ioread(base, SFP_SDA_IN);
}

int iic_check(unsigned int base)
{
	int i;

	for (i=0; i<17; i++) {
		if(iic_recv_ack(base))
			return 0;
	}

	return -1;
} 

void iic_write_byte(unsigned int base, unsigned char data)
{
	int i;

	for (i=0; i<8; i++){
		sfp_iowrite(base, SFP_SCL_OUT, 0);
		sfp_iowrite(base, SFP_SDA_OUT, (data&0x80));
		sfp_iowrite(base, SFP_SCL_OUT, 1);
		data <<= 1;
	}
}

unsigned char iic_read_byte(unsigned int base)
{
	int i;
	unsigned char ret=0;

	for (i=0; i<8; i++){
		ret <<= 1;
		sfp_iowrite(base, SFP_SCL_OUT, 0);
		sfp_iowrite(base, SFP_SDA_OUT, 1);//主机写1上拉数据线
		sfp_iowrite(base, SFP_SCL_OUT, 1);
		ret |= sfp_ioread(base, SFP_SDA_IN);
	}

	return ret;
}

//AFCT5765内部EEPROM读写协议实现
unsigned int eeprom_read(unsigned int base, unsigned char blk_addr, unsigned char reg_addr)
{
	unsigned char ret = 0;

	//设置寄存器地址
	iic_start(base);

	iic_write_byte(base, blk_addr<<1);
	if (iic_recv_ack(base)!=0) {
		return -1;
	}

	iic_write_byte(base, reg_addr);
	if (iic_recv_ack(base)!=0) {
		return -1;
	}


	//读寄存器数据
	iic_start(base);

	iic_write_byte(base, (blk_addr<<1) | 0x01);
	if(iic_recv_ack(base)!=0){
		return -1;
	}

	ret = iic_read_byte(base);
	iic_send_ack(base,1);

	iic_stop(base);
	return ret;
}

unsigned int eeprom_write(unsigned int base, unsigned char blk_addr, unsigned char reg_addr, unsigned char data)
{
	unsigned char ret = 0;

	iic_start(base);

	//写块地址
	iic_write_byte(base, blk_addr<<1);
	if(iic_recv_ack(base)!=0){
		return -1;
	}

	//写寄存器地址
	iic_write_byte(base, reg_addr);
	if(iic_recv_ack(base)!=0){
		return -1;
	}

	//写寄存器数据
	iic_write_byte(base, data);
	if(iic_recv_ack(base)!=0){
		return -1;
	}

	iic_stop(base);

	return ret;
}

static int sfp_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned int base=0;
	unsigned char ret;
	if (down_interruptible(&sfp_st.sem))
		return - ERESTARTSYS;

	base = (arg==SFP_DEV0) ? SFP0_BASE : SFP1_BASE;

	ret = __raw_readb(base) & (SFP_TXDIS | SFP_TXF | SFP_RLOS | SFP_DETN);

	up(&sfp_st.sem);

	return ret;
}

static ssize_t sfp_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	struct sfp_elem elem;
	unsigned int base=0, ret=0;

	if (down_interruptible(&sfp_st.sem))
		return - ERESTARTSYS;

	if (copy_from_user(&elem, buf, sizeof(elem))){
		printk("BSP: %s failed while copy_from_user\n", __FUNCTION__);
		up(&sfp_st.sem);
		return  - EFAULT;
	}

	base = (elem.dev == SFP_DEV0) ? SFP0_BASE : SFP1_BASE;

	if(iic_check(base)<0){
		printk("BSP: ERROR, Device IIC bus locked\n");
		up(&sfp_st.sem);
		return - ERESTARTSYS;
	}

	ret = eeprom_write(base, elem.block, elem.addr, elem.data);

	if (ret !=0 ){
		printk("BSP: ERROR, Devie Read/Write failed, loose slaver's ack\n");
	}

	up(&sfp_st.sem);

	return ret;
}

static ssize_t sfp_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	struct sfp_elem elem;
	unsigned int base=0, ret=0;

	if (down_interruptible(&sfp_st.sem))
		return - ERESTARTSYS;

	if (copy_from_user(&elem, buf, sizeof(elem))){
		printk("BSP: %s failed while copy_from_user\n", __FUNCTION__);
		up(&sfp_st.sem);
		return  - EFAULT;
	}

	base = (elem.dev == SFP_DEV0) ? SFP0_BASE : SFP1_BASE;

	if(iic_check(base)<0){
		printk("BSP: ERROR,Device IIC bus locked!\n");
		up(&sfp_st.sem);
		return - ERESTARTSYS;
	}

	elem.data = eeprom_read(base, elem.block, elem.addr);

	if (copy_to_user(buf, &elem, sizeof(elem))){
		printk("BSP: %s failed while copy_to_user\n", __FUNCTION__);
		up(&sfp_st.sem);
		return  - EFAULT;
	}

	up(&sfp_st.sem);
	return ret;
}

static const struct file_operations sfp_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = sfp_read,
	.write  = sfp_write,
	.ioctl  = sfp_ioctl,
};


static int __init sfp_init(void)
{
	int ret = 0;

	// 初始化设备结构体
	memset(&sfp_st, 0, sizeof(sfp_st));
	init_MUTEX(&sfp_st.sem);

	sfp_st.dev.minor = MISC_DYNAMIC_MINOR;
	sfp_st.dev.name = "g410sd_sfp";
	sfp_st.dev.fops = &sfp_fops;

	// register device
	ret = misc_register(&sfp_st.dev);

	if (ret)
		printk("BSP: %s failed while registe device\n", __FUNCTION__);
	else {
		printk("BSP: SFP Driver installed\n");

		// 初始化设备端口和状态
		sfp_iowrite(SFP0_BASE, (SFP_SCL_OUT | SFP_SDA_OUT), 1);
		sfp_iowrite(SFP1_BASE, (SFP_SCL_OUT | SFP_SDA_OUT), 1);
	}

	return ret;
}
static void __exit sfp_exit(void)
{
	printk("BSP: SFP Driver removed\n");
}

module_init(sfp_init);
module_exit(sfp_exit);

MODULE_AUTHOR("Athurg.Feng <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("SFP");
MODULE_LICENSE("GPL");
