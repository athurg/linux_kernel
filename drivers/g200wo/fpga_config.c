/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      File Name  : .c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010.06.24
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	None
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

#include "hardware.h"
#include "fpga_config.h"


#define FPGA_CFG_CTRL_DONE	(1<<5)
#define FPGA_CFG_CTRL_INT	(1<<4)

#define FPGA_CFG_CTRL_PROG	(1<<1)
#define FPGA_CFG_CTRL_CS	(1<<0)

#define FILE_BUF_LEN            (1024*64) //64k byte

//------------------------------------------------------------------------------
// structure
//------------------------------------------------------------------------------
struct fpga_cfg_st
{
	struct cdev cdev;
	struct semaphore sem;
	unsigned char data;
};

//------------------------------------------------------------------------------
// global
//------------------------------------------------------------------------------
struct fpga_cfg_st *fpga_cfg_stp;
char *cfile_data;
struct file *cfile_filp;
int wb_count;

//------------------------------------------------------------------------------
// io functions
//------------------------------------------------------------------------------
static inline void fpga_write_clock(int active)
{
	__raw_writeb((active ? 1:0), io_p2v(ADDR_FPGA_CFG_CLK));
}

static inline void fpga_write_data(char dat)
{
	__raw_writeb(dat, io_p2v(ADDR_FPGA_CFG_DAT));
}

static inline void fpga_write_ctrl(int port, int active)
{
	static unsigned char data;

	fpga_cfg_stp->data &= port;
	if(active)	fpga_cfg_stp->data |= port;

	__raw_writeb(fpga_cfg_stp->data, io_p2v(ADDR_FPGA_CFG_CTRL));
}


//------------------------------------------------------------------------------
// hardware functions
//------------------------------------------------------------------------------
int check_file(char *f)
{
	cfile_filp = filp_open(f, O_RDONLY, 0);

	if(IS_ERR(cfile_filp)){
		printk("BSP: '%s' not exist!\n", f);
		return ERR_FILE_EXIST;
	}

	if (cfile_filp->f_dentry->d_inode->i_size<1){
		printk("BSP: '%s' is empty!\n", f);
		filp_close(cfile_filp, NULL);
		return ERR_FILE_EXIST;
	}

	filp_close(cfile_filp, NULL);

	return 0;
}

int cfile_open(char *filename)
{
	struct inode *inode;
	off_t fsize;

	cfile_filp = filp_open(filename, O_RDONLY, 0);
	
	if(IS_ERR(cfile_filp)){
		//printk(KERN_ALERT "'%s' not exist!\n", filename);
		return -1;
	}

	inode = cfile_filp->f_dentry->d_inode;
	fsize = inode->i_size;
	printk("BSP: file size:%i\n", (unsigned int)fsize);
	return fsize;
}

int cfile_get(int len)
{
	int ret;
	mm_segment_t fs;

	fs = get_fs();
	set_fs(get_ds());
	ret = cfile_filp->f_op->read(cfile_filp, cfile_data, len, &(cfile_filp->f_pos));
	set_fs(fs);
	return ret;
}

int cfile_close(void)
{
	filp_close(cfile_filp, NULL);
	return 0;
}

void fpga_write(int len)
{
	int i;
	for (i=0; i<len; i++)
	{
		fpga_write_data(cfile_data[i]);
		fpga_write_clock(0);
		fpga_write_clock(1);
		wb_count++;
	}
	fpga_write_clock(0);
}

void fpga_startup(void)
{
	int i;
	unsigned char tmp;

	for (i=0; i<40000; i++){
		tmp = FPGA_CFG_CTRL_DONE & __raw_readb(io_p2v(ADDR_FPGA_CFG_CTRL));
		if (tmp)	break;
		fpga_write_clock(0);
		fpga_write_clock(1);
	}

	for (i=0; i<16; i++){
		fpga_write_clock(0);
		fpga_write_clock(1);
	}

	fpga_write_clock(0);
}

int fpga_do_config(char *file)
{
	int file_len, read_len, i=0;
	unsigned char tmp;

	cfile_data = (char *)kmalloc(FILE_BUF_LEN, GFP_ATOMIC);
	if (!cfile_data){
		printk("BSP: %s fail kmalloc\n", __FUNCTION__);
		return ERR_NOMEM;
	}
	
	//make CS & PROG inactive
	fpga_write_ctrl((FPGA_CFG_CTRL_CS | FPGA_CFG_CTRL_PROG), 0);
	fpga_write_ctrl(FPGA_CFG_CTRL_PROG, 1);
	udelay(250);
	fpga_write_ctrl(FPGA_CFG_CTRL_PROG, 0);
	mdelay(4);
	
	//Check INIT_B
	while(1){
		tmp = FPGA_CFG_CTRL_INT & __raw_readb(io_p2v(ADDR_FPGA_CFG_CTRL));
		if(tmp)	//OK
			break;
		else if(i<5){	//timeout
			i++;
			udelay(2000);
			continue;
		}else{	//fail
			printk("BSP: INIT_B is low! Hardware Fail!\n");
			return ERR_INIT_LOW;
		}
	}

	// open file
	file_len = cfile_open(file);
	wb_count = 0;
		
	// select fpga
	fpga_write_ctrl(FPGA_CFG_CTRL_CS, 1);
		
	// write fpga
	while (file_len>0){
		read_len = (file_len > FILE_BUF_LEN) ? FILE_BUF_LEN: file_len;
		cfile_get(read_len);
		fpga_write(read_len);

		file_len -= read_len;
	}
	
	//free memory
	cfile_close();
	kfree(cfile_data);

	// do some starup
	fpga_startup();

	// check done
	tmp = FPGA_CFG_CTRL_DONE & __raw_readb(io_p2v(ADDR_FPGA_CFG_CTRL));
	fpga_write_ctrl((FPGA_CFG_CTRL_CS | FPGA_CFG_CTRL_PROG), 0);

	if(tmp){
		printk("BSP: Configurating FPGA done!\n");
		return 0;
	}else{
		printk("BSP: Configurating FPGA timeout!\n");
		return ERR_TIMEOUT;
	}
}


static ssize_t fpga_cfg_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	char *file;
	char kbuf[MAX_FILENAME_LEN*3+8];
	int err, len;

	if (down_interruptible(&fpga_cfg_stp->sem))
		return - ERESTARTSYS;
	if (copy_from_user(kbuf, buf, size)){
		printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
		up(&fpga_cfg_stp->sem);
		return - EFAULT;
	}
	// split filename
	file = kbuf;
	err = 0;
	len = strlen(file);
	if ((len<1)||(len>MAX_FILENAME_LEN))
		err = ERR_FILE_NAME;
	printk("BSP: IR FILE %s\n", file);

	if (err<0){
		printk("BSP: Filename length ERR\n");
		up(&fpga_cfg_stp->sem);
		return err;
	}
	// check file exist
	err = check_file(file);
	if (err<0){
		printk("BSP: File ERR\n");
		up(&fpga_cfg_stp->sem);
		return err;
	}
	// Do FPGA configure
	err = fpga_do_config(file);
	if (err<0)
	{
		printk("BSP: Config ERR\n");
		up(&fpga_cfg_stp->sem);
		return err;
	}
	up(&fpga_cfg_stp->sem);
	return size;
}


//------------------------------------------------------------------------------
// register module
//------------------------------------------------------------------------------
static const struct file_operations fpga_cfg_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = NULL,
	.write  = fpga_cfg_write,
};

static int __init fpga_cfg_init(void)
{
	int ret = 0, err = 0;
	dev_t devno;
	// register chrdev
	devno = MKDEV(MAJ_FPGA_CFG, MIN_FPGA_CFG);
	ret = register_chrdev_region(devno, 1, "g200wo_fpga_cfg");
	if (ret<0)
	{
		printk("BSP: %s fail register_chrdev_region\n", __FUNCTION__);
		return ret;
	}
	// alloc dev
	fpga_cfg_stp = kmalloc(sizeof(struct fpga_cfg_st), GFP_KERNEL);
	if (!fpga_cfg_stp)
	{
		ret = - ENOMEM;
		goto fail_malloc;
	}
	memset(fpga_cfg_stp, 0, sizeof(struct fpga_cfg_st));
	init_MUTEX(&fpga_cfg_stp->sem);

	// add cdev
	cdev_init(&fpga_cfg_stp->cdev, &fpga_cfg_fops);
	fpga_cfg_stp->cdev.owner = THIS_MODULE;
	fpga_cfg_stp->cdev.ops = &fpga_cfg_fops;
	err = cdev_add(&fpga_cfg_stp->cdev, devno, 1);
	if (err)
	{
		printk("BSP: %s fail cdev_add\n", __FUNCTION__);
		goto fail_remap;
	}

	printk("NTS FPGA_CFG Driver installed\n");
	return 0;

fail_remap:
	kfree(fpga_cfg_stp);

fail_malloc:
	unregister_chrdev_region(devno, 1);
	printk("Fail to install NTS FPGA_CFG driver\n");
	return ret;
}

static void __exit fpga_cfg_exit(void)
{
	dev_t devno;

	cdev_del(&fpga_cfg_stp->cdev);
	kfree(fpga_cfg_stp);
	devno = MKDEV(MAJ_FPGA_CFG, MIN_FPGA_CFG);
	unregister_chrdev_region(devno, 1);
	printk("NTS FPGA_CFG Driver removed\n");
}

module_init(fpga_cfg_init);
module_exit(fpga_cfg_exit);

MODULE_AUTHOR("Ray.Zhou, <ray.zhou@nts-intl.com>");
MODULE_DESCRIPTION("NTS FPGA_CFG");
MODULE_LICENSE("GPL");
