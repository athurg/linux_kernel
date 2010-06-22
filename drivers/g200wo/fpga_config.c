//------------------------------------------------------------------------------
// fpga_cfg.c
// fpga selecedmap configuration drvier
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
#include "fpga_config.h"


#define FPGA_CFG_CTRL_DONE     0x10
#define FPGA_CFG_CTRL_INT       0x08
#define FPGA_CFG_CTRL_PROGRAM   0x04
#define FPGA_CFG_CTRL_CS       0x01

#define FILE_BUF_LEN            (1024*64) //64k byte

//------------------------------------------------------------------------------
// structure
//------------------------------------------------------------------------------
struct fpga_cfg_st
{
	struct cdev cdev;
	struct semaphore sem;
	volatile unsigned char __iomem *regp;
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
static inline u8 fpga_cfg_in(unsigned int off)
{
	return __raw_readb(&fpga_cfg_stp->regp[off]);
}

static inline void fpga_cfg_out(unsigned int off, u8 val)
{
	__raw_writeb(val, &fpga_cfg_stp->regp[off]);
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
		fpga_cfg_out(OFFSET_FPGA_CFG_DATA, cfile_data[i]);
		fpga_cfg_out(OFFSET_FPGA_CFG_CLK, 0);
		fpga_cfg_out(OFFSET_FPGA_CFG_CLK, 1);
		wb_count++;
	}
	fpga_cfg_out(OFFSET_FPGA_CFG_CLK, 0);
}

void fpga_startup(unsigned char bit_done)
{
	int i;
	unsigned char tmp;
	for (i=0; i<40000; i++)
	{
		tmp = fpga_cfg_in(OFFSET_FPGA_CFG_CTRL);
		if (tmp&bit_done)
			break;
		fpga_cfg_out(OFFSET_FPGA_CFG_CLK, 0);
		fpga_cfg_out(OFFSET_FPGA_CFG_CLK, 1);
	}
	for (i=0; i<16; i++)
	{
		fpga_cfg_out(OFFSET_FPGA_CFG_CLK, 0);
		fpga_cfg_out(OFFSET_FPGA_CFG_CLK, 1);
	}
	fpga_cfg_out(OFFSET_FPGA_CFG_CLK, 0);
}

int fpga_config(char *file)
{
	int file_len, rest_len, read_len, i;
	unsigned char t;

	cfile_data = (char *)kmalloc(FILE_BUF_LEN, GFP_ATOMIC);
	if (!cfile_data)
	{
		printk("BSP: %s fail kmalloc\n", __FUNCTION__);
		return ERR_NOMEM;
	}
	
	fpga_cfg_out(OFFSET_FPGA_CFG_CTRL, 0x00);	// ---- deselect all ----
	fpga_cfg_out(OFFSET_FPGA_CFG_CTRL, FPGA_CFG_CTRL_PROGRAM);	//Set PROG_CS_B active
	udelay(250);
	fpga_cfg_out(OFFSET_FPGA_CFG_CTRL, 0x00);	//Set PROG_CS_B inactive
	mdelay(4);
	
	for (i=0; i<=5; i++){	//Check INIT_B
		t = fpga_cfg_in(OFFSET_FPGA_CFG_CTRL);
		if (t&FPGA_CFG_CTRL_INT)
			break;
		udelay(2000);
	}
	if (t&FPGA_CFG_CTRL_INT){ //INIT_B should be high
		// open file
		file_len = cfile_open(file);
		rest_len = file_len; // we have checked it yet.
		wb_count = 0;
		
		// select fpga
		fpga_cfg_out(OFFSET_FPGA_CFG_CTRL, FPGA_CFG_CTRL_CS);
		
		// write fpga
		while (rest_len>0){
			if (rest_len>FILE_BUF_LEN)
				read_len = FILE_BUF_LEN;
			else
				read_len = rest_len;
			cfile_get(read_len);
			fpga_write(read_len);
			rest_len -= read_len;
		}
		
		// do some starup
		fpga_startup(FPGA_CFG_CTRL_DONE);

		// check done
		t = fpga_cfg_in(OFFSET_FPGA_CFG_CTRL);
		if(t&FPGA_CFG_CTRL_DONE){
			printk("BSP: Configurating FPGA0 done!\n");
		}else{
			printk("BSP: Configurating FPGA0 timeout!\n");
			fpga_cfg_out(OFFSET_FPGA_CFG_CTRL, 0x00);
			cfile_close();
			return ERR_TIMEOUT;
		}

		cfile_close();
		// deselect all
		fpga_cfg_out(OFFSET_FPGA_CFG_CTRL, 0x00);
	}else{
		printk("BSP: INIT_B is low! Hardware Fail!\n");
		return ERR_INIT_LOW;
	}

	kfree(cfile_data);
	return 0;
}

//------------------------------------------------------------------------------
// module functions
//------------------------------------------------------------------------------
static int fpga_cfg_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int fpga_cfg_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t fpga_cfg_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	return 0;
}

static ssize_t fpga_cfg_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	char *file;
	char kbuf[MAX_FILENAME_LEN*3+8];
	int err, len;

	if (down_interruptible(&fpga_cfg_stp->sem))
		return - ERESTARTSYS;
	if (copy_from_user(kbuf, buf, size))
	{
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

	if (err<0)
	{
		printk("BSP: Filename length ERR\n");
		up(&fpga_cfg_stp->sem);
		return err;
	}
	// check file exist
	err = check_file(file);
	if (err<0)
	{
		printk("BSP: File ERR\n");
		up(&fpga_cfg_stp->sem);
		return err;
	}
	// config FPGA
	err = fpga_config(file);
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
	.open   = fpga_cfg_open,
	.release= fpga_cfg_release,
	.read   = fpga_cfg_read,
	.write  = fpga_cfg_write,
};

static int __init fpga_cfg_init(void)
{
	int ret = 0, err = 0;
	dev_t devno;
	// register chrdev
	devno = MKDEV(MAJ_FPGA_CFG, MIN_FPGA_CFG);
	ret = register_chrdev_region(devno, 1, NAME_FPGA_CFG);
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
	// ioremap
	fpga_cfg_stp->regp = ioremap(BASE_FPGA_CFG, CPLD_RMSIZE);
	if (fpga_cfg_stp->regp==NULL)
	{
		printk("BSP: %s fail ioremap\n", __FUNCTION__);
		goto fail_remap;
	}
	fpga_cfg_stp->regp[OFFSET_FPGA_CFG_DATA] = 0;
	fpga_cfg_stp->regp[OFFSET_FPGA_CFG_CLK]  = 0;
	fpga_cfg_stp->regp[OFFSET_FPGA_CFG_CTRL] = 0;
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
	iounmap(fpga_cfg_stp->regp);
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
