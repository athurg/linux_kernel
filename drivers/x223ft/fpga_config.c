/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Athurg.Feng
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G410SD
 ::  ::  ::       ::           :::      FileName   : fpga_config.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-11-12 15:16:09
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	2010-07-07	Change cdev to miscdevices
	None
*/
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>

#include <asm/io.h>
#include <linux/delay.h>
#include <asm/uaccess.h>

#include <x223ft/x223ft_hw.h>
#include <x223ft/fpga_config.h>

struct{
	struct miscdevice dev;
	struct semaphore sem;
	unsigned char data;
}fpga_cfg_st;

char *cfile_data;
struct file *cfile_filp;

static inline void fpga_write_data(char dat)
{
	__raw_writeb(dat, FPGA_CFG_DAT_BASE);
	__raw_writeb(0,   FPGA_CFG_CLK_BASE);
}

static inline void fpga_write_ctrl(int port, int active)
{
	fpga_cfg_st.data &= ~port;
	if(active)	fpga_cfg_st.data |= port;

	__raw_writeb(fpga_cfg_st.data, FPGA_CFG_CTRL_BASE);
}


int cfile_open(char *filename)
{
	off_t fsize;

	cfile_filp = filp_open(filename, O_RDONLY, 0);

	if(IS_ERR(cfile_filp)){
		printk("BSP: '%s' not exist!\n", filename);
		return ERR_FILE_EXIST;
	}

	fsize = cfile_filp->f_dentry->d_inode->i_size;
	if(fsize < 1){
		printk("BSP: '%s' is empty!\n", filename);
		filp_close(cfile_filp, NULL);
		return ERR_FILE_EMPTY;
	}

	printk("BSP: file size:%i\n", (unsigned int)fsize);
	return fsize;
}

int cfile_read(unsigned int len)
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

void fpga_write(unsigned int len)
{
	unsigned int i;
	for (i=0; i<len; i++)
	{
		fpga_write_data(cfile_data[i]);
	}
}

void fpga_startup(void)
{
	int i;
	unsigned char tmp;

	for (i=0; i<40000; i++){
		tmp = FPGA_CFG_CTRL_DONE & __raw_readb(FPGA_CFG_CTRL_BASE);
		if (tmp)	break;
		//write dummy data to generate clock
		fpga_write_data(0xFF); // DIN be all high (refer to ug380 p81)
	}

	for (i=0; i<16; i++){
		//write dummy data to generate clock
		fpga_write_data(0xFF);
	}

}

/*
	Real configuration FPGA

step1:	open cfile, request memory and clear all pins

step2:	PROG set
	First ACTIVE more than 250us, Then delay more than 4ms

step3:	INIT_B check
	init should be valid to ensure fpga have initial everything

step4:	get and write data

step5:	check startup
step6:	check DONE
step7:	free memory
*/
int fpga_do_config(char *file)
{
	int file_len, read_len, i=0;
	unsigned char tmp;

	// open file
	file_len = cfile_open(file);
	if(file_len < 0){
		printk("BSP: FileName length ERR\n");
		return file_len;
	}

	// request buffer memory
	cfile_data = (char *)kmalloc(FILE_BUF_LEN, GFP_ATOMIC);
	if (!cfile_data){
		printk("BSP: %s fail kmalloc\n", __FUNCTION__);
		return ERR_NOMEM;
	}

	// clear all pins
	fpga_write_ctrl((FPGA_CFG_CTRL_CS | FPGA_CFG_CTRL_PROG), 0);

	// PROG Setting
	fpga_write_ctrl(FPGA_CFG_CTRL_PROG, 1);
	udelay(250);
	fpga_write_ctrl(FPGA_CFG_CTRL_PROG, 0);
	mdelay(4);

	// INIT Check
	while(1){
		if(FPGA_CFG_CTRL_INT & __raw_readb(FPGA_CFG_CTRL_BASE)){
			break;
		}

		if(i > INIT_CHECK_MAX_TIME){	//timeout
			printk("BSP: INIT_B is low! Hardware Fail!\n");
			kfree(cfile_data);
			return ERR_INIT_LOW;
		}

		i++;
		udelay(2000);
	}

	// select fpga and then write
	fpga_write_ctrl(FPGA_CFG_CTRL_CS, 1);
	udelay(200);
	while(file_len>0){
		read_len = min(FILE_BUF_LEN,file_len);

		cfile_read(read_len);
		fpga_write(read_len);

		file_len -= read_len;
	}

	// do some starup
	fpga_startup();

	// check done
	tmp = FPGA_CFG_CTRL_DONE & __raw_readb(FPGA_CFG_CTRL_BASE);
	fpga_write_ctrl((FPGA_CFG_CTRL_CS | FPGA_CFG_CTRL_PROG), 0);

	//free memory
	kfree(cfile_data);
	cfile_close();

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
	char file[FILENAME_MAX_LEN+8];
	int rtn;

	if (down_interruptible(&fpga_cfg_st.sem))
		return - ERESTARTSYS;

	// check filename length
	if((size<1) || (size > FILENAME_MAX_LEN)){
		up(&fpga_cfg_st.sem);
		return ERR_FILE_NAME;
	}

	rtn = copy_from_user(file, buf, size);
	if(rtn){
		printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
		up(&fpga_cfg_st.sem);
		return - EFAULT;
	}

	// Do FPGA configure
	rtn = fpga_do_config(file);
	if (rtn) {
		printk("BSP: Config ERR\n");
		up(&fpga_cfg_st.sem);
		return rtn;
	}

	up(&fpga_cfg_st.sem);

	return size;
}


static const struct file_operations fpga_cfg_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = NULL,
	.write  = fpga_cfg_write,
};

static int __init fpga_cfg_init(void)
{
	int ret = 0;

	// Initial structure
	memset(&fpga_cfg_st, 0, sizeof(fpga_cfg_st));

	init_MUTEX(&fpga_cfg_st.sem);

	fpga_cfg_st.dev.minor = MISC_DYNAMIC_MINOR;
	fpga_cfg_st.dev.name = "g410sd_fpga_config";
	fpga_cfg_st.dev.fops = &fpga_cfg_fops;

	// register device
	ret = misc_register(&fpga_cfg_st.dev);
	if (ret)
		printk("BSP: %s fail register device\n", __FUNCTION__);
	else
		printk("BSP: FPGA CONFIG Driver installed\n");

	return ret;
}

static void __exit fpga_cfg_exit(void)
{
	misc_deregister(&fpga_cfg_st.dev);
	printk("BSP: FPGA CONFIG Driver removed\n");
}

module_init(fpga_cfg_init);
module_exit(fpga_cfg_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("FPGA_CFG");
MODULE_LICENSE("GPL");
