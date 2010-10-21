/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : Ray.Zhou
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      FileName   : fpga_config.c
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-10-21 16:34:44
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	2010-10-20	Change cfile to firmware
	2010-07-07	Change cdev to miscdevices
	None
*/
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>
#include <linux/firmware.h>

#include <asm/io.h>
#include <linux/delay.h>
#include <asm/uaccess.h>

#include <g200wo/g200wo_hw.h>
#include <g200wo/fpga_config.h>

static struct{
	struct miscdevice dev;
	struct semaphore sem;
	unsigned char data;
}fpga_cfg_st;

static inline void fpga_write_data(char dat)
{
	__raw_writeb(dat, FPGA_CFG_DAT_BASE);
	__raw_writeb(0, FPGA_CFG_CLK_BASE);
}

static inline void fpga_write_ctrl(int port, int active)
{
	fpga_cfg_st.data &= ~port;
	if(active)	fpga_cfg_st.data |= port;

	__raw_writeb(fpga_cfg_st.data, FPGA_CFG_CTRL_BASE);
}

static void fpga_startup(void)
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

static int fpga_init_check(void)
{
	unsigned int i=0;
	// INIT Check
	while(1){
		if(FPGA_CFG_CTRL_INT & __raw_readb(FPGA_CFG_CTRL_BASE)){
			break;
		}

		if(i > INIT_CHECK_MAX_TIME){	//timeout
			printk("BSP: INIT_B is low! Hardware Fail!\n");
			return -1;
		}

		i++;
		udelay(2000);
	}

	return 0;
}


/*
	Real configuration FPGA

	PROG set ==> INIT_B check ==> Write Data ==> check startup => check DONE
*/
static int fpga_load_firmware(const struct firmware *firmware)
{
	ssize_t i=0;
	unsigned char ret;

	if ( (firmware=NULL) || (firmware->data==NULL) || (firmware->size==0)) {
		return -1;
	}

	// clear all pins
	fpga_write_ctrl((FPGA_CFG_CTRL_CS | FPGA_CFG_CTRL_PROG), 0);

	// PROG Setting
	fpga_write_ctrl(FPGA_CFG_CTRL_PROG, 1);
	udelay(250);
	fpga_write_ctrl(FPGA_CFG_CTRL_PROG, 0);
	mdelay(4);

	ret = fpga_init_check();
	if (!ret){
		printk("BSP: INIT_B is low, hardware failed!\n");
		return -1;
	}

	// select fpga and then write
	fpga_write_ctrl(FPGA_CFG_CTRL_CS, 1);
	udelay(200);

	for (i=0; i<firmware->size; i++){
		fpga_write_data(firmware->data[i]);
	}

	// do some starup
	fpga_startup();

	// check done
	ret = FPGA_CFG_CTRL_DONE & __raw_readb(FPGA_CFG_CTRL_BASE);
	fpga_write_ctrl((FPGA_CFG_CTRL_CS | FPGA_CFG_CTRL_PROG), 0);

	if (ret) {
		printk("BSP: Configurating FPGA done!\n");
		return 0;
	} else {
		printk("BSP: Configurating FPGA timeout!\n");
		return ERR_TIMEOUT;
	}
}


static ssize_t fpga_load_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	char fw_name[FILENAME_MAX_LEN+8]="../../";
	const struct firmware *firmware = NULL;
	int rtn;

	if (down_interruptible(&fpga_cfg_st.sem))
		return - ERESTARTSYS;

	// Got firmware filename
	rtn = copy_from_user(fw_name+5, buf, size);
	if(rtn){
		printk("BSP: %s fail copy_from_user\n", __FUNCTION__);
		up(&fpga_cfg_st.sem);
		return - EFAULT;
	}

	// Do FPGA configure
	request_firmware(&firmware, fw_name, fpga_cfg_st.dev.this_device);
	if(firmware == NULL || firmware->data == NULL || firmware->size==0)
		rtn = -1;
	else {
		rtn = fpga_load_firmware(firmware);
		release_firmware(firmware);
	}

	up(&fpga_cfg_st.sem);

	return rtn;
}


static const struct file_operations fpga_cfg_fops = {
	.owner  = THIS_MODULE,
	.open   = NULL,
	.release= NULL,
	.read   = NULL,
	.write  = fpga_load_write,
};

static int __init fpga_load_init(void)
{
	int ret = 0;

	// Initial structure
	memset(&fpga_cfg_st, 0, sizeof(fpga_cfg_st));

	init_MUTEX(&fpga_cfg_st.sem);

	fpga_cfg_st.dev.minor = MISC_DYNAMIC_MINOR;
	fpga_cfg_st.dev.name = "g200wo_fpga_load";
	fpga_cfg_st.dev.fops = &fpga_cfg_fops;

	// register device
	ret = misc_register(&fpga_cfg_st.dev);
	if (ret)
		printk("BSP: %s fail register device\n", __FUNCTION__);
	else
		printk("BSP: FPGA_LOAD Driver installed\n");

	return ret;
}

static void __exit fpga_load_exit(void)
{
	misc_deregister(&fpga_cfg_st.dev);
	printk("BSP: G200WO FPGA_LOAD Driver removed\n");
}

module_init(fpga_load_init);
module_exit(fpga_load_exit);

MODULE_AUTHOR("Athurg.Feng, <athurg.feng@nts-intl.com>");
MODULE_DESCRIPTION("G200WO FPGA_LOAD");
MODULE_LICENSE("GPL");
