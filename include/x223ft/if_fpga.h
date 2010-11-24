#ifndef __IF_FPGA_H__
#define __IF_FPGA_H__

#include <asm/ioctl.h>
#include <x223ft/x223ft_hw.h>

#define SIG_IF_AGC			39
#define MAX_IF_FPGA_LEN			(32*1024*2) //MAX read and write bytes

#define CMD_IF_FPGA_READ_WORD		_IOR(X223FT_IOCTL_MAGIC, 0x01, int)
#define CMD_IF_FPGA_WRITE_WORD		_IOW(X223FT_IOCTL_MAGIC, 0x01, int)
#define CMD_IF_SET_PID			_IOW(X223FT_IOCTL_MAGIC, 0x02, int)

//enum if_fpga_reg_type{normal,fifo,cfr,dpd};
enum if_fpga_reg_type{normal, fifo,cfr_a, cfr_b, dpd_a, dpd_b};

struct if_fpga_elem
{
	unsigned short addr;
	unsigned short wlen;
	enum if_fpga_reg_type type;
	void *buf;
};

#endif // __IF_FPGA_H__

