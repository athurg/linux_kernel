#ifndef __IF_FPGA_H__
#define __IF_FPGA_H__

#include <asm/ioctl.h>
#include <g200wo/g200wo_hw.h>

#define SIG_IF_AGC		39
#define MAX_IF_FPGA_LEN		(32*1024*2) //MAX read and write bytes

#define CMD_IF_FPGA_READ_WORD		_IOR(G200WO_IOCTL_MAGIC, 0x01, int)
#define CMD_IF_FPGA_WRITE_WORD		_IOW(G200WO_IOCTL_MAGIC, 0x01, int)
#define CMD_IF_SET_PID			_IOW(G200WO_IOCTL_MAGIC, 0x02, int)

#define ADDR_CFR_A	0x100
#define ADDR_CFR_B	0x200
#define ADDR_DPD	0x300

#define TYPE_IF_FPGA_NORMAL	0
#define TYPE_IF_FPGA_FIFO	1
#define TYPE_IF_FPGA_CFRA	2
#define TYPE_IF_FPGA_CFRB	3
#define TYPE_IF_FPGA_DPD	4

struct if_fpga_elem
{
    unsigned short addr;
    unsigned short wlen;
    unsigned int type;
    void *buf;
};

#endif // __IF_FPGA_H__

