#ifndef __IF_FPGA_H__
#define __IF_FPGA_H__

#include <asm/ioctl.h>

#define SIG_IF_AGC                  39

#define CMD_IF_FPGA_TYPE            MAGIC_IF_FPGA  //magic number
#define CMD_IF_FPGA_READ_WORD       _IO(CMD_IF_FPGA_TYPE, 1) //read a word
#define CMD_IF_FPGA_WRITE_WORD      _IO(CMD_IF_FPGA_TYPE, 2) //write a word
#define CMD_IF_SET_PID              _IO(CMD_IF_FPGA_TYPE, 3)

struct if_fpga_elem
{
    unsigned short addr;
    unsigned short wlen;
    unsigned int type;
    void *buf;
};

#define TPYE_IF_FPGA_NORMAL         0
#define TPYE_IF_FPGA_FIFO           1
#define TPYE_IF_FPGA_CFRA           2
#define TPYE_IF_FPGA_CFRB           3
#define TPYE_IF_FPGA_DPD            4

#define MAX_IF_FPGA_WLEN            (32*1024) //unit WORD
#define MAX_IF_FPGA_BLEN            (MAX_IF_FPGA_WLEN*2)

#endif // __IF_FPGA_H__
