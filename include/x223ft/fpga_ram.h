#ifndef __FPGA_RAM_H__
#define __FPGA_RAM_H__

#include <asm/ioctl.h>
#include <x223ft/x223ft_hw.h>

#define MAX_GROUP_LEN		4096 //MAX read and write bytes
#define FPGA_RAM_INVALID_ADDR	0xFFFF

struct fpga_ram_elem
{
	unsigned short reg_addrh;//if reg_addrh is FPGA_RAM_ADDR_INVALID, just skip reg_addrh
	unsigned short reg_addrl;
	unsigned short reg_datah;//if reg_datah is FPGA_RAM_ADDR_INVALID, just skip reg_datah
	unsigned short reg_datal;
	unsigned int group_len;
	void *buf;
};

#endif // __FPGA_RAM_H__

