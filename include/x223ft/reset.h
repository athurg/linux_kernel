#ifndef __RESET_H__
#define __RESET_H__

#include <asm/ioctl.h>
#include <x223ft/x223ft_hw.h>

#define CMD_RESET	_IOW(X223FT_IOCTL_MAGIC, 0x01, int)

#define RESET_NONE	0x00
#define RESET_FPGA	0x08
#define RESET_ADC	0x04
#define RESET_DACA	0x02
#define RESET_ALL	0xFF
#define RESET_WDG	0xA5

#endif // __RESET_H__
