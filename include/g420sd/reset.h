#ifndef __RESET_H__
#define __RESET_H__

#include <asm/ioctl.h>
#include <g420sd/g420sd_hw.h>

#define CMD_RESET	_IOW(G420SD_IOCTL_MAGIC, 0x01, int)

#define RESET_NONE	0x00
#define RESET_FPGA	0x20
#define RESET_ADC	0x10
#define RESET_DACD	0x08
#define RESET_DACC	0x04
#define RESET_DACB	0x02
#define RESET_DACA	0x01
#define RESET_ALL	0xFF
#define RESET_WDG	0xA5

#endif // __RESET_H__
