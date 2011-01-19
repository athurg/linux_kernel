#ifndef __ADF4350_H__
#define __ADF4350_H__

#include <asm/ioctl.h>
#include <g420sd/g420sd_hw.h>

#define CMD_ADF4350_TX_SET		_IOW(G420SD_IOCTL_MAGIC, 0x01, int)
#define CMD_ADF4350_RXFB_SET		_IOW(G420SD_IOCTL_MAGIC, 0x02, int)

#define CMD_ADF4350_TX_GET_LD		_IOR(G420SD_IOCTL_MAGIC, 0x01, int)
#define CMD_ADF4350_RXFB_GET_LD		_IOR(G420SD_IOCTL_MAGIC, 0x02, int)

#endif	// __ADF4350_H__
