#ifndef __ADF4350_H__
#define __ADF4350_H__

#include <asm/ioctl.h>
#include "g200wo_hw.h"

#define CMD_LO_TRXA_SET		_IOW(G200WO_IOCTL_MAGIC, 0x01, int)
#define CMD_LO_TXB_SET		_IOW(G200WO_IOCTL_MAGIC, 0x02, int)
#define CMD_LO_RXB_SET		_IOW(G200WO_IOCTL_MAGIC, 0x03, int)

#define CMD_LO_TRXA_GET_LD	_IOR(G200WO_IOCTL_MAGIC, 0x01, int)
#define CMD_LO_TXB_GET_LD	_IOR(G200WO_IOCTL_MAGIC, 0x02, int)
#define CMD_LO_RXB_GET_LD	_IOR(G200WO_IOCTL_MAGIC, 0x03, int)

#endif	// __ADF4350_H__
