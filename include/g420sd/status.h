#ifndef __STATUS_H__
#define __STATUS_H__

#include <asm/ioctl.h>
#include <g420sd/g420sd_hw.h>

#define CMD_GET_AGE_STATUS	_IOR(G420SD_IOCTL_MAGIC, 0x01, int)
#define CMD_GET_PA_BOARD_STATUS	_IOR(G420SD_IOCTL_MAGIC, 0x02, int)

#endif // __STATUS_H__
