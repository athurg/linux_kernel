#ifndef __STATUS_H__
#define __STATUS_H__

#include <asm/ioctl.h>
#include <x223ft/x223ft_hw.h>

#define CMD_GET_AGE_STATUS	_IOR(X223FT_IOCTL_MAGIC, 0x01, int)
#define CMD_GET_PA_BOARD_STATUS	_IOR(X223FT_IOCTL_MAGIC, 0x02, int)
#define CMD_GET_BDA_ID_STATUS	_IOR(X223FT_IOCTL_MAGIC, 0x03, int)

#endif // __STATUS_H__
