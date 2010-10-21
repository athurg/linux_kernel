#ifndef __STATUS_H__
#define __STATUS_H__

#include <asm/ioctl.h>
#include <g410sd/g410sd_hw.h>

#define STATUS_IOC_GET_AGE	_IOR(G410SD_IOCTL_MAGIC, 0x01, int)
#define STATUS_IOC_GET_PA_BOARD	_IOR(G410SD_IOCTL_MAGIC, 0x02, int)

#endif // __STATUS_H__
