#ifndef __STATUS_H__
#define __STATUS_H__

#include <asm/ioctl.h>

#define CMD_GET_AGE_STATUS	_IOR(G200WO_IOCTL_MAGIC, 0x01, int)
#define CMD_GET_PA_STATUS	_IOR(G200WO_IOCTL_MAGIC, 0x02, int)

#endif // __STATUS_H__
