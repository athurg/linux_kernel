#ifndef __STATUS_H__
#define __STATUS_H__

#include <asm/ioctl.h>

#define CMD_STATUS_TYPE	MAGIC_STATUS    //magic number
#define CMD_STATUS_AGE	_IO(CMD_STATUS_TYPE, 1)
#define CMD_STATUS_PA	_IO(CMD_STATUS_TYPE, 2)

#endif // __STATUS_H__
