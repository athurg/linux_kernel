#ifndef __STATUS_H__
#define __STATUS_H__

#include <asm/ioctl.h>

#define CMD_STATUS_TYPE         MAGIC_STATUS    //magic number
#define CMD_STATUS_OLD          _IO(CMD_STATUS_TYPE, 1)
#define CMD_STATUS_PA           _IO(CMD_STATUS_TYPE, 2)
#define CMD_STATUS_DRY1         _IO(CMD_STATUS_TYPE, 3)
#define CMD_STATUS_DRY2         _IO(CMD_STATUS_TYPE, 4)
#define CMD_STATUS_DRY3         _IO(CMD_STATUS_TYPE, 5)

#endif // __STATUS_H__
