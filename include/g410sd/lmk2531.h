#ifndef __LMK2531_H__
#define __LMK2531_H__

#include <asm/ioctl.h>
#include <g410sd/g410sd_hw.h>

#define CMD_TRX_LO_SET		_IOW(G410SD_IOCTL_MAGIC, 0x01, int)
#define CMD_DET_LO_SET		_IOW(G410SD_IOCTL_MAGIC, 0x02, int)

#define CMD_TRX_LO_GET_LD	_IOR(G410SD_IOCTL_MAGIC, 0x03, int)
#define CMD_DET_LO_GET_LD	_IOR(G410SD_IOCTL_MAGIC, 0x04, int)

#endif	// __LMK2531_H__
