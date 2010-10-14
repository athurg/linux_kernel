#ifndef __LMK03000_H__
#define __LMK03000_H__

#include <asm/ioctl.h>
#include <g410sd/g410sd_hw.h>

#define LMK03000_IOC_SET_DATA		_IOW(G410SD_IOCTL_MAGIC, 0x01, int)
#define LMK03000_IOC_SET_SYNC		_IOW(G410SD_IOCTL_MAGIC, 0x02, int)
#define LMK03000_IOC_SET_GOE		_IOW(G410SD_IOCTL_MAGIC, 0x03, int)

#define LMK03000_IOC_GET_LD		_IOR(G410SD_IOCTL_MAGIC, 0x01, int)

#define ARGS_SYNC_ON	1
#define ARGS_SYNC_OFF	0

#define ARGS_GOE_OPEN	1
#define ARGS_GOE_CLOSE	0

#endif	// __LMK03000_H__
