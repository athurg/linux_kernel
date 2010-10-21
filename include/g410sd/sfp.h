#ifndef __SFP_H__
#define __SFP_H__

#include <asm/ioctl.h>
#include <g410sd/g410sd_hw.h>


#define SFP_DEV0	0
#define SFP_DEV1	1

struct sfp_elem{
	unsigned char dev;	//should be SFP_DEV0 or SFP_DEV1
	unsigned char block;	//Block addr, should be 0xA0 or 0xA2
	unsigned char addr;	//Register addr, refer datasheet
	unsigned char data;	//data to set or get
};

// command of ioctl
#define CMD_SFP_STATUS_READ	_IOR(G410SD_IOCTL_MAGIC, 0x00, int)

#endif // __SFP_H__
