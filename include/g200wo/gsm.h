#ifndef __GSM_H__
#define __GSM_H__

#include <asm/ioctl.h>
#include <g200wo/g200wo_hw.h>

// command of ioctrl
#define CMD_GSM_PWR	_IOW(G200WO_IOCTL_MAGIC, 0x01, int)
#define CMD_GSM_VCHARGE	_IOW(G200WO_IOCTL_MAGIC, 0x02, int)
#define CMD_GSM_ATT	_IOW(G200WO_IOCTL_MAGIC, 0x03, int)

// args of ioctl, the port num of the leds
#define ARG_GSM_PWR_ON			0
#define ARG_GSM_PWR_OFF			1
#define ARG_GSM_VCHARGE_ON		1
#define ARG_GSM_VCHARGE_OFF		0

#endif // __GSM_H__
