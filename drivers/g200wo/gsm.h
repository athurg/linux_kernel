#ifndef __LED_H__
#define __LED_H__

#include <asm/ioctl.h>

// command of ioctrl
#define CMD_GSM_PWR	_IOW(G200WO_IOCTL_MAGIC, 0x01, int)
#define CMD_GSM_ATT	_IOW(G200WO_IOCTL_MAGIC, 0x02, int)

// args of ioctl, the port num of the leds
#define ARG_GSM_PWR_ON		0
#define ARG_GSM_PWR_OFF		1

#endif // __LED_H__
