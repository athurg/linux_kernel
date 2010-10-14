#ifndef __LED_H__
#define __LED_H__

#include <asm/ioctl.h>
#include <g410sd/g410sd_hw.h>

// command of ioctl
#define LED_IOC_ON	_IOW(G410SD_IOCTL_MAGIC, 0x01, int)
#define LED_IOC_OFF	_IOW(G410SD_IOCTL_MAGIC, 0x00, int)

// args of ioctl, the port num of the leds
#define ARG_LED_RUN		LED_RUN
#define ARG_LED_ALM		LED_ALM
#define ARG_LED_VR1		LED_VR1
#define ARG_LED_VR2		LED_VR2

#endif // __LED_H__
