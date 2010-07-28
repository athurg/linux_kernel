#ifndef __LED_H__
#define __LED_H__

#include <asm/ioctl.h>
#include <g200wo/g200wo_hw.h>

// command of ioctl
#define CMD_LED_ON	_IOW(G200WO_IOCTL_MAGIC, 0x01, int)
#define CMD_LED_OFF	_IOW(G200WO_IOCTL_MAGIC, 0x00, int)

// args of ioctl, the port num of the leds
#define ARG_LED_RUN		LED_RUN
#define ARG_LED_ALM		LED_ALM
#define ARG_LED_VR1		LED_VR1
#define ARG_LED_VR2		LED_VR2

#endif // __LED_H__
