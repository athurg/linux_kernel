#ifndef __LED_H__
#define __LED_H__

#include <asm/ioctl.h>

// command of ioctl
#define CMD_LED_ON	_IOW(G200WO_IOCTL_MAGIC, 0x01, int)
#define CMD_LED_OFF	_IOW(G200WO_IOCTL_MAGIC, 0x00, int)

// args of ioctl, the port num of the leds
#define ARG_LED_RUN_OK		(1<<1)
#define ARG_LED_RUN_ERR		(1<<4)
#define ARG_LED_VSWR1		(1<<5)
#define ARG_LED_VSWR2		(1<<11)

#endif // __LED_H__
