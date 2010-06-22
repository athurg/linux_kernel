#ifndef __LED_H__
#define __LED_H__

// command of ioctrl
#define CMD_LED_ON	1
#define CMD_LED_OFF	0

// args of ioctl, the port num of the leds
#define ARG_LED_RUN_OK		1
#define ARG_LED_RUN_ERR		4
#define ARG_LED_VSWR1		5
#define ARG_LED_VSWR2		11

#endif // __LED_H__
