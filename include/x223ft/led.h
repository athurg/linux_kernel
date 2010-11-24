#ifndef __LED_H__
#define __LED_H__

#include <asm/ioctl.h>
#include <x223ft/x223ft_hw.h>

// command of ioctl
#define LED_IOC_ON	_IOW(X223FT_IOCTL_MAGIC, 0x01, int)
#define LED_IOC_OFF	_IOW(X223FT_IOCTL_MAGIC, 0x00, int)

// args of ioctl, the port num of the leds
#define ARG_LED_RUN		LED_RUN
#define ARG_LED_DPD		LED_DPD

#endif // __LED_H__
