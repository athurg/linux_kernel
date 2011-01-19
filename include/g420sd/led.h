#ifndef __LED_H__
#define __LED_H__

#include <asm/ioctl.h>
#include <g420sd/g420sd_hw.h>

// command of ioctl
#define CMD_LED_SET	_IOW(G420SD_IOCTL_MAGIC, 0x00, int)

// args of ioctl, the port num of the leds
#define  AGE_LED_RUN_OK            0 
#define  AGE_LED_RUN_ERR           1 
#define  AGE_LED_DPD_OK            2 
#define  AGE_LED_DPD_ERR           3 
#define  AGE_LED_VSWR_OK	   4
#define  AGE_LED_VSWR_ERR	   5
#define  AGE_LED_DPD_OFF           6

#endif // __LED_H__
