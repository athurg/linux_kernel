#ifndef __DAC_H__
#define __DAC_H__

#define DEV_DAC_A	0
#define DEV_DAC_B	1

struct dac_elem
{
	unsigned char dev;
	unsigned char addr;
	unsigned char data;
	unsigned char pad;
};

#endif // __DAC_H__
