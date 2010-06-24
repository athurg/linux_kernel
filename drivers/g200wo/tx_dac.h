#ifndef __TX_DAC_H__
#define __TX_DAC_H__

#define DEV_DAC_A	0
#define DEV_DAC_B	1

struct tx_dac_elem
{
	unsigned char dev;
	unsigned char addr;
	unsigned char data;
	unsigned char pad;
};

#endif // __TX_DAC_H__
