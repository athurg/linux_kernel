#ifndef __ADC_H__
#define __ADC_H__

#define DEV_ADC_A	0
#define DEV_ADC_B	1


struct adc_elem
{
	unsigned char dev;
	unsigned char addr;
	unsigned char data;
	unsigned char pad;
};

#endif // __ADC_H__
