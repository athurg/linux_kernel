#ifndef __ADDA_H__
#define __ADDA_H__

#define DEV_ADDA_A	0
#define DEV_ADDA_B	1

struct adda_elem
{
	unsigned char dev;
	unsigned char addr;
	unsigned char data;
	unsigned char pad;
};

#endif // __ADDA_H__
