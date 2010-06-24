#ifndef __RESET_H__
#define __RESET_H__

#define CMD_RESET_TYPE          MAGIC_RESET    //magic number
#define CMD_RESET_SET           _IO(CMD_RESET_TYPE, 1)

#define RESET_NONE	0x00
#define RESET_IF_FPGA	0x10
#define RESET_RXA_ADC	0x08
#define RESET_TXA_DAC	0x04
#define RESET_RXB_ADC	0x02
#define RESET_TXB_DAC	0x01
#define RESET_ALL	0xFF

#endif // __RESET_H__
