#ifndef __RESET_H__
#define __RESET_H__

#define CMD_RESET_TYPE          MAGIC_RESET    //magic number
#define CMD_RESET_SET           _IO(CMD_RESET_TYPE, 1)
#define CMD_RESET_ENABLE        _IO(CMD_RESET_TYPE, 2)
#define CMD_RESET_DISABLE       _IO(CMD_RESET_TYPE, 3)

#define RESET_NONE              0x00000000
#define RESET_IF_FPGA           0x00000008
#define RESET_IR_FPGA           0x00000004
#define RESET_RX_ADC            0x00000002
#define RESET_TX_DAC            0x00000001


#define RESET_ALL               0xFFFFFFFF

#endif // __RESET_H__
