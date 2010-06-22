#ifndef __LMK03000_H__
#define __LMK03000_H__

#define CMD_LMK03000_DATA	0x00	//写数据，32位
#define CMD_LMK03000_SYNC	0x01	//写SYNC开关，argv=0/1
#define CMD_LMK03000_LD		0x02	//读LD状态，1锁定、0未锁定
#define CMD_LMK03000_GOE	0x03	//写GOE状态，1打开、0关闭

#define ARGS_LMK03000_SYNC_ON	1
#define ARGS_LMK03000_SYNC_OFF	0
#define ARGS_LMK03000_GOE_OPEN	1
#define ARGS_LMK03000_GOE_CLOSE	0

#endif	// __LMK03000_H__
