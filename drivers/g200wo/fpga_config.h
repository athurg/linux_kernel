#ifndef __FPGA_CFG_H__
#define __FPGA_CFG_H__

#define FILENAME_MAX_LEN	50
#define FILE_BUF_LEN            (1024*64) //64k byte
#define INIT_CHECK_MAX_TIME	5	//init_b valid check timeout counter

#define ERR_FILE_NAME	-1
#define ERR_FILE_EXIST	-3
#define ERR_TIMEOUT	-5
#define ERR_FILE_DEV	-7
#define ERR_INIT_LOW	-8
#define ERR_NOMEM	-9
#define ERR_FILE_EMPTY	-11

#endif // __FPGA_CFG_H__
