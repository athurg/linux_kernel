#ifndef __G200WO_HARDWARE_H__
#define __G200WO_HARDWARE_H__

#include <mach/platform.h>

//Virtual Address of GPIO BASE
#define GPIO_IOBASE	io_p2v(GPIO_BASE)


/****************************************
	Devie connect on LPC3250
****************************************/
// TMP125
#define TEMP_SCLK	GPO_16
#define TEMP_CS_N	GPO_18
#define TEMP_DOUT	GPI_22

//GSM Module
#define GSM_PWRON_N	OUTP_STATE_GPO(13)
#define GSM_VCHARGE	OUTP_STATE_GPO(12)
#define GSM_ATT0	OUTP_STATE_GPO(6)
#define GSM_ATT1	OUTP_STATE_GPO(7)
#define GSM_ATT2	OUTP_STATE_GPO(8)
#define GSM_ATT3	OUTP_STATE_GPO(9)
#define GSM_ATT4	OUTP_STATE_GPO(10)

//LED
#define LED_RUN_OK	OUTP_STATE_GPO(1)
#define LED_RUN_ERR	OUTP_STATE_GPO(4)
#define LED_VSWR1	OUTP_STATE_GPO(5)
#define LED_VSWR2	OUTP_STATE_GPO(11)

// SPI
//SPI1
#define LPC_SPI1_SSEL	GPIO_05



/****************************************
 *	Devie connect on CPLD
 ***************************************/
#define CPLD_BASE	0xE2000000	//size 0x01000000
#define CPLD_RMSIZE	0x08		//size of each device connect on CPLD could remap

//Version Module
#define ADDR_HARD_VER		(CPLD_BASE + 0x00)
#define ADDR_CPLD_VER		(CPLD_BASE + 0x01)
#define ADDR_UBOOT_VER	(CPLD_BASE + 0x02)

//Hardware Watchdog Module
#define ADDR_WATCHDOG		(CPLD_BASE + 0x03)

//Status&Detect Module
#define ADDR_STATUS		(CPLD_BASE + 0x04)
#define ADDR_DETECT		(CPLD_BASE + 0x05)

//Reset Module
#define ADDR_RESET		(CPLD_BASE + 0x06)

//LMK03000 Module
#define ADDR_LMK03000		(CPLD_BASE + 0x07)

//Local Osc of A&B channel
#define ADDR_LO_TRXA		(CPLD_BASE + 0x08)
#define ADDR_LO_TXB		(CPLD_BASE + 0x09)
#define ADDR_LO_RXB		(CPLD_BASE + 0x0a)

#define ADDR_ADCA		(CPLD_BASE + 0x0b)
#define ADDR_ADCB		(CPLD_BASE + 0x0c)

#define ADDR_DACA		(CPLD_BASE + 0x0d)
#define ADDR_DACB		(CPLD_BASE + 0x0e)

//FPGA configure Module
#define ADDR_FPGA_CFG_DAT	(CPLD_BASE + 0x0f)
#define ADDR_FPGA_CFG_CTRL	(CPLD_BASE + 0x10)
#define ADDR_FPGA_CFG_CLK	(CPLD_BASE + 0x11)

//Power Module
#define ADDR_POWER_INT		(CPLD_BASE + 0x12)
#define ADDR_POWER_PEND		(CPLD_BASE + 0x13)
#define ADDR_POWER_STAT		(CPLD_BASE + 0x14)


/* Device's MAJ & MIN dev_num*/

#define MAJ_VERSION	220
#define MAJ_STATUS	221
#define MAJ_RESET	222
#define MAJ_TMP125	223
#define MAJ_WATCHDOG	224
#define MAJ_ADC		229
#define MAJ_DAC		230
#define MAJ_LED		231
#define MAJ_RTC		232
#define MAJ_POWER	233
#define MAJ_IF_FPGA	237
#define MAJ_EEPROM	239
#define MAJ_SPIROM	240
#define MAJ_LED		231
#define MAJ_GSM		239
#define MAJ_FPGA_CFG	227

#define MIN_LED		0
#define MIN_TMP125	0
#define MIN_ADC		0
#define MIN_DAC		0
#define MIN_WATCHDOG	0
#define MIN_GSM		0
#define MIN_FPGA_CFG	0
#define MIN_VERSION	0
#define MIN_STATUS	0
#define MIN_RESET	0
#define MIN_POWER	0
#define MIN_RTC		0

#define MAGIC_VERSION	0xE0
#define MAGIC_STATUS	0xE1
#define MAGIC_RESET	0xE2
#define MAGIC_FPGA_CFG	0xE7
#define MAGIC_RX_ADC	0xE9
#define MAGIC_TX_DAC	0xEA
#define MAGIC_LED	0xEB
#define MAGIC_RTC	0xEC
#define MAGIC_POWER	0xED
#define MAGIC_IF_FPGA	0xF1
#define MAGIC_EEPROM	0xF3
#define MAGIC_SPIROM	0xF4

#endif // __G200WO_HARDWARE_H__

