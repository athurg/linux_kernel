#ifndef __G200WO_HARDWARE_H__
#define __G200WO_HARDWARE_H__

/*
	Devie connect on LPC3250
*/
// TMP125
#define TEMP_SCLK	GPO_16
#define TEMP_CS_N	GPO_18
#define TEMP_DOUT	GPI_22


// SPI
//SPI1
#define LPC_SPI1_SSEL	GPIO_05

/****************************************
 *	Devie connect on CPLD
 ***************************************/
#define CPLD_BASE	0xE2000000	//size 0x01000000
#define CPLD_RMSIZE	0x08	//size of each device connect on CPLD could remap

//VERSION MODULE
#define VERSION_BASE		(CPLD_BASE+0x00)
#define OFFSET_HARD_VER 0x00
#define OFFSET_CPLD_VER 0x01
#define OFFSET_UBOOT_VER 0x02

#define STATUS_BASE		(CPLD_BASE+0x04)
#define OFFSET_STATUS 0x00
#define OFFSET_DETECT 0x01

//FPGA CONFIG
#define BASE_FPGA_CFG		(CPLD_BASE+0x020)
#define OFFSET_FPGA_CFG_DATA	0x00
#define OFFSET_FPGA_CFG_CTRL	0x01
#define OFFSET_FPGA_CFG_CLK	0x02

// LMK03000
#define LMK03000_BASE		(CPLD_BASE+0x0C)
#define OFFSET_LMK03000_DATA	0x00
#define OFFSET_LMK03000_SYNC	0x01
#define OFFSET_LMK03000_LD	0x02
#define OFFSET_LMK03000_GOE	0x03

// ADF4350
#define ADF4350_BASE		(CPLD_BASE+0x10)
#define OFFSET_ADF4350_TXA_DATA	0x00
#define OFFSET_ADF4350_TXA_LD	0x01
#define OFFSET_ADF4350_RXA_DATA	0x02
#define OFFSET_ADF4350_RXA_LD	0x03
#define OFFSET_ADF4350_B_DATA	0x04
#define OFFSET_ADF4350_B_LD	0x05



#define POWER_BASE

// TRX
#define TRXA_LO_BASE
#define TXB_BASE
#define RXB_BASE
#define OFFSET_LD
#define OFFSET_DATA
#define OFFSET_CLK
#define OFFSET_LE

//ADC
#define ADCB_BASE
#define ADCA_BASE
#define OFFSET_SDOUT
#define OFFSET_RESET
#define OFFSET_SCLK
#define OFFSET_SDATA
#define OFFSET_SEN

//DAC
#define DACA_BASE
#define DACB_BASE
#define OFFSET_RESET_N
#define OFFSET_SDEN_N
#define OFFSET_SCLK
#define OFFSET_SDIO



/* Device's MAJ & MIN dev_num*/

#define MAJ_VERSION	220
#define MAJ_STATUS	221
#define MAJ_RESET	222
#define MAJ_TMP125	223
#define MAJ_RX_ADC	229
#define MAJ_TX_DAC	230
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
#define MIN_GSM		0
#define MIN_FPGA_CFG	0
#define MIN_VERSION	0
#define MIN_STATUS	0

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

