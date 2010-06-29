#ifndef __G200WO_HARDWARE_H__
#define __G200WO_HARDWARE_H__

#include <mach/platform.h>
#include <mach/lpc32xx_gpio.h>	//GPIO Operate Define

/* ioctrl magic number define */
#define G200WO_IOCTL_MAGIC	'G'


/*
	interrupt number define

	irqs pin	GPIO
	=========================
	LPC_IRQ0_N => IRQ_GPIO_00
	LPC_IRQ1_N => IRQ_GPIO_01
	LPC_IRQ2_N => IRQ_GPI_03	(reserve)
	LPC_IRQ3_N => IRQ_GPI_07	(reserve)
*/

#define POWER_IRQ	IRQ_GPIO_00
#define IF_AGC_IRQ	IRQ_GPIO_01


/****************************************
 *	Devie connect on LPC3250
 ***************************************/
//Virtual Address of GPIO BASE
#define GPIO_IOBASE	io_p2v(GPIO_BASE)

//GSM Module
#define GSM_ATT0	_BIT(6)
#define GSM_ATT1	_BIT(7)
#define GSM_ATT2	_BIT(8)
#define GSM_ATT3	_BIT(9)
#define GSM_ATT4	_BIT(10)

#define GSM_VCHARGE	_BIT(12)
#define GSM_PWRON_N	_BIT(13)

// TMP125
#define TMP125_SCLK	_BIT(16)
#define TMP125_CS_N	_BIT(18)
#define TMP125_DOUT	_BIT(22)

// IF FPGA DPD & CFR Module
#define IF_FPGA_BASE	io_p2v(EMC_CS3_BASE)


/****************************************
 *	Devie connect on CPLD
 ***************************************/
#define CPLD_BASE		io_p2v(EMC_CS2_BASE)	//size is 0x01000000

//Version Module
#define HARD_VER_BASE		(CPLD_BASE + 0x00)
#define CPLD_VER_BASE		(CPLD_BASE + 0x01)
#define UBOOT_VER_BASE		(CPLD_BASE + 0x02)

//Hardware Watchdog Module
#define WATCHDOG_BASE		(CPLD_BASE + 0x03)

//Status&Detect Module
#define STATUS_BASE		(CPLD_BASE + 0x04)
#define DETECT_BASE		(CPLD_BASE + 0x05)

//Reset Module
#define RESET_BASE		(CPLD_BASE + 0x06)

//LMK03000 Module
#define LMK03000_BASE		(CPLD_BASE + 0x07)

//Local Osc of A&B channel
#define LO_TRXA_BASE		(CPLD_BASE + 0x08)
#define LO_TXB_BASE		(CPLD_BASE + 0x09)
#define LO_RXB_BASE		(CPLD_BASE + 0x0a)

#define ADCA_BASE		(CPLD_BASE + 0x0b)
#define ADCB_BASE		(CPLD_BASE + 0x0c)

#define DACA_BASE		(CPLD_BASE + 0x0d)
#define DACB_BASE		(CPLD_BASE + 0x0e)

//FPGA configure Module
#define FPGA_CFG_DAT_BASE	(CPLD_BASE + 0x0f)
#define FPGA_CFG_CTRL_BASE	(CPLD_BASE + 0x10)
#define FPGA_CFG_CLK_BASE	(CPLD_BASE + 0x11)

//Power Module
#define POWER_INT_BASE		(CPLD_BASE + 0x12)
#define POWER_PEND_BASE		(CPLD_BASE + 0x13)
#define POWER_STAT_BASE		(CPLD_BASE + 0x14)


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
#define MIN_IF_FPGA	0
#define MIN_VERSION	0
#define MIN_STATUS	0
#define MIN_RESET	0
#define MIN_POWER	0
#define MIN_RTC		0

#endif // __G200WO_HARDWARE_H__

