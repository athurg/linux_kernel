/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : 
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G200WO
 ::  ::  ::       ::           :::      FileName   : g200wo_hw.h
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-08-18 16:44:31
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	None
*/

#ifndef __G200WO_HARDWARE_H__
#define __G200WO_HARDWARE_H__

#ifdef __KERNEL__
#include <mach/platform.h>
#include <mach/lpc32xx_gpio.h>
#endif

#ifndef _BIT
#define _BIT(n)	(1<<n)
#endif

/* ioctrl magic number define */
#define G200WO_IOCTL_MAGIC	'G'
#define G200WO_PROJECT

/*
 * interrupt number define
 */

#define POWER_IRQ	IRQ_GPIO_00
#define IF_AGC_IRQ	IRQ_GPIO_01
//#define RESERVE	IRQ_GPI_03
//#define RESERVE	IRQ_GPI_07


/*
 * Devie connect on LPC3250
 *
 */
//Virtual Address of GPIO BASE
#define GPIO_IOBASE	io_p2v(GPIO_BASE)

//RTC Module
#define RTC_IOBASE	io_p2v(RTC_BASE)

//LED
#define LED_RUN		_BIT(1)
#define LED_ALM		_BIT(4)
#define LED_VR1		_BIT(5)
#define LED_VR2		_BIT(11)

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

/*
 * Devie connect on FPGA
 *
 */
// IF FPGA BASE
#define IF_FPGA_BASE	io_p2v(EMC_CS3_BASE)

// DPD registers
#define DPD_ADDR_A	0x150
#define DPD_WR_DATAL_A	0x151
#define DPD_WR_DATAH_A	0x152
#define DPD_RD_DATAL_A	0x00C
#define DPD_RD_DATAH_A	0x00D

#define DPD_ADDR_B	0x155
#define DPD_WR_DATAL_B	0x156
#define DPD_WR_DATAH_B	0x157
#define DPD_RD_DATAL_B	0x00E
#define DPD_RD_DATAH_B	0x00F

/****************************************
 *	Devie connect on CPLD
 ***************************************/
// CPLD
#define CPLD_BASE		io_p2v(EMC_CS2_BASE)

//Versions Module
#define HARD_VER_BASE		(CPLD_BASE + 0x00)
#define CPLD_VER_BASE		(CPLD_BASE + 0x01)
#define UBOOT_VER_BASE		(CPLD_BASE + 0x02)

//Hardware Watchdog Module
#define WATCHDOG_BASE		(CPLD_BASE + 0x03)

//Status&Detect Module
#define STATUS_BASE		(CPLD_BASE + 0x04)
#define DETECT_BASE		(CPLD_BASE + 0x05)
#define DETECT_AGE		_BIT(0)
#define DETECT_PA		_BIT(1)

//Reset Module
#define RESET_BASE		(CPLD_BASE + 0x06)

//LMK03000 (cLock modules)
#define LMK03000_BASE		(CPLD_BASE + 0x07)
#define LMK03000_LD		_BIT(5)
#define LMK03000_SYNC		_BIT(4)
#define LMK03000_GOE		_BIT(3)
#define LMK03000_LE		_BIT(2)
#define LMK03000_DAT		_BIT(1)
#define LMK03000_CLK		_BIT(0)


// ADF4350 (Local OSC modules of A&B channel)
#define LO_TRXA_BASE		(CPLD_BASE + 0x08)
#define LO_TXB_BASE		(CPLD_BASE + 0x09)
#define LO_RXB_BASE		(CPLD_BASE + 0x0a)
#define ADF4350_LD		_BIT(3)
#define ADF4350_LE		_BIT(2)
#define ADF4350_CLK		_BIT(1)
#define ADF4350_DAT		_BIT(0)
#define ADF4350_ALL		(ADF4350_LD | ADF4350_LE | ADF4350_CLK | ADF4350_DAT)

// ADS62C17 (ADC modules)
#define ADCA_BASE		(CPLD_BASE + 0x0b)
#define ADCB_BASE		(CPLD_BASE + 0x0c)
#define ADS62C17_SDOUT		_BIT(3)
#define ADS62C17_SEN		_BIT(2)
#define ADS62C17_SDATA		_BIT(1)
#define ADS62C17_SCLK		_BIT(0)
#define ADS62C17_ALL		(ADS62C17_SCLK | ADS62C17_SDATA | ADS62C17_SEN | ADS62C17_SDOUT)

// DAC5682Z (DAC modules)
#define DACA_BASE		(CPLD_BASE + 0x0d)
#define DACB_BASE		(CPLD_BASE + 0x0e)
#define DAC5682Z_SDOUT		_BIT(3)
#define DAC5682Z_SEN		_BIT(2)
#define DAC5682Z_SDATA		_BIT(1)
#define DAC5682Z_SCLK		_BIT(0)
#define DAC5682Z_ALL		(DAC5682Z_SCLK | DAC5682Z_SDATA | DAC5682Z_SEN | DAC5682Z_SDOUT)


//FPGA configure Module
#define FPGA_CFG_DAT_BASE	(CPLD_BASE + 0x0f)

#define FPGA_CFG_CTRL_BASE	(CPLD_BASE + 0x10)
#define FPGA_CFG_CTRL_DONE	_BIT(5)
#define FPGA_CFG_CTRL_INT	_BIT(4)
#define FPGA_CFG_CTRL_CS	_BIT(1)
#define FPGA_CFG_CTRL_PROG	_BIT(0)

//Power MISC Module
#define POWER_INT_BASE		(CPLD_BASE + 0x12)
#define POWER_INT_ENA		_BIT(4)
#define POWER_INT_ACT		_BIT(0)

#define POWER_PEND_BASE		(CPLD_BASE + 0x13)

#define POWER_STAT_BASE		(CPLD_BASE + 0x14)
#define POWER_STAT_P28		_BIT(3)	//PA control Port

#endif // __G200WO_HARDWARE_H__

