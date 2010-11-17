/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::   Author     : 
 ::::    ::       ::        ::          Maintainer : Athurg.Feng
 :: ::   ::       ::         ::         Project    : G410SD
 ::  ::  ::       ::           :::      FileName   : g410sd_hw.h
 ::   :: ::       ::             ::     Generate   : 2009.06.02
 ::    ::::       ::       ::      ::   Update     : 2010-11-17 13:13:50
::::    :::     ::::::      ::::::::    Version    : v0.2

Description
	None
*/

#ifndef __G410SD_HARDWARE_H__
#define __G410SD_HARDWARE_H__

#ifdef __KERNEL__
#include <mach/platform.h>
#include <mach/lpc32xx_gpio.h>
#endif

#ifndef _BIT
#define _BIT(n)	(1<<n)
#endif

/* ioctrl magic number define */
#define G410SD_IOCTL_MAGIC	'G'
#define G410SD_PROJECT

/*
 * interrupt number define
 */

#define IF_AGC_IRQ	IRQ_GPIO_00	//PCB pin name is LPC_IRQ0_N
#define IF_ALC_IRQ	IRQ_GPIO_01	//PCB pin name is LPC_IRQ1_N
#define POWER_IRQ	IRQ_GPI_03	//PCB pin name is LPC_IRQ2_N
#define CPRI_IRQ	IRQ_GPI_07	//PCB pin name is LPC_IRQ3_N


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

//CPRI registers
#define CPRI_BASE	(IF_FPGA_BASE + 200)
#define CPRI_VER	(CPRI_BASE + 0xF0)
#define CPRI_LEVEL	(CPRI_BASE + 0xF0)//级联编号
#define CPRI_MAC0	(CPRI_BASE + 0xF6)
#define CPRI_MAC1	(CPRI_BASE + 0xF7)
#define CPRI_MAC2	(CPRI_BASE + 0xF8)
//收发FIFO
#define CPRI_RX1_FIFO	(CPRI_BASE + 0xF0)
#define CPRI_RX2_FIFO	(CPRI_BASE + 0xF0)
#define CPRI_RX3_FIFO	(CPRI_BASE + 0xF0)
#define CPRI_RX4_FIFO	(CPRI_BASE + 0xF0)
#define CPRI_RX5_FIFO	(CPRI_BASE + 0xF0)
#define CPRI_RX6_FIFO	(CPRI_BASE + 0xF0)
#define CPRI_RX_FIFO	CPRI_RX1_FIFO
#define CPRI_TX_FIFO	(CPRI_BASE + 0xF0)
//收发长度寄存器
#define CPRI_RX1_LEN	(CPRI_BASE + 0xF0)
#define CPRI_RX2_LEN	(CPRI_BASE + 0xF0)
#define CPRI_RX3_LEN	(CPRI_BASE + 0xF0)
#define CPRI_RX4_LEN	(CPRI_BASE + 0xF0)
#define CPRI_RX5_LEN	(CPRI_BASE + 0xF0)
#define CPRI_RX6_LEN	(CPRI_BASE + 0xF0)
#define CPRI_RX_LEN	CPRI_RX1_LEN
//收发中断标志寄存器
#define CPRI_INTERRUPT_FLAG	(CPRI_BASE+0xFF)
#define CPRI_INTERRUPT_TX	(1<<0)
#define CPRI_INTERRUPT_RX	(1<<1)

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

//Detect Module
#define DETECT_BASE		(CPLD_BASE + 0x04)
#define DETECT_AGE		_BIT(0)
#define DETECT_PA		_BIT(1)

//Status Module
#define STATUS_BASE		(CPLD_BASE + 0x05)

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


// LMK2531 (Local OSC modules of A&B channel)
#define TRX_LO_BASE		(CPLD_BASE + 0x08)
#define DET_LO_BASE		(CPLD_BASE + 0x09)
#define LMK2531_LD		_BIT(3)
#define LMK2531_LE		_BIT(2)
#define LMK2531_CLK		_BIT(1)
#define LMK2531_DAT		_BIT(0)
#define LMK2531_ALL		(LMK2531_LD | LMK2531_LE | LMK2531_CLK | LMK2531_DAT)

// ADS62C17 (ADC modules)
#define ADCA_BASE		(CPLD_BASE + 0x0A)
#define ADCB_BASE		(CPLD_BASE + 0x0B)
#define ADS62C17_SDOUT		_BIT(3)
#define ADS62C17_SEN		_BIT(2)
#define ADS62C17_SDATA		_BIT(1)
#define ADS62C17_SCLK		_BIT(0)
#define ADS62C17_ALL		(ADS62C17_SCLK | ADS62C17_SDATA | ADS62C17_SEN | ADS62C17_SDOUT)

// DAC5682Z (DAC modules)
#define DACA_BASE		(CPLD_BASE + 0x0C)
#define DACB_BASE		(CPLD_BASE + 0x0D)
#define DAC5682Z_SDOUT		_BIT(3)
#define DAC5682Z_SEN		_BIT(2)
#define DAC5682Z_SDATA		_BIT(1)
#define DAC5682Z_SCLK		_BIT(0)
#define DAC5682Z_ALL		(DAC5682Z_SCLK | DAC5682Z_SDATA | DAC5682Z_SEN | DAC5682Z_SDOUT)


//FPGA configure Module
#define FPGA_CFG_DAT_BASE	(CPLD_BASE + 0x0E)

#define FPGA_CFG_CTRL_BASE	(CPLD_BASE + 0x0F)
#define FPGA_CFG_CTRL_DONE	_BIT(5)
#define FPGA_CFG_CTRL_INT	_BIT(4)
#define FPGA_CFG_CTRL_CS	_BIT(1)
#define FPGA_CFG_CTRL_PROG	_BIT(0)

#define FPGA_CFG_CLK_BASE	(CPLD_BASE + 0x10)

//SFP Control Module
#define SFP0_BASE		(CPLD_BASE + 0x11)
#define SFP1_BASE		(CPLD_BASE + 0x12)
#define SFP_SCL_OUT		_BIT(0)
#define SFP_SDA_OUT		_BIT(1)
#define SFP_SDA_IN		_BIT(2)
#define SFP_TXDIS		_BIT(3)
#define SFP_TXF			_BIT(4)
#define SFP_RLOS		_BIT(5)
#define SFP_DETN		_BIT(6)

//Power MISC Module
#define POWER_INT_BASE		(CPLD_BASE + 0x13)
#define POWER_INT_ENA		_BIT(4)
#define POWER_INT_ACT		_BIT(0)

#define POWER_PEND_BASE		(CPLD_BASE + 0x14)

#define POWER_STAT_BASE		(CPLD_BASE + 0x15)
#define POWER_STAT_P28		_BIT(3)	//PA control Port

#endif // __G410SD_HARDWARE_H__

