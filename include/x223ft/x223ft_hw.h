/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::       Author     : 
 ::::    ::       ::        ::              Maintainer : Andy.wu
 :: ::   ::       ::         ::             Project    : X223WO
 ::  ::  ::       ::           :::          File Name  : x223wo_hw.h
 ::   :: ::       ::             ::         Generate   : 2010.07.25
 ::    ::::       ::       ::      ::       Update     : 2010-11-24 12:00:17
::::    :::     ::::::      ::::::::        Version    : v0.1

Description
	None
*/

#ifndef __X223FT_HARDWARE_H__
#define __X223FT_HARDWARE_H__

#ifdef  __KERNEL__
#include <mach/platform.h>
#include <mach/lpc32xx_gpio.h>
#endif

#define  X223FT_PROJECT

/* ioctrl magic number define */
#define X223FT_IOCTL_MAGIC	'G'


/*
	interrupt number define

	irqs pin	GPIO
	=========================
	LPC_IRQ0_N => IRQ_GPIO_00
	LPC_IRQ1_N => IRQ_GPIO_01
	LPC_IRQ2_N => IRQ_GPI_03	(reserve)
	LPC_IRQ3_N => IRQ_GPI_07	(reserve)
*/

#define POWER_IRQ	        IRQ_GPIO_00
#define IF_AGC_IRQ	        IRQ_GPI_03
#define CPRI_IRQ		IRQ_GPI_07

#ifndef _BIT
#define _BIT(n)	(1<<n)
#endif



/****************************************
 *	Devie connect on LPC3250
 ***************************************/
//Virtual Address of GPIO BASE
#define GPIO_IOBASE	        io_p2v(GPIO_BASE)
                                
//RTC Module                    
#define RTC_IOBASE	        io_p2v(RTC_BASE)
                                                                
// TMP125                       
#define TMP125_SCLK	        _BIT(16)
#define TMP125_CS_N	        _BIT(18)
#define TMP125_DOUT	 	_BIT(22)

//LEDS
#define LED_RUN		_BIT(1)
#define LED_DPD		_BIT(4)

/****************************************
 *	Devie connect on FPGA
 ***************************************/
// IF FPGA DPD & CFR Module
#define IF_FPGA_BASE	        io_p2v(EMC_CS3_BASE)

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


// Some normal registers
#ifndef ADDR_VERSION_VER	//to avoid user-space comflict with driver
#define ADDR_VERSION_VER	0x00
#define ADDR_VERSION_DATE	0x01
#define ADDR_VERSION_YEAR	0x02
#endif

// DPD registers
#define DPD_ADDR_A		0x300
#define DPD_ADDR_B       	0x300
#define DPD_ADDR_WR_DATAL	0x301
#define DPD_ADDR_WR_DATAH	0x302
#define DPD_ADDR_RD_DATAL	0x303
#define DPD_ADDR_RD_DATAH	0x304

//FIXME:
//	We don't have CFR Modules in G200WO
//	These address define should refer to X223WO's Documents
// CFR registers
#define CFR_ADDR_A		0x320
#define CFR_ADDR_DATAL_A	0x321
#define CFR_ADDR_DATAH_A	0x322

#define CFR_ADDR_B		0x326
#define CFR_ADDR_DATAL_B	0x327
#define CFR_ADDR_DATAH_B	0x328

/****************************************
 *	Devie connect on CPLD
 ***************************************/
// CPLD
#define CPLD_BASE		io_p2v(EMC_CS2_BASE)

/****************************************
 *	Devie connect on CPLD
 ***************************************/

//Version Module
#define HARD_VER_BASE		(CPLD_BASE + 0x00)
#define CPLD_VER_BASE		(CPLD_BASE + 0x01)
#define UBOOT_VER_BASE		(CPLD_BASE + 0x02)

//Hardware Watchdog Module
#define WATCHDOG_BASE		(CPLD_BASE + 0x03)

//Status&Detect Module
#define DETECT_BASE		(CPLD_BASE + 0x04)
#define STATUS_BASE		(CPLD_BASE + 0x05)

//Reset Module
#define RESET_BASE		(CPLD_BASE + 0x06)

//LMK04031 Module
#define LMK04031_BASE		(CPLD_BASE + 0x07)
#define LMK04031_LD	        _BIT(5)
#define LMK04031_SYNC	        _BIT(4)
#define LMK04031_GOE	        _BIT(3)
#define LMK04031_LE	        _BIT(2)
#define LMK04031_DAT	        _BIT(1)
#define LMK04031_CLK	        _BIT(0)

//Local Osc of A&B channel (adf4350)
#define ADF4350_A_BASE          (CPLD_BASE + 0x08)
#define ADF4350_B_BASE          (CPLD_BASE + 0x09)
#define ADF4350_LD		_BIT(3)
#define ADF4350_LE		_BIT(2)
#define ADF4350_CLK		_BIT(1)
#define ADF4350_DAT		_BIT(0)
#define ADF4350_ALL		(ADF4350_LD | ADF4350_LE | ADF4350_CLK | ADF4350_DAT)

//ADC Module
#define ADCA_BASE		(CPLD_BASE + 0x0A)
#define ADS62C17_SDOUT		_BIT(3)
#define ADS62C17_SEN		_BIT(2)
#define ADS62C17_SDATA		_BIT(1)
#define ADS62C17_SCLK		_BIT(0)
#define ADS62C17_ALL		(ADS62C17_SDATA | ADS62C17_SDOUT)

//DAC Module
#define DACA_BASE		(CPLD_BASE + 0x0D)
#define DAC5682Z_SDOUT		_BIT(3)
#define DAC5682Z_SEN		_BIT(2)
#define DAC5682Z_SDATA		_BIT(1)
#define DAC5682Z_SCLK		_BIT(0)
#define DAC5682Z_ALL		(DAC5682Z_SCLK | DAC5682Z_SDATA | DAC5682Z_SDOUT)

//FPGA configure Module
#define FPGA_CFG_DAT_BASE	(CPLD_BASE + 0x0E)
#define FPGA_CFG_CTRL_BASE	(CPLD_BASE + 0x0F)
#define FPGA_CFG_CTRL_DONE	_BIT(5)
#define FPGA_CFG_CTRL_INT	_BIT(4)
#define FPGA_CFG_CTRL_PROG	_BIT(1)
#define FPGA_CFG_CTRL_CS	_BIT(0)

#define FPGA_CFG_CLK_BASE	(CPLD_BASE + 0x10)

#endif // __G200WO_HARDWARE_H__

