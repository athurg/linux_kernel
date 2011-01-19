/*
::::    :::: ::::::::::::    .::::::    Company    : NTS-intl
 :::     ::   ::  ::  ::   ::      ::       Author     : 
 ::::    ::       ::        ::              Maintainer : Andy.wu
 :: ::   ::       ::         ::             Project    : X223WO
 ::  ::  ::       ::           :::          File Name  : x223wo_hw.h
 ::   :: ::       ::             ::         Generate   : 2010.07.25
 ::    ::::       ::       ::      ::       Update     : 2010-12-11 16:15:43
::::    :::     ::::::      ::::::::        Version    : v0.1

Description
	None
*/

#ifndef __X223WO_HARDWARE_H__
#define __X223WO_HARDWARE_H__

#ifdef  __KERNEL__
#include <mach/platform.h>
#include <mach/lpc32xx_gpio.h>
#endif

#define  G420SD_PROJECT

/* ioctrl magic number define */
#define G420SD_IOCTL_MAGIC	'G'


/*
	interrupt number define

	irqs pin	GPIO
	=========================
	LPC_IRQ0_N => IRQ_GPIO_00
	LPC_IRQ1_N => IRQ_GPIO_01
	LPC_IRQ2_N => IRQ_GPI_03	(reserve)
	LPC_IRQ3_N => IRQ_GPI_07	(reserve)
*/

//#define POWER_IRQ	        IRQ_GPIO_00
#define IF_AGC_IRQ	        IRQ_GPI_03


/****************************************
 *	Devie connect on LPC3250
 ***************************************/
//Virtual Address of GPIO BASE
#define GPIO_IOBASE	        io_p2v(GPIO_BASE)
                                
//RTC Module                    
#define RTC_IOBASE	        io_p2v(RTC_BASE)


//ADC Module
#define SIC1_IOBASE            io_p2v(SIC1_BASE)
#define SIC1_RSR_IOBASE        (SIC1_IOBASE + 0x04)

#define CLK_PM_IOBASE          io_p2v(CLK_PM_BASE)
#define ADCLK_CTRL             (CLK_PM_IOBASE + 0xb4)
#define ADCLK_CTRL1            (CLK_PM_IOBASE + 0x60) 

#define ADC_LPC3250_IOBASE     io_p2v(ADC_BASE)
#define ADC_SELECT             (ADC_LPC3250_IOBASE + 0x04)
#define ADC_CTRL               (ADC_LPC3250_IOBASE + 0x08)
#define ADC_VALUE              (ADC_LPC3250_IOBASE + 0x48)


                                                                
// TMP125                       
#define TMP125_SCLK	        _BIT(16)
#define TMP125_CS_N	        _BIT(18)
#define TMP125_DOUT	 	_BIT(22)


/****************************************
 *	Devie connect on FPGA
 ***************************************/
// IF FPGA DPD & CFR Module
#define IF_FPGA_BASE	        io_p2v(EMC_CS3_BASE)

// Some normal registers
#ifndef ADDR_VERSION_VER	//to avoid user-space comflict with driver
#define ADDR_VERSION_VER	0x00
#define ADDR_VERSION_DATE	0x01
#define ADDR_VERSION_YEAR	0x02
#endif

// DPD registers
#define DPD_ADDR_A		0x60
#define DPD_ADDR_B       	0x60
#define DPD_ADDR_WR_DATAL	0x61
#define DPD_ADDR_WR_DATAH	0x62
#define DPD_ADDR_RD_DATAL	0x10
#define DPD_ADDR_RD_DATAH	0x11

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
#define STATUS_BASE		(CPLD_BASE + 0x04)
#define DETECT_BASE		(CPLD_BASE + 0x05)

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
#define ADF4350_TX_BASE        (CPLD_BASE + 0x08)
#define ADF4350_RXFB_BASE      (CPLD_BASE + 0x09)
#define ADF4350_LD		_BIT(5)
#define ADF4350_LE		_BIT(2)
#define ADF4350_DAT		_BIT(1)
#define ADF4350_CLK		_BIT(0)
#define ADF4350_ALL		(ADF4350_LD | ADF4350_LE | ADF4350_CLK | ADF4350_DAT)

//ADC Module
#define ADCA_BASE		(CPLD_BASE + 0x0a)
#define ADS62C17_SDOUT		_BIT(3)
#define ADS62C17_SEN		_BIT(2)
#define ADS62C17_SDATA		_BIT(1)
#define ADS62C17_SCLK		_BIT(0)
#define ADS62C17_ALL		(ADS62C17_SDATA | ADS62C17_SDOUT)

//DAC Module
#define DACA_BASE		(CPLD_BASE + 0x0b)
#define DACB_BASE		(CPLD_BASE + 0x0c)
#define DACC_BASE		(CPLD_BASE + 0x0d)
#define DACD_BASE		(CPLD_BASE + 0x0e)
#define AD9122_SDOUT		_BIT(3)
#define AD9122_SCS		_BIT(2)
#define AD9122_SDIO		_BIT(1)
#define AD9122_SCLK		_BIT(0)
#define AD9122_ALL		(AD9122_SCLK | AD9122_SDIO)

//FPGA configure Module
#define FPGA_CFG_DAT_BASE	(CPLD_BASE + 0x0f)
#define FPGA_CFG_CTRL_BASE	(CPLD_BASE + 0x10)
#define FPGA_CFG_CLK_BASE	(CPLD_BASE + 0x11)
#define FPGA_CFG_CTRL_DONE	_BIT(5)
#define FPGA_CFG_CTRL_INT	_BIT(4)
#define FPGA_CFG_CTRL_PROG	_BIT(1)
#define FPGA_CFG_CTRL_CS	_BIT(0)

//RS422 
#define RS422_BASE	        (CPLD_BASE + 0x12)

#endif // __G200WO_HARDWARE_H__

