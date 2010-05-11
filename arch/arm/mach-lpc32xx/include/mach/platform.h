/*
 * asm-arm/arch-lpc32xx/platform.h
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
 *
 * Copyright (C) 2008 NXP Semiconductors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __LPC32XX_PLATFORM_H__
#define __LPC32XX_PLATFORM_H__

#include <mach/hardware.h>

/*
 * AHB 0 physical base addresses
 */
#define SLC_BASE	0x20020000
#define SSP0_BASE       0x20084000
#define SPI1_BASE	0x20088000
#define SSP1_BASE       0x2008C000
#define SPI2_BASE	0x20090000
#define I2S0_BASE       0x20094000
#define SD_BASE		0x20098000
#define I2S1_BASE       0x2009C000
#define MLC_BASE	0x200A8000
#define AHB0_START      SLC_BASE
#define AHB0_SIZE       ((MLC_BASE - SLC_BASE) + SZ_4K)

/*
 * AHB 1 physical base addresses
 */
#define DMA_BASE	0x31000000
#define USB_BASE	0x31020000
#define USBH_BASE	0x31020000
#define USB_OTG_BASE	0x31020000
#define OTG_I2C_BASE	0x31020300
#define LCD_BASE	0x31040000
#define ETHERNET_BASE	0x31060000
#define EMC_BASE        0x31080000
#define ETB_CFG_BASE	0x310C0000
#define ETB_DATA_BASE	0x310E0000
#define AHB1_START      DMA_BASE
#define AHB1_SIZE       ((EMC_BASE - DMA_BASE) + SZ_4K)

/*
 * FAB physical base addresses
 */
#define CLK_PM_BASE	0x40004000
#define MIC_BASE	0x40008000
#define SIC1_BASE	0x4000C000
#define SIC2_BASE	0x40010000
#define HS_UART1_BASE	0x40014000
#define HS_UART2_BASE	0x40018000
#define HS_UART7_BASE	0x4001C000
#define RTC_BASE	0x40024000
#define RTC_RAM_BASE	0x40024080
#define GPIO_BASE	0x40028000
#define PWM3_BASE	0x4002C000
#define PWM4_BASE	0x40030000
#define MSTIM_BASE	0x40034000
#define HSTIM_BASE	0x40038000
#define WDTIM_BASE	0x4003C000
#define DEBUG_CTRL_BASE	0x40040000
#define TIMER0_BASE     0x40044000
#define ADC_BASE	0x40048000
#define TIMER1_BASE     0x4004C000
#define KSCAN_BASE	0x40050000
#define UART_CTRL_BASE	0x40054000
#define TIMER2_BASE     0x40058000
#define PWM1_BASE	0x4005C000
#define PWM2_BASE	0x4005C004
#define TIMER3_BASE     0x40060000

/*
 * APB physical base addresses
 */

#define UART3_BASE	0x40080000
#define UART4_BASE	0x40088000
#define UART5_BASE	0x40090000
#define UART6_BASE	0x40098000
#define I2C1_BASE	0x400A0000
#define I2C2_BASE	0x400A8000

/* FAB and APB base and sizing */
#define FABAPB_START    CLK_PM_BASE
#define FABAPB_SIZE    ((I2C2_BASE - CLK_PM_BASE) + SZ_4K)

/*
 * Internal memory Bases
 */
#define IRAM_BASE       0x08000000
#define IROM_BASE       0x0C000000

/*
 * External Static Memory Bank Address Space Bases
 */
#define EMC_CS0_BASE	0xE0000000
#define EMC_CS1_BASE	0xE1000000
#define EMC_CS2_BASE	0xE2000000
#define EMC_CS3_BASE	0xE3000000

/*
 * External SDRAM Memory Bank Address Space Bases
 */
#define EMC_DYCS0_BASE	0x80000000
#define EMC_DYCS1_BASE	0xA0000000

/*
 * Clock and crystal information
 */
#define MAIN_OSC_FREQ 13000000
#define CLOCK_OSC_FREQ 32768

/*
 * IRAM size
*/
#if defined (CONFIG_MACH_LPC32XX_IRAM_SIZE_256)
#define LPC32XX_IRAM_SIZE (256 * 1024)
#else
#define LPC32XX_IRAM_SIZE (128 * 1024)
#endif

#endif

