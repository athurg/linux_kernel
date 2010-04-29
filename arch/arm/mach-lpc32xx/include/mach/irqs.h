/*
 * asm-arm/arch-lpc32xx/irqs.h
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

#ifndef __LPC32XX_IRQS_h__
#define __LPC32XX_IRQS_h__

#define INTC_MASK			0x00
#define INTC_RAW_STAT			0x04
#define INTC_STAT			0x08
#define INTC_POLAR			0x0C
#define INTC_ACT_TYPE			0x10
#define INTC_TYPE			0x14

#define INTC_SIC1_OFFS			32
#define INTC_SIC2_OFFS			64

/*
 * Default value represeting the Activation polarity of all internal
 * interrupt sources
 */
#define MIC_APR_DEFAULT		0x3FF0EFE0
#define SIC1_APR_DEFAULT	0xFBD27186
#define SIC2_APR_DEFAULT	0x801810C0

/*
 * Default value represeting the Activation Type of all internal
 * interrupt sources. All are level senesitive.
 */
#define MIC_ATR_DEFAULT		0x00000000
#define SIC1_ATR_DEFAULT	0x00026000
#define SIC2_ATR_DEFAULT	0x00000000

/*
 * MIC interrupts
 */
#define IRQ_SUB1IRQ			0
#define IRQ_SUB2IRQ			1
#define IRQ_PWM3			3
#define IRQ_PWM4			4
#define IRQ_HSTIMER			5
#define IRQ_WATCH			6
#define IRQ_UART_IIR3			7
#define IRQ_UART_IIR4			8
#define IRQ_UART_IIR5			9
#define IRQ_UART_IIR6			10
#define IRQ_FLASH			11
#define IRQ_SD1				13
#define IRQ_LCD				14
#define IRQ_SD0				15
#define IRQ_TIMER0			16
#define IRQ_TIMER1			17
#define IRQ_TIMER2			18
#define IRQ_TIMER3			19
#define IRQ_SSP0			20
#define IRQ_SSP1			21
#define IRQ_I2S0			22
#define IRQ_I2S1			23
#define IRQ_UART_IIR7			24
#define IRQ_UART_IIR2			25
#define IRQ_UART_IIR1			26
#define IRQ_MSTIMER			27
#define IRQ_DMA				28
#define IRQ_ETHERNET			29
#define IRQ_SUB1FIQ			30
#define IRQ_SUB2FIQ			31

/*
 * SIC1 interrupts
 */
#define IRQ_JTAG_COMM_TX		(INTC_SIC1_OFFS + 1)
#define IRQ_JTAG_COMM_RX		(INTC_SIC1_OFFS + 2)
#define IRQ_GPI_11			(INTC_SIC1_OFFS + 4)
#define IRQ_TS_P			(INTC_SIC1_OFFS + 6)
#define IRQ_TS_IRQ			(INTC_SIC1_OFFS + 7)
#define IRQ_TS_AUX			(INTC_SIC1_OFFS + 8)
#define IRQ_SPI2			(INTC_SIC1_OFFS + 12)
#define IRQ_PLLUSB			(INTC_SIC1_OFFS + 13)
#define IRQ_PLLHCLK			(INTC_SIC1_OFFS + 14)
#define IRQ_PLL397			(INTC_SIC1_OFFS + 17)
#define IRQ_I2C_2			(INTC_SIC1_OFFS + 18)
#define IRQ_I2C_1			(INTC_SIC1_OFFS + 19)
#define IRQ_RTC				(INTC_SIC1_OFFS + 20)
#define IRQ_KEY				(INTC_SIC1_OFFS + 22)
#define IRQ_SPI1			(INTC_SIC1_OFFS + 23)
#define IRQ_SW				(INTC_SIC1_OFFS + 24)
#define IRQ_USB_OTG_TIMER		(INTC_SIC1_OFFS + 25)
#define IRQ_USB_OTG_ATX			(INTC_SIC1_OFFS + 26)
#define IRQ_USB_HOST			(INTC_SIC1_OFFS + 27)
#define IRQ_USB_DEV_DMA			(INTC_SIC1_OFFS + 28)
#define IRQ_USB_DEV_LP			(INTC_SIC1_OFFS + 29)
#define IRQ_USB_DEV_HP			(INTC_SIC1_OFFS + 30)
#define IRQ_USB_I2C			(INTC_SIC1_OFFS + 31)

/*
 * SIC2 interrupts
 */
#define IRQ_GPIO_00			(INTC_SIC2_OFFS + 0)
#define IRQ_GPIO_01			(INTC_SIC2_OFFS + 1)
#define IRQ_GPIO_02			(INTC_SIC2_OFFS + 2)
#define IRQ_GPIO_03			(INTC_SIC2_OFFS + 3)
#define IRQ_GPIO_04			(INTC_SIC2_OFFS + 4)
#define IRQ_GPIO_05			(INTC_SIC2_OFFS + 5)
#define IRQ_SPI2_DATAIN			(INTC_SIC2_OFFS + 6)
#define IRQ_U2_HCTS			(INTC_SIC2_OFFS + 7)
#define IRQ_P0_P1_IRQ			(INTC_SIC2_OFFS + 8)
#define IRQ_GPI_08			(INTC_SIC2_OFFS + 9)
#define IRQ_GPI_09			(INTC_SIC2_OFFS + 10)
#define IRQ_GPI_10			(INTC_SIC2_OFFS + 11)
#define IRQ_U7_HCTS			(INTC_SIC2_OFFS + 12)
#define IRQ_GPI_07			(INTC_SIC2_OFFS + 15)
#define IRQ_SDIO			(INTC_SIC2_OFFS + 18)
#define IRQ_U5_RX			(INTC_SIC2_OFFS + 19)
#define IRQ_SPI1_DATAIN			(INTC_SIC2_OFFS + 20)
#define IRQ_GPI_00			(INTC_SIC2_OFFS + 22)
#define IRQ_GPI_01			(INTC_SIC2_OFFS + 23)
#define IRQ_GPI_02			(INTC_SIC2_OFFS + 24)
#define IRQ_GPI_03			(INTC_SIC2_OFFS + 25)
#define IRQ_GPI_04			(INTC_SIC2_OFFS + 26)
#define IRQ_GPI_05			(INTC_SIC2_OFFS + 27)
#define IRQ_GPI_06			(INTC_SIC2_OFFS + 28)
#define IRQ_SYSCLK			(INTC_SIC2_OFFS + 31)

#define NR_IRQS         96

#endif /* __LPC32XX_IRQS_h__ */

