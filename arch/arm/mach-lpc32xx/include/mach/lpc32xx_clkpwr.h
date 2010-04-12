/*
 * asm-arm/arch-lpc32xx/lpc32xx_clkpwr.h
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

#ifndef LPC32XX_CLKPWR_H
#define LPC32XX_CLKPWR_H

#define _BIT(n) (1 << (n))

/**********************************************************************
* Clock and Power control register offsets
**********************************************************************/

#define CLKPWR_DEBUG_CTRL(x)			(x + 0x000)
#define CLKPWR_BOOTMAP(x)			(x + 0x014)
#define CLKPWR_P01_ER(x)			(x + 0x018)
#define CLKPWR_USBCLK_PDIV(x)			(x + 0x01C)
#define CLKPWR_INT_ER(x)			(x + 0x020)
#define CLKPWR_INT_RS(x)			(x + 0x024)
#define CLKPWR_INT_SR(x)			(x + 0x028)
#define CLKPWR_INT_AP(x)			(x + 0x02C)
#define CLKPWR_PIN_ER(x)			(x + 0x030)
#define CLKPWR_PIN_RS(x)			(x + 0x034)
#define CLKPWR_PIN_SR(x)			(x + 0x038)
#define CLKPWR_PIN_AP(x)			(x + 0x03C)
#define CLKPWR_HCLK_DIV(x)			(x + 0x040)
#define CLKPWR_PWR_CTRL(x)			(x + 0x044)
#define CLKPWR_PLL397_CTRL(x)			(x + 0x048)
#define CLKPWR_MAIN_OSC_CTRL(x)			(x + 0x04C)
#define CLKPWR_SYSCLK_CTRL(x)			(x + 0x050)
#define CLKPWR_LCDCLK_CTRL(x)			(x + 0x054)
#define CLKPWR_HCLKPLL_CTRL(x)			(x + 0x058)
#define CLKPWR_ADC_CLK_CTRL_1(x)		(x + 0x060)
#define CLKPWR_USB_CTRL(x)			(x + 0x064)
#define CLKPWR_SDRAMCLK_CTRL(x)			(x + 0x068)
#define CLKPWR_DDR_LAP_NOM(x)			(x + 0x06C)
#define CLKPWR_DDR_LAP_COUNT(x)			(x + 0x070)
#define CLKPWR_DDR_LAP_DELAY(x)			(x + 0x074)
#define CLKPWR_SSP_CLK_CTRL(x)			(x + 0x078)
#define CLKPWR_I2S_CLK_CTRL(x)			(x + 0x07C)
#define CLKPWR_MS_CTRL(x)			(x + 0x080)
#define CLKPWR_MACCLK_CTRL(x)			(x + 0x090)
#define CLKPWR_TEST_CLK_SEL(x)			(x + 0x0A4)
#define CLKPWR_SFW_INT(x)			(x + 0x0A8)
#define CLKPWR_I2C_CLK_CTRL(x)			(x + 0x0AC)
#define CLKPWR_KEY_CLK_CTRL(x)			(x + 0x0B0)
#define CLKPWR_ADC_CLK_CTRL(x)			(x + 0x0B4)
#define CLKPWR_PWM_CLK_CTRL(x)			(x + 0x0B8)
#define CLKPWR_TIMER_CLK_CTRL(x)		(x + 0x0BC)
#define CLKPWR_TIMERS_PWMS_CLK_CTRL_1(x)	(x + 0x0C0)
#define CLKPWR_SPI_CLK_CTRL(x)			(x + 0x0C4)
#define CLKPWR_NAND_CLK_CTRL(x)			(x + 0x0C8)
#define CLKPWR_UART3_CLK_CTRL(x)		(x + 0x0D0)
#define CLKPWR_UART4_CLK_CTRL(x)		(x + 0x0D4)
#define CLKPWR_UART5_CLK_CTRL(x)		(x + 0x0D8)
#define CLKPWR_UART6_CLK_CTRL(x)		(x + 0x0DC)
#define CLKPWR_IRDA_CLK_CTRL(x)			(x + 0x0E0)
#define CLKPWR_UART_CLK_CTRL(x)			(x + 0x0E4)
#define CLKPWR_DMA_CLK_CTRL(x)			(x + 0x0E8)
#define CLKPWR_AUTOCLOCK(x)			(x + 0x0EC)

/**********************************************************************
* clkpwr_debug_ctrl register definitions
**********************************************************************/
/* VFP clock enable */
#define CLKPWR_VFP_CLOCK_ENABLE_BIT _BIT(4)

/**********************************************************************
* clkpwr_bootmap register definitions
**********************************************************************/
/* Boot mapping at address 0x0, (0) = IROM, (1) = IRAM */
#define CLKPWR_BOOTMAP_SEL_BIT     0x1

/**********************************************************************
* clkpwr_start_gpio register bit definitions
**********************************************************************/
/* GPI/O sources bit positions for interrupt wakeup */
#define CLKPWR_GPIOSRC_P1IO23_BIT  _BIT(31)
#define CLKPWR_GPIOSRC_P1IO22_BIT  _BIT(30)
#define CLKPWR_GPIOSRC_P1IO21_BIT  _BIT(29)
#define CLKPWR_GPIOSRC_P1IO20_BIT  _BIT(28)
#define CLKPWR_GPIOSRC_P1IO19_BIT  _BIT(27)
#define CLKPWR_GPIOSRC_P1IO18_BIT  _BIT(26)
#define CLKPWR_GPIOSRC_P1IO17_BIT  _BIT(25)
#define CLKPWR_GPIOSRC_P1IO16_BIT  _BIT(24)
#define CLKPWR_GPIOSRC_P1IO15_BIT  _BIT(23)
#define CLKPWR_GPIOSRC_P1IO14_BIT  _BIT(22)
#define CLKPWR_GPIOSRC_P1IO13_BIT  _BIT(21)
#define CLKPWR_GPIOSRC_P1IO12_BIT  _BIT(20)
#define CLKPWR_GPIOSRC_P1IO11_BIT  _BIT(19)
#define CLKPWR_GPIOSRC_P1IO10_BIT  _BIT(18)
#define CLKPWR_GPIOSRC_P1IO9_BIT   _BIT(17)
#define CLKPWR_GPIOSRC_P1IO8_BIT   _BIT(16)
#define CLKPWR_GPIOSRC_P1IO7_BIT   _BIT(15)
#define CLKPWR_GPIOSRC_P1IO6_BIT   _BIT(14)
#define CLKPWR_GPIOSRC_P1IO5_BIT   _BIT(13)
#define CLKPWR_GPIOSRC_P1IO4_BIT   _BIT(12)
#define CLKPWR_GPIOSRC_P1IO3_BIT   _BIT(11)
#define CLKPWR_GPIOSRC_P1IO2_BIT   _BIT(10)
#define CLKPWR_GPIOSRC_P1IO1_BIT   _BIT(9)
#define CLKPWR_GPIOSRC_P1IO0_BIT   _BIT(8)
#define CLKPWR_GPIOSRC_P0IO7_BIT   _BIT(7)
#define CLKPWR_GPIOSRC_P0IO6_BIT   _BIT(6)
#define CLKPWR_GPIOSRC_P0IO5_BIT   _BIT(5)
#define CLKPWR_GPIOSRC_P0IO4_BIT   _BIT(4)
#define CLKPWR_GPIOSRC_P0IO3_BIT   _BIT(3)
#define CLKPWR_GPIOSRC_P0IO2_BIT   _BIT(2)
#define CLKPWR_GPIOSRC_P0IO1_BIT   _BIT(1)
#define CLKPWR_GPIOSRC_P0IO0_BIT   _BIT(0)

/**********************************************************************
* clkpwr_usbclk_pdiv register definitions
**********************************************************************/
/* Macro for setting USB PLL clock predivider */
#define CLKPWR_SET_PLL_USBPDIV(n)  ((n) & 0xF)
/* Mask for USB PLL clock predivider bits */
#define CLKPWR_USBPDIV_PLL_MASK    0xF

/**********************************************************************
* clkpwr_start_int, clkpwr_start_raw_sts_int, clkpwr_start_sts_int,
* clkpwr_start_pol_int, register bit definitions
**********************************************************************/
/* Internal sources bit positions for interrupt wakeup */
#define CLKPWR_INTSRC_ADC_BIT      _BIT(31)
#define CLKPWR_INTSRC_TS_P_BIT     _BIT(30)
#define CLKPWR_INTSRC_TS_AUX_BIT   _BIT(29)
#define CLKPWR_INTSRC_USBAHNEEDCLK_BIT _BIT(26)
#define CLKPWR_INTSRC_MSTIMER_BIT  _BIT(25)
#define CLKPWR_INTSRC_RTC_BIT      _BIT(24)
#define CLKPWR_INTSRC_USBNEEDCLK_BIT _BIT(23)
#define CLKPWR_INTSRC_USB_BIT      _BIT(22)
#define CLKPWR_INTSRC_I2C_BIT      _BIT(21)
#define CLKPWR_INTSRC_USBOTGTIMER_BIT _BIT(20)
#define CLKPWR_INTSRC_USBATXINT_BIT _BIT(19)
#define CLKPWR_INTSRC_KEY_BIT      _BIT(16)
#define CLKPWR_INTSRC_MAC_BIT      _BIT(7)
#define CLKPWR_INTSRC_P0P1_BIT     _BIT(6)
#define CLKPWR_INTSRC_GPIO_05_BIT  _BIT(5)
#define CLKPWR_INTSRC_GPIO_04_BIT  _BIT(4)
#define CLKPWR_INTSRC_GPIO_03_BIT  _BIT(3)
#define CLKPWR_INTSRC_GPIO_02_BIT  _BIT(2)
#define CLKPWR_INTSRC_GPIO_01_BIT  _BIT(1)
#define CLKPWR_INTSRC_GPIO_00_BIT  _BIT(0)

/**********************************************************************
* clkpwr_start_pin, clkpwr_start_raw_sts_pin, clkpwr_start_sts_pin,
* clkpwr_start_pol_pin register bit definitions
**********************************************************************/
/* External sources bit positions for interrupt wakeup */
#define CLKPWR_EXTSRC_U7_RX_BIT    _BIT(31)
#define CLKPWR_EXTSRC_U7_HCTS_BIT  _BIT(30)
#define CLKPWR_EXTSRC_U6_IRRX_BIT  _BIT(28)
#define CLKPWR_EXTSRC_U5_RX_BIT    _BIT(26)
#define CLKPWR_EXTSRC_GPI_11_BIT   _BIT(25)
#define CLKPWR_EXTSRC_U3_RX_BIT    _BIT(24)
#define CLKPWR_EXTSRC_U2_HCTS_BIT  _BIT(23)
#define CLKPWR_EXTSRC_U2_RX_BIT    _BIT(22)
#define CLKPWR_EXTSRC_U1_RX_BIT    _BIT(21)
#define CLKPWR_EXTSRC_MSDIO_INT_BIT _BIT(18)
#define CLKPWR_EXTSRC_MSDIO_SRT_BIT _BIT(17)
#define CLKPWR_EXTSRC_GPIO_O6_BIT  _BIT(16)
#define CLKPWR_EXTSRC_GPIO_O5_BIT  _BIT(15)
#define CLKPWR_EXTSRC_GPIO_O4_BIT  _BIT(14)
#define CLKPWR_EXTSRC_GPIO_O3_BIT  _BIT(13)
#define CLKPWR_EXTSRC_GPIO_O2_BIT  _BIT(12)
#define CLKPWR_EXTSRC_GPIO_O1_BIT  _BIT(11)
#define CLKPWR_EXTSRC_GPIO_O0_BIT  _BIT(10)
#define CLKPWR_EXTSRC_SYSCLKEN_BIT _BIT(9)
#define CLKPWR_EXTSRC_SPI1_DATIN_BIT _BIT(8)
#define CLKPWR_EXTSRC_GPIO_O7_BIT  _BIT(7)
#define CLKPWR_EXTSRC_SPI2_DATIN_BIT _BIT(6)
#define CLKPWR_EXTSRC_GPIO_10_BIT  _BIT(5)
#define CLKPWR_EXTSRC_GPIO_O9_BIT  _BIT(4)
#define CLKPWR_EXTSRC_GPIO_O8_BIT  _BIT(3)

/**********************************************************************
* clkpwr_hclk_div register definitions
**********************************************************************/
/* HCLK Divider DDRAM clock stop (used for SDRAM only) */
#define CLKPWR_HCLKDIV_DDRCLK_STOP (0x0 << 7)
/* HCLK Divider DDRAM clock is the same speed as the ARM */
#define CLKPWR_HCLKDIV_DDRCLK_NORM (0x1 << 7)
/* HCLK Divider DDRAM clock is half the speed as the ARM */
#define CLKPWR_HCLKDIV_DDRCLK_HALF (0x2 << 7)
/* HCLK Divider PERIPH_CLK divider, for a value of n, the divider is
   (1+n), maximum value of n is 32 */
#define CLKPWR_HCLKDIV_PCLK_DIV(n) (((n) & 0x1F) << 2)
/* HCLK Divider, for a value of n, the divider is (2^n), maximum
   value of n is 2 for a divider of 4 */
#define CLKPWR_HCLKDIV_DIV_2POW(n) ((n) & 0x3)

/**********************************************************************
* clkpwr_pwr_ctrl register definitions
**********************************************************************/
/* Force HCLK and ARMCLK to run from PERIPH_CLK to save power */
#define CLKPWR_CTRL_FORCE_PCLK      _BIT(10)
/* SDRAM self refresh request */
#define CLKPWR_SDRAM_SELF_RFSH      _BIT(9)
/* Update SDRAM self refresh request */
#define CLKPWR_UPD_SDRAM_SELF_RFSH  _BIT(8)
/* Enable auto exit SDRAM self refresh */
#define CLKPWR_AUTO_SDRAM_SELF_RFSH _BIT(7)
/* Highcore pin level (when CLKPWR_HIGHCORE_GPIO_EN is set) */
#define CLKPWR_HIGHCORE_STATE_BIT   _BIT(5)
/* SYSCLKEN pin level (when CLKPWR_SYSCLKEN_GPIO_EN is set) */
#define CLKPWR_SYSCLKEN_STATE_BIT   _BIT(4)
/* Enable SYSCLKEN pin as a GPIO bit */
#define CLKPWR_SYSCLKEN_GPIO_EN     _BIT(3)
/* Selects direct run mode (0) or run mode (1) */
#define CLKPWR_SELECT_RUN_MODE      _BIT(2)
/* Enable Highcore pin as a GPIO bit */
#define CLKPWR_HIGHCORE_GPIO_EN     _BIT(1)
/* Enable Highcore pin as a GPIO bit */
#define CLKPWR_STOP_MODE_CTRL       _BIT(0)

/**********************************************************************
* clkpwr_pll397_ctrl register definitions
**********************************************************************/
/* Backup PLL397 lock status mask bit */
#define CLKPWR_PLL397_MSLOCK_STS   _BIT(10)
/* PLL397 bypass enable bit */
#define CLKPWR_PLL397_BYPASS       _BIT(9)
/* PLL397 charge pump biases */
/* Normal charge pump bias */
#define CLKPWR_PLL397_BIAS_NORM    0x000
/* -12.5% charge pump bias */
#define CLKPWR_PLL397_BIAS_N12_5   0x040
/* -25% charge pump bias */
#define CLKPWR_PLL397_BIAS_N25     0x080
/* -37.5% charge pump bias */
#define CLKPWR_PLL397_BIAS_N37_5   0x0C0
/* 12.5% charge pump bias */
#define CLKPWR_PLL397_BIAS_P12_5   0x100
/* 25% charge pump bias */
#define CLKPWR_PLL397_BIAS_P25     0x140
/* 37.5% charge pump bias */
#define CLKPWR_PLL397_BIAS_P37_5   0x180
/* 50% charge pump bias */
#define CLKPWR_PLL397_BIAS_P50     0x1C0
/* PLL397 charge pump bias mask */
#define CLKPWR_PLL397_BIAS_MASK    0x1C0
/* PLL397 enable/disable bit, (0) = enable, (1) = disable */
#define CLKPWR_SYSCTRL_PLL397_DIS  _BIT(1)
/* Read only status mask bit of the PLL397 oscillator lock state,
   (0) = PLL397 is not locked, (1) = PLL397 is locked */
#define CLKPWR_SYSCTRL_PLL397_STS  _BIT(0)

/**********************************************************************
* clkpwr_main_osc_ctrl register definitions
**********************************************************************/
/* Main oscillator load cap adder, adds 0 to 12.7pF, or 0.1pF per
   increment */
#define CLKPWR_MOSC_ADD_CAP(n)      (((n) & 0x7F) << 2)
/* Main oscillator load cap adder bit mask */
#define CLKPWR_MOSC_CAP_MASK        (0x7F << 2)
/* Main oscillator test mode, passes through mode */
#define CLKPWR_TEST_MODE            _BIT(1)
/* Main oscillator disable, power down mode */
#define CLKPWR_MOSC_DISABLE         _BIT(0)

/**********************************************************************
* clkpwr_sysclk_ctrl register definitions
**********************************************************************/
/* Number used by the clock switching circuitry to decide how long a
   bad phase must be present before clock switching is triggered */
#define CLKPWR_SYSCTRL_BP_TRIG(n)   (((n) & 0x3FF) << 2)
/* Mask for bad phase bits */
#define CLKPWR_SYSCTRL_BP_MASK      (0x3FF << 2)
/* (1) = Use main oscillator, (1) = use PLL397 oscillator */
#define CLKPWR_SYSCTRL_USEPLL397    _BIT(1)
/* Read only status mask bit of the select oscillator, (0) = main
   oscillator, (1) = PLL397 oscillator */
#define CLKPWR_SYSCTRL_SYSCLKMUX    _BIT(0)

/**********************************************************************
* clkpwr_lcdclk_ctrl register definitions
**********************************************************************/
/* Muxed pin configuration for TFT display and RGB444 support */
#define CLKPWR_LCDCTRL_LCDTYPE_TFT12 0x000
/* Muxed pin configuration for TFT display and RGB565 support */
#define CLKPWR_LCDCTRL_LCDTYPE_TFT16 0x040
/* Muxed pin configuration for TFT display and RGB1:555 support */
#define CLKPWR_LCDCTRL_LCDTYPE_TFT15 0x080
/* Muxed pin configuration for TFT display and RGB888 support */
#define CLKPWR_LCDCTRL_LCDTYPE_TFT24 0x0C0
/* Muxed pin configuration for STN display and 4-bit mono support */
#define CLKPWR_LCDCTRL_LCDTYPE_STN4M 0x100
/* Muxed pin configuration for STN display and 8-bit mono or color
   support */
#define CLKPWR_LCDCTRL_LCDTYPE_STN8C 0x140
/* Muxed pin configuration for DSTN display and 4-bit mono support */
#define CLKPWR_LCDCTRL_LCDTYPE_DSTN4M 0x180
/* Muxed pin configuration for DSTN display and 8-bit mono or color
   support */
#define CLKPWR_LCDCTRL_LCDTYPE_DSTN8C 0x1C0
/* Mask for LCD panel type */
#define CLKPWR_LCDCTRL_LCDTYPE_MSK 0x01C0
/* LCD clock disable (0) / enable (1) bit */
#define CLKPWR_LCDCTRL_CLK_EN      0x020
/* Macro for setting LCD prescaler, n = 1 to 32 */
#define CLKPWR_LCDCTRL_SET_PSCALE(n) ((n - 1) & 0x1F)
/* Mask for LCD prescaler */
#define CLKPWR_LCDCTRL_PSCALE_MSK  0x001F

/**********************************************************************
* clkpwr_hclkpll_ctrl register definitions
**********************************************************************/
/* Bit to start (1) or stop (0) the main HCLK PLL */
#define CLKPWR_HCLKPLL_POWER_UP    _BIT(16)
/* Main HCLK PLL CCO bypass control (0) = CCO clock to post divider,
   (1) = Bypass CCO and route PLL clock to post divider */
#define CLKPWR_HCLKPLL_CCO_BYPASS  _BIT(15)
/* Main HCLK PLL post divider bypass control (0) = use post divider,
   (1) = Bypass post divider */
#define CLKPWR_HCLKPLL_POSTDIV_BYPASS _BIT(14)
/* Main HCLK PLL feedback divider path control, (0) = feedback
   divider clocked by CCO, (1) = feedback divider clocked by FCLKOUT */
#define CLKPWR_HCLKPLL_FDBK_SEL_FCLK _BIT(13)
/* Main HCLK PLL post divider setting, for a value of n, the divider
   is 2^n, maximum value of n is 3 */
#define CLKPWR_HCLKPLL_POSTDIV_2POW(n) (((n) & 0x3) << 11)
/* Main HCLK PLL pre divider setting, for a value of n, the divider
   is (1+n), maximum value of n is 3 */
#define CLKPWR_HCLKPLL_PREDIV_PLUS1(n) (((n) & 0x3) << 9)
/* Main HCLK PLL feedback setting, for a value of n, the feedback
   is (1+n), maximum value of n is 255 */
#define CLKPWR_HCLKPLL_PLLM(n)     (((n) & 0xFF) << 1)
/* Read only status mask bit of the PLL lock state, (0) = PLL is not
   locked, (1) = PLL is locked */
#define CLKPWR_HCLKPLL_PLL_STS     _BIT(0)

/**********************************************************************
* clkpwr_adc_clk_ctrl_1 register definitions
**********************************************************************/
/* Macro for setting the ADC clock divider when the PERIPH_CLK is
   selected, n = 1 to 256, use 0 to disable */
#define CLKPWR_ADCCTRL1_RTDIV(n)   (((n) & 0xFF) << 0)
/* Clock selection bit for ADC, (0) = RTC, (1) = PERIPH_CLK */
#define CLKPWR_ADCCTRL1_PCLK_SEL   _BIT(8)

/**********************************************************************
* clkpwr_usb_ctrl register definitions
**********************************************************************/
/* USB slave HCLK clock disable (0) / enable (1) bit */
#define CLKPWR_USBCTRL_HCLK_EN     _BIT(24)
/* USB I2C enable, (0) = automatic USB I2C enable, (1) = disable (by
   driving '0' to the OE_TP_N pad */
#define CLKPWR_USBCTRL_USBI2C_EN   _BIT(23)
/* USB_DEV_NEED_CLK enable, (0) = USB_DEV_NEED_CLK not let into clock
   switch, (1) = USB_DEV_NEED_CLK let into clock switch */
#define CLKPWR_USBCTRL_USBDVND_EN  _BIT(22)
/* USB_HOST_NEED_CLK enable, (0) = USB_HOST_NEED_CLK not let into clock
   switch, (1) = USB_HOST_NEED_CLK let into clock switch */
#define CLKPWR_USBCTRL_USBHSTND_EN _BIT(21)
/* USB_DAT_VP and USB_SE0_VM pull-up added to pad */
#define CLKPWR_USBCTRL_PU_ADD      (0x0 << 19)
/* USB_DAT_VP and USB_SE0_VM bus keeper mode */
#define CLKPWR_USBCTRL_BUS_KEEPER  (0x1 << 19)
/* USB_DAT_VP and USB_SE0_VM pull-down added to pad */
#define CLKPWR_USBCTRL_PD_ADD      (0x3 << 19)
/* USB (CLKEN2) clock disable (0) / enable (1) bit */
#define CLKPWR_USBCTRL_CLK_EN2     _BIT(18)
/* USB (CLKEN1) clock disable (0) / enable (1) bit */
#define CLKPWR_USBCTRL_CLK_EN1     _BIT(17)
/* USB PLL Power up (1) / power down (0) bit */
#define CLKPWR_USBCTRL_PLL_PWRUP   _BIT(16)
/* USB PLL CCO bypass bit, (0) = use post divider, (1) = bypass */
#define CLKPWR_USBCTRL_CCO_BYPASS  _BIT(15)
/* USB PLL direct output bit, (0) = use post divider as PLL output,
   (1) = bypass post divider */
#define CLKPWR_USBCTRL_POSTDIV_BYPASS _BIT(14)
/* USB PLL feedback divider path control, (0) = feedback
   divider clocked by CCO, (1) = feedback divider clocked by FCLKOUT */
#define CLKPWR_USBCTRL_FDBK_SEL_FCLK _BIT(13)
/* USB PLL post divider setting, for a value of n, the divider is 2^n,
   maximum value of n is 3 */
#define CLKPWR_USBCTRL_POSTDIV_2POW(n) (((n) & 0x3) << 11)
/* USB PLL pre divider setting, for a value of n, the divider
   is (1+n), maximum value of n is 3 */
#define CLKPWR_USBCTRL_PREDIV_PLUS1(n) (((n) & 0x3) << 9)
/* USB PLL feedback setting, for a value of n, the feedback
   is (1+n), maximum value of n is 255 */
#define CLKPWR_USBCTRL_FDBK_PLUS1(n) (((n) & 0xFF) << 1)
/* Read only status mask bit of the USB PLL lock state, (0) = PLL is
   not locked, (1) = PLL is locked */
#define CLKPWR_USBCTRL_PLL_STS     _BIT(0)

/**********************************************************************
* clkpwr_sdramclk_ctrl register definitions
**********************************************************************/
/* SDRAM RAM_CLK fast slew rate control selection bit */
#define CLKPWR_SDRCLK_FASTSLEW_CLK _BIT(22)
/* SDRAM grouping fast slew rate control selection bit */
#define CLKPWR_SDRCLK_FASTSLEW     _BIT(21)
/* SDRAM data fast slew rate control selection bit */
#define CLKPWR_SDRCLK_FASTSLEW_DAT _BIT(20)
/* SDRAM/DDR controller reset bit */
#define CLKPWR_SDRCLK_SW_DDR_RESET _BIT(19)
/* Select HCLK delay calibration value, n = 0 to 31 at .25nS per tick */
#define CLKPWR_SDRCLK_HCLK_DLY(n)  (((n) & 0x1F) << 14)
/* SDRAM/DDR delay circuitry address status bit */
#define CLKPWR_SDRCLK_DLY_ADDR_STS _BIT(13)
/* Sensitivity factor for DDR SDRAM cal, n = 0 to 7 */
#define CLKPWR_SDRCLK_SENS_FACT(n) (((n) & 0x7) << 10)
/* Use calibrated settings for DDR SDRAM bit */
#define CLKPWR_SDRCLK_USE_CAL      _BIT(9)
/* Perform a DDR delay calibration bit */
#define CLKPWR_SDRCLK_DO_CAL       _BIT(8)
/* Enable auto DDR cal on RTC tick bit */
#define CLKPWR_SDRCLK_CAL_ON_RTC   _BIT(7)
/* Select DQS input delay value, n = 0 to 31 at .25nS per tick */
#define CLKPWR_SDRCLK_DQS_DLY(n)   (((n) & 0x1F) << 2)
/* Use DDR (1) or SDRAM (0) bit */
#define CLKPWR_SDRCLK_USE_DDR      _BIT(1)
/* SDRAM/DDR clock disable bit */
#define CLKPWR_SDRCLK_CLK_DIS      _BIT(0)

/**********************************************************************
* clkpwr_ssp_blk_ctrl register definitions
**********************************************************************/
/* SSP1 RX DMA selection, (0) = SSP1RX not connected/SPI2 connected,
   (1) = SSP1RX connected/SPI2 not connected */
#define CLKPWR_SSPCTRL_DMA_SSP1RX  _BIT(5)
/* SSP1 TX DMA selection, (0) = SSP1TX not connected/SPI1 connected,
   (1) = SSP1TX connected/SPI1 not connected */
#define CLKPWR_SSPCTRL_DMA_SSP1TX  _BIT(4)
/* SSP0 RX DMA selection, (0) = SSP1RX not connected/SPI2 connected,
   (1) = SSP1RX connected/SPI3 not connected */
#define CLKPWR_SSPCTRL_DMA_SSP0RX  _BIT(3)
/* SSP0 TX DMA selection, (0) = SSP1TX not connected/SPI1 connected,
   (1) = SSP1TX connected/SPI4 not connected */
#define CLKPWR_SSPCTRL_DMA_SSP0TX  _BIT(2)
/* SSP0 clock disable (0) / enable (1) bit */
#define CLKPWR_SSPCTRL_SSPCLK1_EN  _BIT(1)
/* SSP0 clock disable (0) / enable (1) bit */
#define CLKPWR_SSPCTRL_SSPCLK0_EN  _BIT(0)

/**********************************************************************
* clkpwr_i2s_clk_ctrl register definitions
**********************************************************************/
/* I2S1 TX clock mode, (0) = TX clock drives TX timing, (1) = RX clock
   drives TX timing */
#define CLKPWR_I2SCTRL_I2S1_RX_FOR_TX _BIT(6)
/* I2S1 RX clock mode, (0) = RX clock drives RX timing, (1) = TX clock
   drives RX timing */
#define CLKPWR_I2SCTRL_I2S1_TX_FOR_RX _BIT(5)
/* I2S1 DMA muxing, (0) = HS-UAT7 uses DMA, (1) = I2S1 uses DMA */
#define CLKPWR_I2SCTRL_I2S1_USE_DMA _BIT(4)
/* I2S0 TX clock mode, (0) = TX clock drives TX timing, (1) = RX clock
   drives TX timing */
#define CLKPWR_I2SCTRL_I2S0_RX_FOR_TX _BIT(3)
/* I2S0 RX clock mode, (0) = RX clock drives RX timing, (1) = TX clock
   drives RX timing */
#define CLKPWR_I2SCTRL_I2S0_TX_FOR_RX _BIT(2)
/* I2S1 clock disable (0) / enable (1) bit */
#define CLKPWR_I2SCTRL_I2SCLK1_EN  _BIT(1)
/* I2S0 clock disable (0) / enable (1) bit */
#define CLKPWR_I2SCTRL_I2SCLK0_EN  _BIT(0)

/**********************************************************************
* clkpwr_ms_ctrl register definitions
**********************************************************************/
/* Disable SD pins(1) or enable (0) */
#define CLKPWR_MSCARD_MSDIO_PIN_DIS _BIT(10)
/* MSSDIO pullup enable (1) / disable (0) */
#define CLKPWR_MSCARD_MSDIO_PU_EN  _BIT(9)
/* MSSDIO data 2 and 3 pullup disable (1) / enable (0) */
#define CLKPWR_MSCARD_MSDIO23_DIS  _BIT(8)
/* MSSDIO data 1 pullup disable (1) / enable (0) */
#define CLKPWR_MSCARD_MSDIO1_DIS   _BIT(7)
/* MSSDIO data 0 pullup disable (1) / enable (0) */
#define CLKPWR_MSCARD_MSDIO0_DIS   _BIT(6)
/* SDCard clock disable (0) / enable (1) */
#define CLKPWR_MSCARD_SDCARD_EN    _BIT(5)
/* SDCard clock divider = (ARM_PLL clock / n) Hz, n = 1 to 15,
   disabled when n = 0*/
#define CLKPWR_MSCARD_SDCARD_DIV(n) ((n) & 0xF)

/**********************************************************************
* clkpwr_macclk_ctrl register definitions
**********************************************************************/
/* Disables ethernet MAC pins */
#define CLKPWR_MACCTRL_NO_ENET_PIS 0x00
/* Ethernet MAC pins setup for MII */
#define CLKPWR_MACCTRL_USE_MII_PINS 0x08
/* Ethernet MAC pins setup for RMII */
#define CLKPWR_MACCTRL_USE_RMII_PINS 0x18
/* Mask for MAC pins selection */
#define CLKPWR_MACCTRL_PINS_MSK    0x18
/* Ethernet MAC DMA clock disable (0) / enable (1) bit */
#define CLKPWR_MACCTRL_DMACLK_EN   _BIT(2)
/* Ethernet MAC MMIO clock disable (0) / enable (1) bit */
#define CLKPWR_MACCTRL_MMIOCLK_EN  _BIT(1)
/* Ethernet MAC host registers clock disable (0) / enable (1) bit */
#define CLKPWR_MACCTRL_HRCCLK_EN   _BIT(0)

/**********************************************************************
* clkpwr_test_clk_sel register definitions
**********************************************************************/
/* Route PERIPH_CLK to TEST_CLK1 pin */
#define CLKPWR_TESTCLK1_SEL_PERCLK (0x0 << 5)
/* Route RTC to TEST_CLK1 pin */
#define CLKPWR_TESTCLK1_SEL_RTC    (0x1 << 5)
/* Route Main oscillator clock to TEST_CLK1 pin */
#define CLKPWR_TESTCLK1_SEL_MOSC   (0x2 << 5)
/* TEST_CLK1 pin signal selection mask */
#define CLKPWR_TESTCLK1_SEL_MASK   (0x3 << 5)
/* TEST_CLK1 output select, (0) = connected to GPO_00, (1) = use
   selected test 1 clock */
#define CLKPWR_TESTCLK_TESTCLK1_EN _BIT(4)
/* Route PERIPH_CLK to TEST_CLK2 pin */
#define CLKPWR_TESTCLK2_SEL_HCLK   (0x0 << 1)
/* Route PERIPH_CLK to TEST_CLK2 pin */
#define CLKPWR_TESTCLK2_SEL_PERCLK (0x1 << 1)
/* Route USB_CLK to TEST_CLK2 pin */
#define CLKPWR_TESTCLK2_SEL_USBCLK (0x2 << 1)
/* Route Main oscillator clock to TEST_CLK2 pin */
#define CLKPWR_TESTCLK2_SEL_MOSC   (0x5 << 1)
/* Route PLL397 to TEST_CLK2 pin */
#define CLKPWR_TESTCLK2_SEL_PLL397 (0x7 << 1)
/* TEST_CLK2 pin signal selection mask */
#define CLKPWR_TESTCLK2_SEL_MASK   (0x7 << 1)
/* TEST_CLK2 output select, (0) = TEST_CLK2 turned off, (1) = use
   selected test 2 clock */
#define CLKPWR_TESTCLK_TESTCLK2_EN _BIT(0)

/**********************************************************************
* clkpwr_sw_int register definitions
**********************************************************************/
/* Macro for loading the SW interrupt argument value and generating
   and interrupt */
#define CLKPWR_SW_INT(n)           (_BIT(0) | (((n) & 0x7F) << 1))
/* Macro for reading the SW interrupt argument */
#define CLKPWR_SW_GET_ARG(n)       (((n) & 0xFE) >> 1)

/**********************************************************************
* clkpwr_i2c_clk_ctrl register definitions
**********************************************************************/
/* Driver strength for USB_I2C clock and data, (0) = low driver,
   (1) = high drive */
#define CLKPWR_I2CCLK_USBI2CHI_DRIVE _BIT(4)
/* Driver strength for I2C2 clock and data, (0) = low driver,
   (1) = high drive */
#define CLKPWR_I2CCLK_I2C2HI_DRIVE _BIT(3)
/* Driver strength for I2C1 clock and data, (0) = low driver,
   (1) = high drive */
#define CLKPWR_I2CCLK_I2C1HI_DRIVE _BIT(2)
/* Clock enable for I2C2, (0) = disable, (1) = enable */
#define CLKPWR_I2CCLK_I2C2CLK_EN   _BIT(1)
/* Clock enable for I2C1, (0) = disable, (1) = enable */
#define CLKPWR_I2CCLK_I2C1CLK_EN   _BIT(0)

/**********************************************************************
* clkpwr_key_clk_ctrl register definitions
**********************************************************************/
/* Key scanner clock disable (0) / enable (1) bit */
#define CLKPWR_KEYCLKCTRL_CLK_EN   0x1

/**********************************************************************
* clkpwr_adc_clk_ctrl register definitions
**********************************************************************/
/* ADC 32KHz clock disable (0) / enable (1) bit */
#define CLKPWR_ADC32CLKCTRL_CLK_EN 0x1

/**********************************************************************
* clkpwr_pwm_clk_ctrl register definitions
**********************************************************************/
/* PWM2 clock frequency = CLKIN / n, n = 1 to 15, disabled when
   n = 0 */
#define CLKPWR_PWMCLK_PWM2_DIV(n)  (((n) & 0xF) << 8)
/* PWM1 clock frequency = CLKIN / n, n = 1 to 15, disabled when
   n = 0 */
#define CLKPWR_PWMCLK_PWM1_DIV(n)  (((n) & 0xF) << 4)
/* Selects PWM2 clock source, (0) = 32KHz clock, (1) = PERIPH_CLK */
#define CLKPWR_PWMCLK_PWM2SEL_PCLK 0x8
/* Enables PWM2 clock, (0) = disable, (1) = enable */
#define CLKPWR_PWMCLK_PWM2CLK_EN   0x4
/* Selects PWM1 clock source, (0) = 32KHz clock, (1) = PERIPH_CLK */
#define CLKPWR_PWMCLK_PWM1SEL_PCLK 0x2
/* Enables PWM1 clock, (0) = disable, (1) = enable */
#define CLKPWR_PWMCLK_PWM1CLK_EN   0x1

/**********************************************************************
* clkpwr_timer_clk_ctrl register definitions
**********************************************************************/
/* High speed timer clock enable, (0) = disable, (1) = enable */
#define CLKPWR_PWMCLK_HSTIMER_EN   0x2
/* Watchdog timer clock enable, (0) = disable, (1) = enable */
#define CLKPWR_PWMCLK_WDOG_EN      0x1

/**********************************************************************
* clkpwr_timers_pwms_clk_ctrl_1 register definitions
**********************************************************************/
/* Timer 3 clock enable, (0) = disable, (1) = enable */
#define CLKPWR_TMRPWMCLK_TIMER3_EN 0x20
/* Timer 2 clock enable, (0) = disable, (1) = enable */
#define CLKPWR_TMRPWMCLK_TIMER2_EN 0x10
/* Timer 1 clock enable, (0) = disable, (1) = enable */
#define CLKPWR_TMRPWMCLK_TIMER1_EN 0x08
/* Timer 0 clock enable, (0) = disable, (1) = enable */
#define CLKPWR_TMRPWMCLK_TIMER0_EN 0x04
/* PWM 4 clock enable, (0) = disable, (1) = enable */
#define CLKPWR_TMRPWMCLK_PWM4_EN   0x02
/* PWM 3 clock enable, (0) = disable, (1) = enable */
#define CLKPWR_TMRPWMCLK_PWM3_EN   0x01

/**********************************************************************
* clkpwr_spi_clk_ctrl register definitions
**********************************************************************/
/* SPI2 DATIO output level is CLKPWR_SPICLK_USE_SPI2 is 0 */
#define CLKPWR_SPICLK_SET_SPI2DATIO 0x80
/* SPI2 CLK output level is CLKPWR_SPICLK_USE_SPI2 is 0 */
#define CLKPWR_SPICLK_SET_SPI2CLK  0x40
/* SPI2 DATIO and CLK output control, (0) = GPO values in bits 6 and
   7, (1) = controlled by the SPI2 block */
#define CLKPWR_SPICLK_USE_SPI2     0x20
/* SPI2 clock enable, (0) = disable, (1) = enable */
#define CLKPWR_SPICLK_SPI2CLK_EN   0x10
/* SPI1 DATIO output level is CLKPWR_SPICLK_USE_SPI1 is 0 */
#define CLKPWR_SPICLK_SET_SPI1DATIO 0x08
/* SPI1 CLK output level is CLKPWR_SPICLK_USE_SPI1 is 0 */
#define CLKPWR_SPICLK_SET_SPI1CLK  0x04
/* SPI1 DATIO and CLK output control, (0) = GPO values in bits 2 and
   3, (1) = controlled by the SPI1 block */
#define CLKPWR_SPICLK_USE_SPI1     0x02
/* SPI1 clock enable, (0) = disable, (1) = enable */
#define CLKPWR_SPICLK_SPI1CLK_EN   0x01

/**********************************************************************
* clkpwr_nand_clk_ctrl register definitions
**********************************************************************/
/* NAND FLASH controller interrupt select, (0) = SLC, (1) = MLC */
#define CLKPWR_NANDCLK_INTSEL_MLC  0x20
/* Enable DMA_REQ on NAND_RnB for MLC */
#define CLKPWR_NANDCLK_DMA_RNB     0x10
/* Enable DMA_REQ on NAND_INT for MLC */
#define CLKPWR_NANDCLK_DMA_INT     0x08
/* NAND FLASH controller select, (0) = MLC, (1) = SLC */
#define CLKPWR_NANDCLK_SEL_SLC     0x04
/* NAND FLASH MLC clock enable, (0) = disable, (1) = enable */
#define CLKPWR_NANDCLK_MLCCLK_EN   0x02
/* NAND FLASH SLC clock enable, (0) = disable, (1) = enable */
#define CLKPWR_NANDCLK_SLCCLK_EN   0x01

/**********************************************************************
* clkpwr_uart3_clk_ctrl, clkpwr_uart4_clk_ctrl, clkpwr_uart5_clk_ctrl
* and clkpwr_uart6_clk_ctrl register definitions
**********************************************************************/
/* Macro for loading UART 'Y' divider value */
#define CLKPWR_UART_Y_DIV(y)       ((y) & 0xFF)
/* Macro for loading UART 'X' divider value */
#define CLKPWR_UART_X_DIV(x)       (((x) & 0xFF) << 8)
/* Bit for using HCLK as the UART X/Y divider input, or PERIPH_CLK */
#define CLKPWR_UART_USE_HCLK       _BIT(16)

/**********************************************************************
* clkpwr_irda_clk_ctrl register definitions
**********************************************************************/

/* Macro for loading IRDA 'Y' divider value */
#define CLKPWR_IRDA_Y_DIV(y)       ((y) & 0xFF)
/* Macro for loading IRDA 'X' divider value */
#define CLKPWR_IRDA_X_DIV(x)       (((x) & 0xFF) << 8)

/**********************************************************************
* clkpwr_uart_clk_ctrl register definitions
**********************************************************************/
/* UART6 clock disable (0) / enable (1) bit */
#define CLKPWR_UARTCLKCTRL_UART6_EN _BIT(3)
/* UART5 clock disable (0) / enable (1) bit */
#define CLKPWR_UARTCLKCTRL_UART5_EN _BIT(2)
/* UART4 clock disable (0) / enable (1) bit */
#define CLKPWR_UARTCLKCTRL_UART4_EN _BIT(1)
/* UART3 clock disable (0) / enable (1) bit */
#define CLKPWR_UARTCLKCTRL_UART3_EN _BIT(0)

/**********************************************************************
* clkpwr_dmaclk_ctrl register definitions
**********************************************************************/
/* DMA clock disable (0) / enable (1) bit */
#define CLKPWR_DMACLKCTRL_CLK_EN   0x1

/**********************************************************************
* clkpwr_autoclock register definitions
**********************************************************************/
/* (0) = enable USB autoclock, (1) = always clock */
#define CLKPWR_AUTOCLK_USB_EN      0x40
/* (0) = enable IRAM autoclock, (1) = always clock */
#define CLKPWR_AUTOCLK_IRAM_EN     0x02
/* (0) = enable IROM autoclock, (1) = always clock */
#define CLKPWR_AUTOCLK_IROM_EN     0x01

#endif /* LPC32XX_CLKPWR_H */
