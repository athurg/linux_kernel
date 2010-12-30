/*
 *  linux/arch/arm/mach-lpc32xx/sys-lpc32xx.h
 *
 *  Copyright (C) 2008 NXP Semiconductors
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

#ifndef __SYS_LPC32XX_H
#define __SYS_PLC32XX_H

#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>

#include <mach/platform.h>
#include <mach/board.h>

#define TIMER0_IOBASE io_p2v(TIMER0_BASE)
#define TIMER1_IOBASE io_p2v(TIMER1_BASE)
#define CLKPWR_IOBASE io_p2v(CLK_PM_BASE)
#define GPIO_IOBASE io_p2v(GPIO_BASE)

/*
 * Board specific functions
 */
extern void board_init(void);

#if defined (CONFIG_ENABLE_BOARD_LED_TICK)
#if defined (CONFIG_MACH_PHY3250)
#define LEDTICK { \
	static int blink = 0; \
	static int tick1 = 0; \
	tick1++; \
	if (tick1 > HZ) \
	{ \
		tick1 = 0; \
		blink = 1 - blink; \
		if (blink == 0) { \
			__raw_writel(OUTP_STATE_GPO(1), GPIO_P3_OUTP_SET(GPIO_IOBASE)); \
		} \
		else { \
			__raw_writel(OUTP_STATE_GPO(1), GPIO_P3_OUTP_CLR(GPIO_IOBASE)); \
		} \
	} \
}
#endif
#if defined (CONFIG_MACH_ARM9DIMM3250)
#define LEDTICK { \
	static int blink = 0; \
	static int tick1 = 0; \
	tick1++; \
	if (tick1 > HZ) \
	{ \
		tick1 = 0; \
		blink = 1 - blink; \
		if (blink == 0) { \
			__raw_writel(OUTP_STATE_GPO(3), GPIO_P3_OUTP_SET(GPIO_IOBASE)); \
		} \
		else { \
			__raw_writel(OUTP_STATE_GPO(3), GPIO_P3_OUTP_CLR(GPIO_IOBASE)); \
		} \
	} \
}
#endif
#endif

#if !defined LEDTICK
#define LEDTICK {}
#endif

/* Chip specific structures and functions */
extern struct platform_device serial_std_platform_device;
extern struct platform_device serial_hspd_platform_device;
extern struct sys_timer lpc32xx_timer;
extern void lpc32xx_init_irq (void);
extern void serial_init(void);
extern void __init lpc32xx_init (void);
extern void __init lpc32xx_map_io(void);
extern int __init clk_init(void);
extern void lpc32xx_watchdog_reset(void);

#endif

