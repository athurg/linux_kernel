/*
 *  linux/arch/arm/mach-lpc32xx/serial-lpc32xx.c
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

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <linux/serial_8250.h>
#include <linux/clk.h>

#include <mach/platform.h>
#include <mach/io.h>
#include <mach/lpc32xx_clkpwr.h>
#include <mach/lpc32xx_uart.h>
#include "sys-lpc32xx.h"

#define UARTCTL_CLKMODE(x)	(x + 0x04)

/* Standard 8250/16550 compatible serial ports */
struct plat_serial8250_port serial_std_platform_data[] =
{
#if defined (CONFIG_MACH_LPC32XX_UART5_ENABLE)
	{
		.membase        = (void *) io_p2v(UART5_BASE),
		.mapbase        = UART5_BASE,
		.irq		= IRQ_UART_IIR5,
		.uartclk	= MAIN_OSC_FREQ,
		.regshift	= 2,
		.iotype		= UPIO_MEM32,
		.flags		= UPF_BOOT_AUTOCONF | UPF_BUGGY_UART | UPF_SKIP_TEST,
	},
#endif
#if defined (CONFIG_MACH_LPC32XX_UART3_ENABLE)
	{
		.membase	= (void *) io_p2v(UART3_BASE),
		.mapbase        = UART3_BASE,
		.irq		= IRQ_UART_IIR3,
		.uartclk	= MAIN_OSC_FREQ,
		.regshift	= 2,
		.iotype		= UPIO_MEM32,
		.flags		= UPF_BOOT_AUTOCONF | UPF_BUGGY_UART | UPF_SKIP_TEST,
	},
#endif
#if defined (CONFIG_MACH_LPC32XX_UART4_ENABLE)
	{
		.membase	= (void *) io_p2v(UART4_BASE),
		.mapbase        = UART4_BASE,
		.irq		= IRQ_UART_IIR4,
		.uartclk	= MAIN_OSC_FREQ,
		.regshift	= 2,
		.iotype		= UPIO_MEM32,
		.flags		= UPF_BOOT_AUTOCONF | UPF_BUGGY_UART | UPF_SKIP_TEST,
	},
#endif
#if defined (CONFIG_MACH_LPC32XX_UART6_ENABLE)
	{
		.membase	= (void *) io_p2v(UART6_BASE),
		.mapbase        = UART6_BASE,
		.irq		= IRQ_UART_IIR6,
		.uartclk	= MAIN_OSC_FREQ,
		.regshift	= 2,
		.iotype		= UPIO_MEM32,
		.flags		= UPF_BOOT_AUTOCONF | UPF_BUGGY_UART | UPF_SKIP_TEST,
	},
#endif
	{ },
};

struct platform_device serial_std_platform_device = {
	.name			= "serial8250",
	.id			= 0,
	.dev			= {
		.platform_data	= serial_std_platform_data,
	},
};

/* High speed serial ports */
struct uart_port serial_hspd_platform_data[] =
{
#if defined (CONFIG_MACH_LPC32XX_HSUART1_ENABLE)
	{
		.membase	= (void *) io_p2v(HS_UART1_BASE),
		.mapbase        = HS_UART1_BASE,
		.irq		= IRQ_UART_IIR1,
		.uartclk	= MAIN_OSC_FREQ,
		.regshift	= 2,
		.iotype		= UPIO_MEM32,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	},
#endif
#if defined (CONFIG_MACH_LPC32XX_HSUART2_ENABLE)
	{
		.membase	= (void *) io_p2v(HS_UART2_BASE),
		.mapbase        = HS_UART2_BASE,
		.irq		= IRQ_UART_IIR2,
		.uartclk	= MAIN_OSC_FREQ,
		.regshift	= 2,
		.iotype		= UPIO_MEM32,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	},
#endif
#if defined (CONFIG_MACH_LPC32XX_HSUART7_ENABLE)
	{
		.membase	= (void *) io_p2v(HS_UART7_BASE),
		.mapbase        = HS_UART7_BASE,
		.irq		= IRQ_UART_IIR7,
		.uartclk	= MAIN_OSC_FREQ,
		.regshift	= 2,
		.iotype		= UPIO_MEM32,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
	},
#endif
	{ },
};

struct platform_device serial_hspd_platform_device = {
	.name			= "lpc32xx_hsuart",
	.id			= 0,
	.dev			= {
		.platform_data	= serial_hspd_platform_data,
	},
};

void serial_init(void)
{
	u32 tmp, rate;
	struct clk *clk;
	int i;

	/* Get the current peripheral clock rate. This is the base clock for the
	   UART block and may vary based on selected oscillator or rate */
	clk = clk_get(NULL, "pclk_ck");
	if (IS_ERR(clk))
	{
		/* Just use main oscillator rate */
		rate = MAIN_OSC_FREQ;
	}
	else
	{
		rate = clk_get_rate(clk);
		clk_put(clk);
	}
	if (rate == 0)
	{
		rate = MAIN_OSC_FREQ;
	}

	/* Update rate for the UART */
	for (i = 0; i < ARRAY_SIZE(serial_std_platform_data); i++)
	{
		serial_std_platform_data[i].uartclk = rate;
	}
	for (i = 0; i < ARRAY_SIZE(serial_hspd_platform_data); i++)
	{
		serial_hspd_platform_data[i].uartclk = rate;
		serial_hspd_platform_data[i].line = i;
	}

	/* All pre-UART block dividers set to 1 */
	__raw_writel(0x0101, CLKPWR_UART3_CLK_CTRL(CLKPWR_IOBASE));
	__raw_writel(0x0101, CLKPWR_UART4_CLK_CTRL(CLKPWR_IOBASE));
	__raw_writel(0x0101, CLKPWR_UART5_CLK_CTRL(CLKPWR_IOBASE));
	__raw_writel(0x0101, CLKPWR_UART6_CLK_CTRL(CLKPWR_IOBASE));

	/* Enable UART clocking in clock and power control */
	tmp = 0;
#if defined (CONFIG_MACH_LPC32XX_UART5_ENABLE)
	tmp |= CLKPWR_UARTCLKCTRL_UART5_EN;
#endif
#if defined (CONFIG_MACH_LPC32XX_UART3_ENABLE)
	tmp |= CLKPWR_UARTCLKCTRL_UART3_EN;
#endif
#if defined (CONFIG_MACH_LPC32XX_UART4_ENABLE)
	tmp |= CLKPWR_UARTCLKCTRL_UART4_EN;
#endif
#if defined (CONFIG_MACH_LPC32XX_UART6_ENABLE)
	tmp |= CLKPWR_UARTCLKCTRL_UART6_EN;
#endif
	__raw_writel(tmp, CLKPWR_UART_CLK_CTRL(CLKPWR_IOBASE));

	/* Setup UART clock modes, disable autoclock */
	tmp = 0;
#if defined (CONFIG_MACH_LPC32XX_UART5_ENABLE)
	tmp |= UART_CLKMODE_LOAD(UART_CLKMODE_ON, 5);
#endif
#if defined (CONFIG_MACH_LPC32XX_UART3_ENABLE)
	tmp |= UART_CLKMODE_LOAD(UART_CLKMODE_ON, 3);
#endif
#if defined (CONFIG_MACH_LPC32XX_UART4_ENABLE)
	tmp |= UART_CLKMODE_LOAD(UART_CLKMODE_ON, 4);
#endif
#if defined (CONFIG_MACH_LPC32XX_UART6_ENABLE)
	tmp |= UART_CLKMODE_LOAD(UART_CLKMODE_ON, 6);
#endif

	/* Enable UART clocks, disable autoclock */
	__raw_writel(tmp, UARTCTL_CLKMODE(io_p2v(UART_CTRL_BASE)));

#if defined (CONFIG_MACH_LPC32XX_UART5_ENABLE)
	/* Send a NULL to fix the UART HW bug */
	__raw_writel(0xC1, UART_IIR_FCR(io_p2v(UART5_BASE)));
	__raw_writel(0x00, UART_DLL_FIFO(io_p2v(UART5_BASE)));
	rate = 64;
	while (rate--) {
		tmp = __raw_readl(UART_DLL_FIFO(io_p2v(UART5_BASE)));
	}
	__raw_writel(0, UART_IIR_FCR(io_p2v(UART5_BASE)));
#endif

#if defined (CONFIG_MACH_LPC32XX_UART3_ENABLE)
	/* Send a NULL to fix the UART HW bug */
	__raw_writel(0xC1, UART_IIR_FCR(io_p2v(UART3_BASE)));
	__raw_writel(0x00, UART_DLL_FIFO(io_p2v(UART3_BASE)));
	rate = 64;
	while (rate--) {
		tmp = __raw_readl(UART_DLL_FIFO(io_p2v(UART3_BASE)));
	}
	__raw_writel(0, UART_IIR_FCR(io_p2v(UART3_BASE)));
#endif

#if defined (CONFIG_MACH_LPC32XX_UART4_ENABLE)
	/* Send a NULL to fix the UART HW bug */
	__raw_writel(0xC1, UART_IIR_FCR(io_p2v(UART4_BASE)));
	__raw_writel(0x00, UART_DLL_FIFO(io_p2v(UART4_BASE)));
	rate = 64;
	while (rate--) {
		tmp = __raw_readl(UART_DLL_FIFO(io_p2v(UART4_BASE)));
	}
	__raw_writel(0, UART_IIR_FCR(io_p2v(UART4_BASE)));
#endif

#if defined (CONFIG_MACH_LPC32XX_UART6_ENABLE)
	/* Send a NULL to fix the UART HW bug */
	__raw_writel(0xC1, UART_IIR_FCR(io_p2v(UART6_BASE)));
	__raw_writel(0x00, UART_DLL_FIFO(io_p2v(UART6_BASE)));
	rate = 64;
	while (rate--) {
		tmp = __raw_readl(UART_DLL_FIFO(io_p2v(UART6_BASE)));
	}
	__raw_writel(0, UART_IIR_FCR(io_p2v(UART6_BASE)));
#endif
}

