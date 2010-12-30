/*
 *  linux/arch/arm/mach-lpc32xx/arch-lpc32xx.c
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

#include <linux/tty.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-pnx.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/dma-mapping.h>
#include <linux/sysdev.h>
#include <linux/amba/bus.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>
#include <asm/mach/mmc.h>
#include <mach/platform.h>

#include <mach/lpc32xx_gpio.h>
#include <mach/lpc32xx_clkpwr.h>
#include <mach/lpc32xx_uart.h>
#include <mach/clock.h>
#include <mach/board.h>
#include <mach/i2c.h>
#include "sys-lpc32xx.h"

#define CLKPWR_IOBASE io_p2v(CLK_PM_BASE)
#define GPIO_IOBASE io_p2v(GPIO_BASE)

#if defined(CONFIG_LPC32XX_WATCHDOG)
/*
 * Watchdog timer resources
 */
static struct resource watchdog_resources[] = {
	[0] = {
		.start = WDTIM_BASE,
		.end = WDTIM_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};
static struct platform_device watchdog_device = {
	.name = "watchdog",
	.id = -1,
	.num_resources = ARRAY_SIZE(watchdog_resources),
	.resource = watchdog_resources,
};
#endif

#if defined (CONFIG_RTC_DRV_LPC32XX)
/*
 * RTC resources
 */
static struct resource rtc_resources[] = {
	[0] = {
		.start = RTC_BASE,
		.end = RTC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_RTC,
		.flags = IORESOURCE_IRQ,
	},
};
static struct platform_device rtc_device = {
	.name =  "rtc-lpc32xx",
	.id = -1,
	.num_resources = ARRAY_SIZE(rtc_resources),
	.resource = rtc_resources,
};
#endif


#if defined (CONFIG_MACH_LPC32XX_I2C0_ENABLE) || defined (CONFIG_MACH_LPC32XX_I2C1_ENABLE) || defined (CONFIG_MACH_LPC32XX_USBOTG_I2C_ENABLE)
/*
 * I2C resources
 */
static void clock_get_name(char *name, int id)
{
	snprintf(name, 10, "i2c%d_ck", id);
}
static int set_clock_run(struct platform_device *pdev)
{
	struct clk *clk;
	char name[10];
	int retval = 0;

	clock_get_name(name, pdev->id);
	clk = clk_get(&pdev->dev, name);
	if (!IS_ERR(clk)) {
		clk_set_rate(clk, 1);
		clk_put(clk);
	} else
		retval = -ENOENT;

	return retval;
}
static int set_clock_stop(struct platform_device *pdev)
{
	struct clk *clk;
	char name[10];
	int retval = 0;

	clock_get_name(name, pdev->id);
	clk = clk_get(&pdev->dev, name);
	if (!IS_ERR(clk)) {
		clk_set_rate(clk, 0);
		clk_put(clk);
	} else
		retval = -ENOENT;

	return retval;
}
static int i2c_lpc32xx_suspend(struct platform_device *pdev, pm_message_t state)
{
	int retval = 0;
#ifdef CONFIG_PM
	retval = set_clock_run(pdev);
#endif
	return retval;
}
static int i2c_lpc32xx_resume(struct platform_device *pdev)
{
	int retval = 0;
#ifdef CONFIG_PM
	retval = set_clock_run(pdev);
#endif
	return retval;
}
static u32 calculate_input_freq(struct platform_device *pdev)
{
	struct clk *clk;
	char name[10];
	u32 clkrate = MAIN_OSC_FREQ;

	clock_get_name(name, pdev->id);
	clk = clk_get(&pdev->dev, name);
	if (!IS_ERR(clk)) {
		/* Just get the parent rate, as the I2C driver hasn't
		   actually enabled the clock prior to querying it, so
		   it will always return 0! */
		clkrate = clk_get_rate(clk->parent);
		clk_put(clk);
	}

	/* Unlike other drivers, the PNX driver requires the rate
	   in MHz */
	clkrate  = clkrate / (1000 * 1000);

	return clkrate;
}
#endif
#if defined (CONFIG_MACH_LPC32XX_I2C0_ENABLE)
static struct i2c_pnx_algo_data lpc32xx_algo_data0 = {
	.base = I2C1_BASE,
	.irq = IRQ_I2C_1,
};
static struct i2c_adapter lpc32xx_adapter0 = {
	.name = I2C_CHIP_NAME "0",
	.algo_data = &lpc32xx_algo_data0,
};
static struct i2c_pnx_data i2c0_data = {
	.suspend = i2c_lpc32xx_suspend,
	.resume = i2c_lpc32xx_resume,
	.calculate_input_freq = calculate_input_freq,
	.set_clock_run = set_clock_run,
	.set_clock_stop = set_clock_stop,
	.adapter = &lpc32xx_adapter0,
};
static struct platform_device i2c0_device = {
	.name = "pnx-i2c",
	.id = 0,
	.dev = {
		.platform_data = &i2c0_data,
	},
};
#endif
#if defined (CONFIG_MACH_LPC32XX_I2C1_ENABLE)
static struct i2c_pnx_algo_data lpc32xx_algo_data1 = {
	.base = I2C2_BASE,
	.irq = IRQ_I2C_2,
};
static struct i2c_adapter lpc32xx_adapter1 = {
	.name = I2C_CHIP_NAME "1",
	.algo_data = &lpc32xx_algo_data1,
};
static struct i2c_pnx_data i2c1_data = {
	.suspend = i2c_lpc32xx_suspend,
	.resume = i2c_lpc32xx_resume,
	.calculate_input_freq = calculate_input_freq,
	.set_clock_run = set_clock_run,
	.set_clock_stop = set_clock_stop,
	.adapter = &lpc32xx_adapter1,
};
static struct platform_device i2c1_device = {
	.name = "pnx-i2c",
	.id = 1,
	.dev = {
		.platform_data = &i2c1_data,
	},
};
#endif
#if defined (CONFIG_MACH_LPC32XX_USBOTG_I2C_ENABLE)
static struct i2c_pnx_algo_data lpc32xx_algo_data2 = {
	.base = OTG_I2C_BASE,
	.irq = IRQ_USB_I2C,
};
static struct i2c_adapter lpc32xx_adapter2 = {
	.name = "USB-I2C",
	.algo_data = &lpc32xx_algo_data2,
};
static struct i2c_pnx_data i2c2_data = {
	.suspend = i2c_lpc32xx_suspend,
	.resume = i2c_lpc32xx_resume,
	.calculate_input_freq = calculate_input_freq,
	.set_clock_run = set_clock_run,
	.set_clock_stop = set_clock_stop,
	.adapter = &lpc32xx_adapter2,
};
static struct platform_device i2c2_device = {
	.name = "pnx-i2c",
	.id = 2,
	.dev = {
		.platform_data = &i2c2_data,
	},
};
#endif

#if defined (CONFIG_USB_OHCI_HCD)
/* The dmamask must be set for OHCI to work */
static u64 ohci_dmamask = ~(u32) 0;
static struct resource ohci_resources[] = {
	{
		.start = IO_ADDRESS(USB_BASE),
		.end = IO_ADDRESS(USB_BASE + 0x100),
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_USB_HOST,
		.flags = IORESOURCE_IRQ,
	},
};
static struct platform_device ohci_device = {
	.name = "usb-ohci",
	.id = -1,
	.dev = {
		.dma_mask = &ohci_dmamask,
		.coherent_dma_mask = 0xFFFFFFFF,
		},
	.num_resources = ARRAY_SIZE(ohci_resources),
	.resource = ohci_resources,
};
#endif

static struct platform_device* lpc32xx_devs[] __initdata = {
	&serial_std_platform_device,
	&serial_hspd_platform_device,
#if defined (CONFIG_RTC_DRV_LPC32XX)
	&rtc_device,
#endif
#if defined (CONFIG_MACH_LPC32XX_I2C0_ENABLE)
	&i2c0_device,
#endif
#if defined (CONFIG_MACH_LPC32XX_I2C1_ENABLE)
	&i2c1_device,
#endif
#if defined (CONFIG_MACH_LPC32XX_USBOTG_I2C_ENABLE)
	&i2c2_device,
#endif
#if defined(CONFIG_USB_OHCI_HCD)
	&ohci_device,
#endif
#if defined(CONFIG_LPC32XX_WATCHDOG)
	&watchdog_device,
#endif
};

void __init lpc32xx_init (void)
{
	u32 tmp = 0;

	/* Setup clocks before anything else! */
	clk_init();

	/* Pre-initialize serial ports */
	serial_init();

	platform_add_devices (lpc32xx_devs, ARRAY_SIZE (lpc32xx_devs));

	tmp = __raw_readl(UARTCTL_CTRL(io_p2v(UART_CTRL_BASE)));
#if defined(MACH_LPC32XX_UART6_IRDAMODE)
	tmp &= ~UART_UART6_IRDAMOD_BYPASS;
#else
	tmp |= UART_UART6_IRDAMOD_BYPASS;
#endif
	__raw_writel(tmp, UARTCTL_CTRL(io_p2v(UART_CTRL_BASE)));
}

static struct map_desc lpc32xx_io_desc[] __initdata = {
	{
		.virtual	= io_p2v(AHB0_START),
		.pfn		= __phys_to_pfn(AHB0_START),
		.length		= AHB0_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(AHB1_START),
		.pfn		= __phys_to_pfn(AHB1_START),
		.length		= AHB1_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(FABAPB_START),
		.pfn		= __phys_to_pfn(FABAPB_START),
		.length		= FABAPB_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(IRAM_BASE),
		.pfn		= __phys_to_pfn(IRAM_BASE),
		.length		= (SZ_64K * 4),
		.type		= MT_DEVICE
	},
	//Add For CPLD connect on EMC_CS2
	{
		.virtual	= io_p2v(EMC_CS2_BASE),
		.pfn		= __phys_to_pfn(EMC_CS2_BASE),
		.length		= EMC_PER_CS_SIZE,
		.type		= MT_DEVICE
	},
	//Add For FPGA connect on EMC_CS3
	{
		.virtual	= io_p2v(EMC_CS3_BASE),
		.pfn		= __phys_to_pfn(EMC_CS3_BASE),
		.length		= EMC_PER_CS_SIZE,
		.type		= MT_DEVICE
	},
};

void __init lpc32xx_map_io(void)
{
	iotable_init (lpc32xx_io_desc, ARRAY_SIZE (lpc32xx_io_desc));
}

/*
 * System reset via the watchdog timer
 */
void lpc32xx_watchdog_reset(void)
{
	/* Make sure WDT clocks are enabled */
	__raw_writel(CLKPWR_PWMCLK_WDOG_EN,
			CLKPWR_TIMER_CLK_CTRL(CLKPWR_IOBASE));

	/* Instant assert of RESETOUT_N with pulse length 1mS */
	__raw_writel(13000, io_p2v(WDTIM_BASE + 0x18));
	__raw_writel(0x70, io_p2v(WDTIM_BASE + 0xC));
}
