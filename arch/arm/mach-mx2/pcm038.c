/*
 * Copyright 2007 Robert Schwebel <r.schwebel@pengutronix.de>, Pengutronix
 * Copyright (C) 2008 Juergen Beisert (kernel@pengutronix.de)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#include <linux/platform_device.h>
#include <linux/mtd/physmap.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/iomux-mx1-mx2.h>
#include <asm/mach/time.h>
#include <mach/imx-uart.h>
#include <mach/board-pcm038.h>

/*
 * Phytec's phyCORE-i.MX27 comes with 32MiB flash,
 * 16 bit width
 */
static struct physmap_flash_data pcm038_flash_data = {
	.width = 2,
};

static struct resource pcm038_flash_resource = {
	.start = 0xc0000000,
	.end   = 0xc1ffffff,
	.flags = IORESOURCE_MEM,
};

static struct platform_device pcm038_nor_mtd_device = {
	.name = "physmap-flash",
	.id = 0,
	.dev = {
		.platform_data = &pcm038_flash_data,
	},
	.num_resources = 1,
	.resource = &pcm038_flash_resource,
};

static int mxc_uart0_pins[] = {
	PE12_PF_UART1_TXD,
	PE13_PF_UART1_RXD,
	PE14_PF_UART1_CTS,
	PE15_PF_UART1_RTS
};

static int uart_mxc_port0_init(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_uart0_pins,
			ARRAY_SIZE(mxc_uart0_pins),
			MXC_GPIO_ALLOC_MODE_NORMAL, "UART0");
}

static int uart_mxc_port0_exit(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_uart0_pins,
			ARRAY_SIZE(mxc_uart0_pins),
			MXC_GPIO_ALLOC_MODE_RELEASE, "UART0");
}

static int mxc_uart1_pins[] = {
	PE3_PF_UART2_CTS,
	PE4_PF_UART2_RTS,
	PE6_PF_UART2_TXD,
	PE7_PF_UART2_RXD
};

static int uart_mxc_port1_init(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_uart1_pins,
			ARRAY_SIZE(mxc_uart1_pins),
			MXC_GPIO_ALLOC_MODE_NORMAL, "UART1");
}

static int uart_mxc_port1_exit(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_uart1_pins,
			ARRAY_SIZE(mxc_uart1_pins),
			MXC_GPIO_ALLOC_MODE_RELEASE, "UART1");
}

static int mxc_uart2_pins[] = { PE10_PF_UART3_CTS,
				PE9_PF_UART3_RXD,
				PE10_PF_UART3_CTS,
				PE9_PF_UART3_RXD };

static int uart_mxc_port2_init(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_uart2_pins,
			ARRAY_SIZE(mxc_uart2_pins),
			MXC_GPIO_ALLOC_MODE_NORMAL, "UART2");
}

static int uart_mxc_port2_exit(struct platform_device *pdev)
{
	return mxc_gpio_setup_multiple_pins(mxc_uart2_pins,
			ARRAY_SIZE(mxc_uart2_pins),
			MXC_GPIO_ALLOC_MODE_RELEASE, "UART2");
}

static struct imxuart_platform_data uart_pdata[] = {
	{
		.init = uart_mxc_port0_init,
		.exit = uart_mxc_port0_exit,
		.flags = IMXUART_HAVE_RTSCTS,
	}, {
		.init = uart_mxc_port1_init,
		.exit = uart_mxc_port1_exit,
		.flags = IMXUART_HAVE_RTSCTS,
	}, {
		.init = uart_mxc_port2_init,
		.exit = uart_mxc_port2_exit,
		.flags = IMXUART_HAVE_RTSCTS,
	},
};

static int mxc_fec_pins[] = {
	PD0_AIN_FEC_TXD0,
	PD1_AIN_FEC_TXD1,
	PD2_AIN_FEC_TXD2,
	PD3_AIN_FEC_TXD3,
	PD4_AOUT_FEC_RX_ER,
	PD5_AOUT_FEC_RXD1,
	PD6_AOUT_FEC_RXD2,
	PD7_AOUT_FEC_RXD3,
	PD8_AF_FEC_MDIO,
	PD9_AIN_FEC_MDC,
	PD10_AOUT_FEC_CRS,
	PD11_AOUT_FEC_TX_CLK,
	PD12_AOUT_FEC_RXD0,
	PD13_AOUT_FEC_RX_DV,
	PD14_AOUT_FEC_CLR,
	PD15_AOUT_FEC_COL,
	PD16_AIN_FEC_TX_ER,
	PF23_AIN_FEC_TX_EN
};

static void gpio_fec_active(void)
{
	mxc_gpio_setup_multiple_pins(mxc_fec_pins,
			ARRAY_SIZE(mxc_fec_pins),
			MXC_GPIO_ALLOC_MODE_NORMAL, "FEC");
}

static void gpio_fec_inactive(void)
{
	mxc_gpio_setup_multiple_pins(mxc_fec_pins,
			ARRAY_SIZE(mxc_fec_pins),
			MXC_GPIO_ALLOC_MODE_RELEASE, "FEC");
}

static struct platform_device *platform_devices[] __initdata = {
	&pcm038_nor_mtd_device,
};

static void __init pcm038_init(void)
{
	int i;
	gpio_fec_active();

	for (i = 0; i < 3; i++)
		imx_init_uart(i, &uart_pdata[i]);

	platform_add_devices(platform_devices, ARRAY_SIZE(platform_devices));

#ifdef CONFIG_MACH_PCM970_BASEBOARD
	pcm970_baseboard_init();
#endif
}

static void __init pcm038_timer_init(void)
{
	mxc_clocks_init(26000000);
	mxc_timer_init("gpt_clk.0");
}

struct sys_timer pcm038_timer = {
	.init = pcm038_timer_init,
};

MACHINE_START(PCM038, "phyCORE-i.MX27")
	.phys_io        = AIPI_BASE_ADDR,
	.io_pg_offst    = ((AIPI_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params    = PHYS_OFFSET + 0x100,
	.map_io         = mxc_map_io,
	.init_irq       = mxc_init_irq,
	.init_machine   = pcm038_init,
	.timer          = &pcm038_timer,
MACHINE_END
