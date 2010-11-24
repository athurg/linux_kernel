/*
 *  linux/arch/arm/mach-lpc32xx/board-phy3250.c
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
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/dma-mapping.h>
#include <linux/sysdev.h>
#include <linux/amba/bus.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>
#include <asm/mach/mmc.h>

#include <mach/platform.h>
#include <mach/lpc32xx_clkpwr.h>
#include <mach/lpc32xx_gpio.h>
#include <mach/lpc32xx_clcdc.h>
#include <mach/lpc32xx_ssp.h>
#include <mach/lpc32xx_uart.h>
#include <mach/clock.h>
#include "sys-lpc32xx.h"

#if defined (CONFIG_MACH_LPC32XX_IRAM_SIZE_256)
#define LPC32XX_IRAM_SIZE (256 * 1024)
#else
#define LPC32XX_IRAM_SIZE (128 * 1024)
#endif
/*
 * Serial EEPROM support
 */
#define AT25256_PAGE_SIZE 0x100


static void phy3250_spi_cs_setup(int cs)
{
	/* Setup SPI CS0 as an output on GPIO5 */
	__raw_writel((1 << 5), GPIO_P2_MUX_CLR(GPIO_IOBASE));

	/* Set chip select high */
	__raw_writel(OUTP_STATE_GPIO(5),
		GPIO_P3_OUTP_SET(GPIO_IOBASE));
}
static int phy3250_spi_cs_set(int cs, int state)
{
	if (cs == 0)
	{
		if (state != 0)
		{
			/* Set chip select high */
			__raw_writel(OUTP_STATE_GPIO(5),
				GPIO_P3_OUTP_SET(GPIO_IOBASE));
		}
		else
		{
			/* Set chip select low */
			__raw_writel(OUTP_STATE_GPIO(5),
				GPIO_P3_OUTP_CLR(GPIO_IOBASE));
		}
	}
	else
	{
		/* Invalid chip select */
		return -ENODEV;
	}

	return 0;
}

#if defined(CONFIG_SPI_LPC32XX)
struct lpc32xx_spi_cfg lpc32xx_spi0data =
{
	.num_cs		= 1, /* Only 1 chip select */
	.spi_cs_setup	= &phy3250_spi_cs_setup,
	.spi_cs_set	= &phy3250_spi_cs_set,
};

/* AT25 driver registration */
static int __init phy3250_spi_board_register(void)
{
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
	struct spi_board_info info =
	{
		.modalias = "spidev",
		.max_speed_hz = 5000000,
		.bus_num = 0,
		.chip_select = 0,
	};

#else
	static struct spi_eeprom eeprom =
	{
		.name = "w25x80a",
		.byte_len = 0x8000,
		.page_size = AT25256_PAGE_SIZE,
		.flags = EE_ADDR3,
	};
	struct spi_board_info info =
	{
		.modalias = "at25",
		.max_speed_hz = 5000000,
		.bus_num = 0,
		.chip_select = 0,
		.platform_data = &eeprom,
	};
#endif
	return spi_register_board_info(&info, 1);
}
arch_initcall(phy3250_spi_board_register);

static struct resource ssp0_resources[] = {
	[0] = {
		.start	= SSP0_BASE,
		.end	= SSP0_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_SSP0,
		.end	= IRQ_SSP0,
		.flags	= IORESOURCE_IRQ,
	},

};
static struct platform_device ssp0_device = {
	.name		= "spi_lpc32xx",
	.id		= 0,
	.dev		= {
		.platform_data	= &lpc32xx_spi0data,
	},
	.num_resources	= ARRAY_SIZE(ssp0_resources),
	.resource	= ssp0_resources,
};
#endif


//default MAC address
static u8 default_mac[]={0x00,0x01,0x90,0x00,0xC0,0x81};


#if defined(CONFIG_MTD_NAND_SLC_LPC32XX)
/*
 * Board specific NAND setup data
 */
static int nandwp_enable(int enable)
{
	if (enable != 0)
	{
		__raw_writel(OUTP_STATE_GPO(19), GPIO_P3_OUTP_CLR(GPIO_IOBASE));
	}
	else
	{
		__raw_writel(OUTP_STATE_GPO(19), GPIO_P3_OUTP_SET(GPIO_IOBASE));
	}

	return 1;
}
#define BLK_SIZE (512 * 32)
static struct mtd_partition __initdata phy3250_nand_partition[] = {
	{
		.name	= "nts3250-nand",
		.offset	= 0,
		.size	= MTDPART_SIZ_FULL
	},
};
static struct mtd_partition * __init phy3250_nand_partitions(int size, int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(phy3250_nand_partition);
	return phy3250_nand_partition;
}
struct lpc32XX_nand_cfg lpc32xx_nandcfg =
{
	.wdr_clks		= 3,
	.wwidth			= 28571428,
	.whold			= 100000000,
	.wsetup			= 66666666,
	.rdr_clks		= 3,
	.rwidth			= 28571428,
	.rhold			= 100000000,
	.rsetup			= 66666666,
	.use16bus		= 0,
	.enable_write_prot	= nandwp_enable,
	.partition_info		= phy3250_nand_partitions,
};

/*
 * SLC NAND resources
 */
static struct resource slc_nand_resources[] = {
	[0] = {
		.start	= SLC_BASE,
		.end	= SLC_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},

	[1] = {
		.start	= IRQ_FLASH,
		.end	= IRQ_FLASH,
		.flags	= IORESOURCE_IRQ,
	},

};
static struct platform_device slc_nand_device = {
	.name		= "lpc32xx-nand",
	.id		= 0,
	.dev		= {
				.platform_data	= &lpc32xx_nandcfg,
	},
	.num_resources	= ARRAY_SIZE(slc_nand_resources),
	.resource	= slc_nand_resources,
};
#endif


/*
 * Network configuration structure
 */
#if defined (CONFIG_LPC32XX_MII)
static int return_mac_address(u8 *mac)
{
	int i;

	for (i = 0; i < 6; i++)		mac [i] = default_mac [i];

	return 0;
}

struct lpc32xx_net_cfg lpc32xx_netdata =
{
	.get_mac_addr	= &return_mac_address,
	.phy_irq	= -1,
	.phy_mask	= (~(1<<4)),

};

static struct resource net_resources[] = {
	[0] = {
		.start	= ETHERNET_BASE,
		.end	= ETHERNET_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},

	[1] = {
		.start	= IRQ_ETHERNET,
		.end	= IRQ_ETHERNET,
		.flags	= IORESOURCE_IRQ,
	},

};

static u64 lpc32xx_mac_dma_mask = 0xffffffffUL;
static struct platform_device net_device = {
	.name		= "lpc32xx-net",
	.id		= 4,
	.dev		= {
		.dma_mask = &lpc32xx_mac_dma_mask,
		.coherent_dma_mask = 0xffffffffUL,
		.platform_data	= &lpc32xx_netdata,
	},
	.num_resources	= ARRAY_SIZE(net_resources),
	.resource	= net_resources,
};
#endif

#if defined(CONFIG_USB_GADGET_LPC32XX)
static void phy3250_usbd_conn_chg(int conn) {
    /* Do nothing, it might be nice to enable an LED
       based on conn state being !0 */
}

static void phy3250_usbd_susp_chg(int susp) {
    /* Device suspend if susp != 0 */
}

static void phy3250_rmwkup_chg(int remote_wakup_enable) {
    /* Enable or disable USB remote wakeup */
}

struct lpc32xx_usbd_cfg lpc32xx_usbddata = {
    .vbus_drv_pol = 1,
    .conn_chgb = &phy3250_usbd_conn_chg,
    .susp_chgb = &phy3250_usbd_susp_chg,
    .rmwk_chgb = &phy3250_rmwkup_chg,
};

/* The dmamask must be set for OHCI to work, align to 128 bytes */
static u64 usbd_dmamask = ~(u32) 0x7F;
static struct resource usbd_resources[] = {
	{
		.start = USB_BASE,
		.end = USB_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}, {
		.start = IRQ_USB_DEV_LP,
		.flags = IORESOURCE_IRQ,
	}, {
		.start = IRQ_USB_DEV_HP,
		.flags = IORESOURCE_IRQ,
	}, {
		.start = IRQ_USB_DEV_DMA,
		.flags = IORESOURCE_IRQ,
	}, {
		.start = IRQ_USB_OTG_ATX,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device usbd_device = {
	.name = "lpc32xx_udc",
	.id = -1,
	.dev = {
		.dma_mask = &usbd_dmamask,
		.coherent_dma_mask = 0xFFFFFFFF,
		.platform_data	= &lpc32xx_usbddata,
		},
	.num_resources = ARRAY_SIZE(usbd_resources),
	.resource = usbd_resources,
};
#endif

#if defined(CONFIG_RTC_DRV_PCF8563)
static struct i2c_board_info __initdata phy3250_i2c_board_info [] = {
	{
		I2C_BOARD_INFO("rtc-pcf8563", 0x51),
	},
};
#endif


static struct platform_device* phy3250_devs[] __initdata = {
#if defined(CONFIG_SPI_LPC32XX)
	&ssp0_device,
#endif
#if defined (CONFIG_LPC32XX_MII)
	&net_device,
#endif
#if defined(CONFIG_MTD_NAND_SLC_LPC32XX)
	&slc_nand_device,
#endif
#if defined(CONFIG_USB_GADGET_LPC32XX)
	&usbd_device,
#endif
};

/*
 * Board specific functions
 */
void __init phy3250_board_init(void)
{
	u32 tmp;

#if defined (CONFIG_ENABLE_BOARD_LED_TICK)
	/* Set LED GPIO as an output */
	__raw_writel(OUTP_STATE_GPO(1), GPIO_P2_DIR_SET(GPIO_IOBASE));
#endif

#if defined (CONFIG_LPC32XX_MII)
	/* Setup network interface for RMII mode */
	tmp = __raw_readl(CLKPWR_MACCLK_CTRL(CLKPWR_IOBASE));
	tmp &= ~CLKPWR_MACCTRL_PINS_MSK;
#if defined (CONFIG_MAC_LPC32XX_MII_SUPPORT)
	tmp |= CLKPWR_MACCTRL_USE_MII_PINS;
#else
	tmp |= CLKPWR_MACCTRL_USE_RMII_PINS;
#endif
	__raw_writel(tmp, CLKPWR_MACCLK_CTRL(CLKPWR_IOBASE));
#endif

	/* Setup SLC NAND controller */
	__raw_writel(CLKPWR_NANDCLK_SEL_SLC, CLKPWR_NAND_CLK_CTRL(CLKPWR_IOBASE));

	/* Set up I2C levels */
	tmp = __raw_readl(CLKPWR_I2C_CLK_CTRL(CLKPWR_IOBASE));
	tmp |= CLKPWR_I2CCLK_USBI2CHI_DRIVE | CLKPWR_I2CCLK_I2C2HI_DRIVE;
	__raw_writel(tmp, CLKPWR_I2C_CLK_CTRL(CLKPWR_IOBASE));

	/* Enable DMA for I2S1 channel */
	tmp = __raw_readl(CLKPWR_I2S_CLK_CTRL(CLKPWR_IOBASE));
	tmp = CLKPWR_I2SCTRL_I2S1_USE_DMA;
	__raw_writel(tmp, CLKPWR_I2S_CLK_CTRL(CLKPWR_IOBASE));

	lpc32xx_init();

	/* Add board platform devices */
	platform_add_devices (phy3250_devs, ARRAY_SIZE (phy3250_devs));

	/* Disable UART5->USB transparent mode or USB won't work */
	tmp = __raw_readl(UARTCTL_CTRL(io_p2v(UART_CTRL_BASE)));
	tmp &= ~UART_U5_ROUTE_TO_USB;
	__raw_writel(tmp, UARTCTL_CTRL(io_p2v(UART_CTRL_BASE)));


#if defined(CONFIG_RTC_DRV_PCF8563)
	/* I2C based RTC device on I2C1 */
	i2c_register_board_info(0, phy3250_i2c_board_info,
				ARRAY_SIZE(phy3250_i2c_board_info));
#endif
}

MACHINE_START (LPC3XXX, "NTS3250 with the LPC3250 Microcontroller")
	.phys_io	= UART5_BASE,
	.io_pg_offst	= ((io_p2v (UART5_BASE))>>18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= lpc32xx_map_io,
	.init_irq	= lpc32xx_init_irq,
	.timer		= &lpc32xx_timer,
	.init_machine	= phy3250_board_init,
MACHINE_END

