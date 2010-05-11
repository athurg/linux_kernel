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

#define BOARDDEBUG

#if defined (CONFIG_MACH_LPC32XX_IRAM_SIZE_256)
#define LPC32XX_IRAM_SIZE (256 * 1024)
#else
#define LPC32XX_IRAM_SIZE (128 * 1024)
#endif

/*
 * Structure used to define the hardware for the Phytec board. This
 * is obtained from index (PHY3250_SEEPROM_CFGOFS) in the AT25 serial
 * EEPROM. The .fieldval value must be checked for the correct value
 * (PHY_HW_VER_VAL) or the sturcture is invalid.
 */
typedef struct
{
  u32 dramcfg;    /* DRAM config word */
  u32 syscfg;     /* Configuration word */
  /* MAC address, use lower 6 bytes only, index 0 is first byte */
  u8  mac[8];     /* Only the first 6 are used */
  u32 rsvd [5];   /* Reserved, must be 0 */
  u32 fieldvval;  /* Must be PHY_HW_VER_VAL */
} PHY_HW_T;
static PHY_HW_T phyhwdata;

/*
 * Serial EEPROM support
 */
#define PHY_HW_VER_VAL 0x000A3250
#define SEEPROM_READ          0x03
#define PHY3250_SEEPROM_SIZE  0x8000
#define PHY3250_SEEPROM_CFGOFS (PHY3250_SEEPROM_SIZE - 0x100)
#define AT25256_PAGE_SIZE 64

#if defined(CONFIG_KEYBOARD_LPC32XX)
/*
 * Board specific key scanner driver data
 */
#define KMATRIX_SIZE 1
static int lpc32xx_keymaps[] =
{
	KEY_1,	/* 1, 1 */
};
struct lpc32XX_kscan_cfg lpc32xx_kscancfg = {
	.matrix_sz	= KMATRIX_SIZE,
	.keymap		= lpc32xx_keymaps,
	/* About a 30Hz scan rate based on a 32KHz clock */
	.deb_clks	= 3,
	.scan_delay	= 34,
};

static struct resource kscan_resources[] = {
	[0] = {
		.start	= KSCAN_BASE,
		.end	= KSCAN_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_KEY,
		.end	= IRQ_KEY,
		.flags	= IORESOURCE_IRQ,
	},

};
static struct platform_device kscan_device = {
	.name		= "lpc32xx_keys",
	.id		= 0,
	.dev		= {
		.platform_data	= &lpc32xx_kscancfg,
	},
	.num_resources	= ARRAY_SIZE(kscan_resources),
	.resource	= kscan_resources,
};
#endif

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
		.name = "at25256a",
		.byte_len = 0x8000,
		.page_size = AT25256_PAGE_SIZE,
		.flags = EE_ADDR2,
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

#if defined (CONFIG_MMC_ARMMMCI)
/*
 * Returns 0 when card is removed, !0 when installed
 */
unsigned int mmc_status(struct device *dev)
{
	u32 tmp, inserted = 0;

	tmp = __raw_readl(GPIO_P3_INP_STATE(GPIO_IOBASE)) &
		INP_STATE_GPIO_01;
	if (tmp == 0)
	{
		inserted = 1;
	}

	return inserted;
}

/*
 * Enable or disable SD slot power
 */
void mmc_power_enable(int enable)
{
	if (enable != 0)
	{
		__raw_writel(OUTP_STATE_GPO(5), GPIO_P3_OUTP_SET(GPIO_IOBASE));
	}
	else
	{
		__raw_writel(OUTP_STATE_GPO(5), GPIO_P3_OUTP_CLR(GPIO_IOBASE));
	}
}

/*
 * Board specific MMC driver data
 */
struct mmc_platform_data lpc32xx_plat_data = {
	.ocr_mask	= MMC_VDD_30_31|MMC_VDD_31_32|MMC_VDD_32_33|MMC_VDD_33_34,
	.status		= mmc_status,
};

/*
 * SD card controller resources
 */
struct amba_device mmc_device = {
	.dev				= {
		.coherent_dma_mask	= ~0,
		.bus_id			= "dev:31",
		.platform_data		= &lpc32xx_plat_data,
	},
	.res				= {
		.start			= SD_BASE,
		.end			= (SD_BASE + SZ_4K - 1),
		.flags			= IORESOURCE_MEM,
	},
	.dma_mask			= ~0,
	.irq				= {IRQ_SD0, IRQ_SD1},
};
#endif

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
		.name	= "phy3250-boot",
		.offset	= 0,
		.size	= (BLK_SIZE * 90)
	},
	{
		.name	= "phy3250-ubt-prms",
		.offset	= (BLK_SIZE * 90),
		.size	= (BLK_SIZE * 10)
	},
	{
		.name	= "phy3250-kernel",
		.offset	= (BLK_SIZE * 100),
		.size	= (BLK_SIZE * 256)
	},
	{
		.name	= "phy3250-rootfs",
		.offset	= (BLK_SIZE * 356),
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

#if defined (CONFIG_FB_ARMCLCD)
/*
 * Board specific LCD setup and functions
 */
#if defined (CONFIG_PHY3250_LCD_PANEL)
#if defined (CONFIG_PHY3250_QVGA_PANEL_1307_0) || defined (CONFIG_PHY3250_QVGA_PANEL_1307_1)
/*
 * Support for QVGA portrait panel from Phytec
 */
static struct clcd_panel conn_lcd_panel = {
	.mode		= {
		.name		= "QVGA portrait",
		.refresh	= 60,
		.xres		= 240,
		.yres		= 320,
		.pixclock	= 191828,
		.left_margin	= 22,
		.right_margin	= 11,
		.upper_margin	= 2,
		.lower_margin	= 1,
		.hsync_len	= 5,
		.vsync_len	= 2,
		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.width		= -1,
	.height		= -1,
	.tim2		= (CLCDC_LCDTIMING2_IVS | CLCDC_LCDTIMING2_IHS),
	.cntl		= (CNTL_BGR | CLCDC_LCDCTRL_TFT | CNTL_LCDVCOMP(1) |
				CLCDC_LCDCTRL_BPP16_565),
	.bpp		= 16,
};
#define PANEL_SIZE (3 * SZ_64K)
#endif
static int lpc32xx_clcd_setup(struct clcd_fb *fb)
{
	dma_addr_t dma;

	fb->fb.screen_base = (void *) NULL;
#if defined(CONFIG_MACH_LPC32XX_IRAM_FOR_CLCD)
	if (PANEL_SIZE <= LPC32XX_IRAM_SIZE) {
		fb->fb.screen_base = (void *) io_p2v(IRAM_BASE);
		fb->fb.fix.smem_start = (dma_addr_t) IRAM_BASE;
	}
#endif

	if (fb->fb.screen_base == NULL) {
		fb->fb.screen_base = dma_alloc_writecombine(&fb->dev->dev,
			PANEL_SIZE, &dma, GFP_KERNEL);
		fb->fb.fix.smem_start = dma;
	}

	if (!fb->fb.screen_base) {
		printk(KERN_ERR "CLCD: unable to map framebuffer\n");
		return -ENOMEM;
	}

	fb->fb.fix.smem_len = PANEL_SIZE;
	fb->panel = &conn_lcd_panel;

	return 0;
}
static int lpc32xx_clcd_mmap(struct clcd_fb *fb, struct vm_area_struct *vma)
{
#if defined(CONFIG_MACH_LPC32XX_IRAM_FOR_CLCD)
	if (PANEL_SIZE <= LPC32XX_IRAM_SIZE) {
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			(vma->vm_end - vma->vm_start), vma->vm_page_prot)) {
			return -EAGAIN;
		}

		return 0;
	}
	else
#endif

	return dma_mmap_writecombine(&fb->dev->dev, vma,
				     fb->fb.screen_base,
				     fb->fb.fix.smem_start,
				     fb->fb.fix.smem_len);
}
static void lpc32xx_clcd_remove(struct clcd_fb *fb)
{
#if defined(CONFIG_MACH_LPC32XX_IRAM_FOR_CLCD)
	if (PANEL_SIZE > LPC32XX_IRAM_SIZE)
#endif

	dma_free_writecombine(&fb->dev->dev, fb->fb.fix.smem_len,
			      fb->fb.screen_base, fb->fb.fix.smem_start);
}
void clcd_disable(struct clcd_fb *fb)
{
	/* Disable the backlight */
#if defined (CONFIG_PHY3250_QVGA_PANEL_1307_0)
	__raw_writel(OUTP_STATE_GPO(4), GPIO_P3_OUTP_SET(GPIO_IOBASE));
#elif defined (CONFIG_PHY3250_QVGA_PANEL_1307_1)
	__raw_writel(OUTP_STATE_GPO(4), GPIO_P3_OUTP_CLR(GPIO_IOBASE));
#endif

	/* Disable the LCD power */
	__raw_writel(OUTP_STATE_GPO(0), GPIO_P3_OUTP_CLR(GPIO_IOBASE));
}
void clcd_enable(struct clcd_fb *fb)
{
	/* Enable the backlight */
#if defined (CONFIG_PHY3250_QVGA_PANEL_1307_0)
	__raw_writel(OUTP_STATE_GPO(4), GPIO_P3_OUTP_CLR(GPIO_IOBASE));
#elif defined (CONFIG_PHY3250_QVGA_PANEL_1307_1)
	__raw_writel(OUTP_STATE_GPO(4), GPIO_P3_OUTP_SET(GPIO_IOBASE));
#endif

	/* Enable the LCD power */
	__raw_writel(OUTP_STATE_GPO(0), GPIO_P3_OUTP_SET(GPIO_IOBASE));
}
struct clcd_board lpc32xx_clcd_data = {
	.name		= "Phytec LCD",
	.check		= clcdfb_check,
	.decode		= clcdfb_decode,
	.disable	= clcd_disable,
	.enable		= clcd_enable,
	.setup		= lpc32xx_clcd_setup,
	.mmap		= lpc32xx_clcd_mmap,
	.remove		= lpc32xx_clcd_remove,
};
#endif

struct amba_device clcd_device = {
	.dev				= {
		.coherent_dma_mask	= ~0,
		.bus_id			= "dev:20",
		.platform_data		= &lpc32xx_clcd_data,
	},
	.res				= {
		.start			= LCD_BASE,
		.end			= (LCD_BASE + SZ_4K - 1),
		.flags			= IORESOURCE_MEM,
	},
	.dma_mask			= ~0,
	.irq				= {IRQ_LCD, NO_IRQ},
};
#endif

/*
 * Network configuration structure
 */
#if defined (CONFIG_LPC32XX_MII)
static int return_mac_address(u8 *mac)
{
	int ret = 0;
	int i;

	if (phyhwdata.fieldvval != PHY_HW_VER_VAL)
	{
		/* Field has garbage in it */
		printk(KERN_ERR "Invalid ethernet MAC address\n");
		ret = -ENODEV;
	}

	/* Use MAC address from hardware descriptor */
	for (i = 0; i < 6; i++)
	{
		mac [i] = phyhwdata.mac [i];
	}

	return ret;
}

struct lpc32xx_net_cfg lpc32xx_netdata =
{
	.get_mac_addr	= &return_mac_address,
	.phy_irq	= -1,
	.phy_mask	= 0xFFFFFFF0,

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
	.id		= 0,
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

/*
 * Load the board hardware descriptor from the serial EEPROM
 */
static void __init phy3250_load_hw_desc(void)
{
	u32 svclk, tmp, addr;
	int i, len;
	u8 *p8, cmd [4], in [4];
	u32 sspiobase = io_p2v(SSP0_BASE);

	/* Enable SSP0 clock */
	svclk = __raw_readl(CLKPWR_SSP_CLK_CTRL(CLKPWR_IOBASE));
	__raw_writel((svclk | CLKPWR_SSPCTRL_SSPCLK0_EN),
		CLKPWR_SSP_CLK_CTRL(CLKPWR_IOBASE));

	/* Setup SSP0 chip select mux */
	phy3250_spi_cs_setup(0);
	phy3250_spi_cs_set(0, 1);

	/* Initial setup of SSP0 for transfer with AT25 serial EEPROM */
	__raw_writel(0, SSP_ICR(sspiobase));
	__raw_writel((SSP_ICR_RORIC | SSP_ICR_RTIC), SSP_IMSC(sspiobase));

	/* Setup default SPI mode at about 5MHz */
	__raw_writel((SSP_CR0_DSS(8) | SSP_CR0_FRF_SPI | SSP_CR0_CPOL(0) |
		SSP_CR0_CPHA(0) | SSP_CR0_SCR(40)), SSP_CR0(sspiobase));
	__raw_writel(SSP_CR1_SSP_ENABLE, SSP_CR1(sspiobase));
	__raw_writel(SSP_CPSR_CPDVSR(40), SSP_CPSR(sspiobase));

	/* Flush RX FIFO */
	while (__raw_readl(SSP_SR(sspiobase)) & SSP_SR_RNE)
	{
		tmp = __raw_readl(SSP_DATA(sspiobase));
	}

	len = sizeof (phyhwdata);
	p8 = (u8 *) &phyhwdata;
	addr = PHY3250_SEEPROM_CFGOFS;
	while (len > 0)
	{
		/* Toggle chip select low */
		phy3250_spi_cs_set(0, 0);

		/* Issue read command and address for 1 byte */
		cmd[0] = SEEPROM_READ;
		cmd[1] = ((addr >> 8) & 0xFF);
		cmd[2] = ((addr >> 0) & 0xFF);
		cmd[3] = 0xFF;
		for (i = 0; i < 4; i++)
		{
			__raw_writel((u32) cmd [i], SSP_DATA(sspiobase));
		}

		/* Save 4 bytes */
		i = 0;
		while (i < 4)
		{
			if (__raw_readl(SSP_SR(sspiobase)) & SSP_SR_RNE)
			{
				in [i] = (u8) __raw_readl(SSP_DATA(sspiobase));
				i++;
			}
		}

		/* Save next byte */
		*p8 = in [3];
		p8++;
		len--;
		addr++;

		/* Toggle chip select high */
		phy3250_spi_cs_set(0, 1);
	}

	/* Stop SSP0 */
	__raw_writel(0, SSP_CR0(sspiobase));

	/* Restore clock */
	__raw_writel(svclk, CLKPWR_SSP_CLK_CTRL(CLKPWR_IOBASE));

	if (phyhwdata.fieldvval != PHY_HW_VER_VAL)
	{
		printk(KERN_ERR "Invalid board descriptor!\n");
	}
#if defined (BOARDDEBUG)
	else
	{
		printk(KERN_INFO "Hardware descriptor info:\n");
		printk(KERN_INFO " DRAM config word: 0x%08x\n", phyhwdata.dramcfg);
		printk(KERN_INFO " syscfg word:      0x%08x\n", phyhwdata.syscfg);
		printk(KERN_INFO " fieldval word:    0x%08x\n", phyhwdata.fieldvval);
		printk(KERN_INFO " MAC address:      ");
		printk(KERN_INFO "%02x:%02x:%02x:%02x:%02x:%02x\n",
			phyhwdata.mac [0], phyhwdata.mac [1], phyhwdata.mac [2],
			phyhwdata.mac [3], phyhwdata.mac [4], phyhwdata.mac [5]);
	}
#endif
}

static struct platform_device* phy3250_devs[] __initdata = {
#if defined(CONFIG_SPI_LPC32XX)
	&ssp0_device,
#endif
#if defined(CONFIG_KEYBOARD_LPC32XX)
	&kscan_device,
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

#if defined (CONFIG_MMC_ARMMMCI)
	/* Enable SD slot power */
	mmc_power_enable(1);
#endif

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

#if defined (CONFIG_PHY3250_LCD_PANEL)
	/* Setup LCD muxing to RGB565 */
	tmp = __raw_readl(CLKPWR_LCDCLK_CTRL(CLKPWR_IOBASE)) &
		~(CLKPWR_LCDCTRL_LCDTYPE_MSK | CLKPWR_LCDCTRL_PSCALE_MSK);
	tmp |= CLKPWR_LCDCTRL_LCDTYPE_TFT16;
	__raw_writel(tmp, CLKPWR_LCDCLK_CTRL(CLKPWR_IOBASE));
#endif

	/* Set up I2C levels */
	tmp = __raw_readl(CLKPWR_I2C_CLK_CTRL(CLKPWR_IOBASE));
	tmp |= CLKPWR_I2CCLK_USBI2CHI_DRIVE | CLKPWR_I2CCLK_I2C2HI_DRIVE;
	__raw_writel(tmp, CLKPWR_I2C_CLK_CTRL(CLKPWR_IOBASE));

	/* Enable DMA for I2S1 channel */
	tmp = __raw_readl(CLKPWR_I2S_CLK_CTRL(CLKPWR_IOBASE));
	tmp = CLKPWR_I2SCTRL_I2S1_USE_DMA;
	__raw_writel(tmp, CLKPWR_I2S_CLK_CTRL(CLKPWR_IOBASE));

	/* Load the board hardware descriptor, as some other board functions
	   require it's data */
	phy3250_load_hw_desc();

	/* Call chip specific init */
	lpc32xx_init();

	/* Add board platform devices */
	platform_add_devices (phy3250_devs, ARRAY_SIZE (phy3250_devs));


#if defined(CONFIG_MMC_ARMMMCI)
	/* Enable SD card clock so AMBA driver will work correctly. The
	   AMBA driver needs the clock before the SD card controller
	   driver initializes it. The clock will turn off once the driver
	   has been initialized. */
	tmp = __raw_readl(CLKPWR_MS_CTRL(CLKPWR_IOBASE));
	tmp |= CLKPWR_MSCARD_SDCARD_EN | CLKPWR_MSCARD_MSDIO_PU_EN;
	__raw_writel(tmp, CLKPWR_MS_CTRL(CLKPWR_IOBASE));

	amba_device_register(&mmc_device, &iomem_resource);
#endif

#if defined(CONFIG_FB_ARMCLCD)
	/* Enable LCD clock so AMBA driver will work correctly. The
	   AMBA driver needs the clock before the LCD driver initializes it.
	   The clock will turn off once the driver has been initialized. */
	tmp = __raw_readl(CLKPWR_LCDCLK_CTRL(CLKPWR_IOBASE));
	tmp |= CLKPWR_LCDCTRL_CLK_EN;
	__raw_writel(tmp, CLKPWR_LCDCLK_CTRL(CLKPWR_IOBASE));

	amba_device_register(&clcd_device, &iomem_resource);
#endif

	/* Disable UART5->USB transparent mode or USB won't work */
	tmp = __raw_readl(UARTCTL_CTRL(io_p2v(UART_CTRL_BASE)));
	tmp &= ~UART_U5_ROUTE_TO_USB;
	__raw_writel(tmp, UARTCTL_CTRL(io_p2v(UART_CTRL_BASE)));

#if defined (CONFIG_SND_LPC3XXX_SOC)
	/* Test clock needed for UDA1380 */
	__raw_writel((CLKPWR_TESTCLK2_SEL_MOSC | CLKPWR_TESTCLK_TESTCLK2_EN),
		CLKPWR_TEST_CLK_SEL(CLKPWR_IOBASE));
#endif

#if defined(CONFIG_RTC_DRV_PCF8563)
	/* I2C based RTC device on I2C1 */
	i2c_register_board_info(0, phy3250_i2c_board_info,
				ARRAY_SIZE(phy3250_i2c_board_info));
#endif
}

MACHINE_START (LPC3XXX, "Phytec 3250 board with the LPC3250 Microcontroller")
	/* Maintainer: Kevin Wells, NXP Semiconductors */
	.phys_io	= UART5_BASE,
	.io_pg_offst	= ((io_p2v (UART5_BASE))>>18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= lpc32xx_map_io,
	.init_irq	= lpc32xx_init_irq,
	.timer		= &lpc32xx_timer,
	.init_machine	= phy3250_board_init,
MACHINE_END

