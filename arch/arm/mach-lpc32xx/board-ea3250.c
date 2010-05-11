/*
 *  linux/arch/arm/mach-lpc32xx/board-ea3250.c
 *
 *  Copyright (C) 2009 Embedded Artists AB
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
#include <linux/kthread.h>

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

#include <linux/spi/ads7846.h>

#include "sys-lpc32xx.h"


#define I2C_PCA9532_ADDR 0x60
#define I2C_24LC256_ADDR 0x50


#if defined(CONFIG_SPI_LPC32XX)

/* chip select touch panel */
#define CS_TP 	0 

static void ea3250_spi_cs_setup(int cs)
{
}

static int ea3250_spi_cs_set(int cs, int state)
{
	if (cs == CS_TP)
	{
		if (state != 0)
		{
			/* Set chip select high */
			__raw_writel(OUTP_STATE_GPO(11),
				GPIO_P3_OUTP_SET(GPIO_IOBASE));
		}
		else
		{
			/* Set chip select low */
			__raw_writel(OUTP_STATE_GPO(11),
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


struct lpc32xx_spi_cfg lpc32xx_spi0data =
{
	.num_cs		= 1, /* Only 1 chip select, don't add LCD controller */
	.spi_cs_setup	= &ea3250_spi_cs_setup,
	.spi_cs_set	= &ea3250_spi_cs_set,
};


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

#if defined (CONFIG_SPI_LPC32XX) && defined(CONFIG_TOUCHSCREEN_ADS7846)

static int ea3250_ads7846_pendown_state(void)
{
	u32 tmp = __raw_readl(GPIO_P3_INP_STATE(GPIO_IOBASE)) & INP_STATE_GPIO_00;
	return (tmp == 0);
}

/* The TSC2046 touch controller is compliant with the ads7846 controller */
static struct ads7846_platform_data ea_ads7846_platform_data __initdata = {

	.debounce_max	= 10,
	.debounce_tol	= 3,
	.pressure_max	= 1024,

	.get_pendown_state = ea3250_ads7846_pendown_state,
};

/* ADS7846 (touch controller) driver registration */
static int __init ea3250_spi_ads7846_register(void)
{

	struct spi_board_info info =
	{
		.modalias      = "ads7846",
		.max_speed_hz  = 2500000,
		.chip_select   = CS_TP,
		.irq           = IRQ_GPIO_00,
		.platform_data = &ea_ads7846_platform_data,
	};

	/* GPIO_00 as input */
	__raw_writel(PIO_DIR_GPIO(0), GPIO_P2_DIR_CLR(GPIO_IOBASE));

	return spi_register_board_info(&info, 1);
}
arch_initcall(ea3250_spi_ads7846_register);

#endif

#if defined (CONFIG_MMC_ARMMMCI)

static int card_inserted = 1;

#if defined (CONFIG_SENSORS_PCA9532)
static int card_detect_thread(void __iomem* d)
{
	int err = 0;
	struct i2c_adapter *adap;
	struct i2c_client *client;
	u8 data = 0;

	while (!kthread_should_stop()) {
		adap = i2c_get_adapter(0);

		if (!adap) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(HZ);

			continue;
		}

		list_for_each_entry(client, &adap->clients, list) {
			if (client->addr == I2C_PCA9532_ADDR) {

				i2c_put_adapter(adap);

				/* select input0 register */
				data = 0;
				err = i2c_master_send(client, (char*)&data ,1);
	
				/* read value from register */
				err = i2c_master_recv(client, (char*)&data, 1);				

				/* LED4 input on PCA9532 is connected to card detect (active low) */
				card_inserted = ((data & 0x10) == 0);
				
				break;
			}
		}

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ);		
		
	}

	return 0;

}

static struct task_struct *cd_thread;
	
static void card_detect_start(void)
{
	cd_thread = kthread_run(card_detect_thread, NULL, "card-detect");
	if (IS_ERR(cd_thread)) {
		printk("\nFailed to start card detect thread\n");
	}
}

static void card_detect_stop(void)
{
	kthread_stop(cd_thread);
}
#else

#define card_detect_start()
#define card_detect_stop()

#endif

/*
 * Returns 0 when card is removed, !0 when installed
 */
unsigned int mmc_status(struct device *dev)
{
	return card_inserted;
}

/*
 * Enable or disable SD slot power
 */
void mmc_power_enable(int enable)
{
	if (enable != 0)
	{
		card_detect_start();

		/* active low */
		__raw_writel(OUTP_STATE_GPO(1), GPIO_P3_OUTP_CLR(GPIO_IOBASE));		
	}
	else
	{
		card_detect_stop();		

		__raw_writel(OUTP_STATE_GPO(1), GPIO_P3_OUTP_SET(GPIO_IOBASE));

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
#define BLK_SIZE (1024 * 128)
static struct mtd_partition __initdata ea3250_nand_partition[] = {
	{
		.name	= "ea3250-boot",
		.offset	= 0,
		.size	= (BLK_SIZE * 7)
	},
	{
		.name	= "ea3250-ubt-prms",
		.offset	= (BLK_SIZE * 7),
		.size	= (BLK_SIZE * 1)
	},
	{
		.name	= "ea3250-kernel",
		.offset	= (BLK_SIZE * 8),
		.size	= (BLK_SIZE * 32)
	},
	{
		.name	= "ea3250-rootfs",
		.offset	= (BLK_SIZE * 40),
		.size	= (BLK_SIZE * 40)
	},
	{
		.name	= "ea3250-jffs2",
		.offset	= (BLK_SIZE * 80),
		.size	= MTDPART_SIZ_FULL
	},
};
static struct mtd_partition * __init ea3250_nand_partitions(int size, int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(ea3250_nand_partition);
	return ea3250_nand_partition;
}
struct lpc32XX_nand_cfg lpc32xx_nandcfg =
{
	.wdr_clks		= 14,
	.wwidth			= 260000000,
	.whold			= 104000000,
	.wsetup			= 200000000,
	.rdr_clks		= 14,
	.rwidth			= 34666666,
	.rhold			= 104000000,
	.rsetup			= 200000000,
	.use16bus		= 0,
	.enable_write_prot	= nandwp_enable,
	.partition_info		= ea3250_nand_partitions,
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

#define ACTIVATE_CS   ( __raw_writel(OUTP_STATE_GPO(4), GPIO_P3_OUTP_CLR(GPIO_IOBASE)) )
#define DEACTIVATE_CS ( __raw_writel(OUTP_STATE_GPO(4), GPIO_P3_OUTP_SET(GPIO_IOBASE)) )
#define SET_RS        ( __raw_writel(OUTP_STATE_GPO(5), GPIO_P3_OUTP_SET(GPIO_IOBASE)) )
#define RESET_RS      ( __raw_writel(OUTP_STATE_GPO(5), GPIO_P3_OUTP_CLR(GPIO_IOBASE)) )

/*
 * Board specific LCD setup and functions
 */
#if defined (CONFIG_EA3250_DISPLAY_SUPPORT)

#define PANEL_SIZE (3 * SZ_64K)

#if defined (CONFIG_EA3250_QVGA_3_2_LCD)

/*
 * Support for Embedded Artists 3.2 inch QVGA LCD panel
 */

static struct clcd_panel conn_lcd_panel = {
	.mode		= {
		.name		= "QVGA portrait",
		.refresh	= 60,
		.xres		= 240,
		.yres		= 320,
		.pixclock	= 121654,
		.left_margin	= 28,
		.right_margin	= 10,
		.upper_margin	= 2,
		.lower_margin	= 2,
		.hsync_len	= 2,
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

#elif defined (CONFIG_EA3250_QVGA_2_8_OLED)

/*
 * Support for Embedded Artists 2.8 inch QVGA OLED panel
 */

static struct clcd_panel conn_lcd_panel = {
	.mode		= {
		.name		= "QVGA portrait",
		.refresh	= 60,
		.xres		= 240,
		.yres		= 320,
		.pixclock	= 176366,
		.left_margin	= 33,
		.right_margin	= 26,
		.upper_margin	= 3,
		.lower_margin	= 8,
		.hsync_len	= 32,
		.vsync_len	= 4,
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

static void spiSend(u8 data)
{
	u32 sspiobase = io_p2v(SSP0_BASE);

	while( (__raw_readl(SSP_SR(sspiobase)) & SSP_SR_TNF) == 0);
	
	__raw_writel((u32)data, SSP_DATA(sspiobase));

	while( (__raw_readl(SSP_SR(sspiobase)) & SSP_SR_BSY) != 0);
}

#if defined (CONFIG_EA3250_QVGA_2_8_OLED)
static void writeToDisp(u16 data)
{
	ACTIVATE_CS;
	spiSend(0x72);
	spiSend(data >> 8);
	spiSend(data & 0xff);
	DEACTIVATE_CS;   
}
#endif

static void writeToReg(u16 addr, u16 data)
{
#if defined (CONFIG_EA3250_QVGA_3_2_LCD)

	RESET_RS;
	ACTIVATE_CS;
	spiSend(0);
	spiSend(addr & 0xff);
	DEACTIVATE_CS;  

	SET_RS;
	ACTIVATE_CS;
	spiSend(data >> 8);  
	spiSend(data & 0xff);
	DEACTIVATE_CS;  

	RESET_RS;
	ACTIVATE_CS;
	spiSend(0);
	spiSend(0x22);
	DEACTIVATE_CS;  

#elif defined (CONFIG_EA3250_QVGA_2_8_OLED)

	ACTIVATE_CS;
	spiSend(0x70);
	spiSend(data >> 8);
	spiSend(data & 0xff);
	DEACTIVATE_CS;  

#endif

}

static void clcd_display_init(void)
{
	u32 tmp;

	/*
 	 * TODO:
	 *
 	 *   Should use SPI driver and not the registers directly
	 */

	tmp = __raw_readl(CLKPWR_SSP_CLK_CTRL(CLKPWR_IOBASE));
	__raw_writel((tmp | CLKPWR_SSPCTRL_SSPCLK0_EN),
		CLKPWR_SSP_CLK_CTRL(CLKPWR_IOBASE));

	/* setup MUX register to use SSP0 */
	__raw_writel(( _BIT(12) | _BIT(10) | _BIT(9) ), GPIO_P_MUX_SET(GPIO_IOBASE));
	tmp = __raw_readl(GPIO_P_MUX_STATE(GPIO_IOBASE));

	/* Enable SSP0 clock */
	tmp = __raw_readl(CLKPWR_SSP_CLK_CTRL(CLKPWR_IOBASE));
	__raw_writel((tmp | CLKPWR_SSPCTRL_SSPCLK0_EN),
		CLKPWR_SSP_CLK_CTRL(CLKPWR_IOBASE));

	/* Initial setup of SSP0 for transfer */
	__raw_writel(0, SSP_ICR(io_p2v(SSP0_BASE)));
	__raw_writel((SSP_ICR_RORIC | SSP_ICR_RTIC), SSP_IMSC(io_p2v(SSP0_BASE)));

	/* Setup default SPI mode */
	__raw_writel((SSP_CR0_DSS(8) | SSP_CR0_FRF_SPI | SSP_CR0_CPOL(0) |
		SSP_CR0_CPHA(0) | SSP_CR0_SCR(9)), SSP_CR0(io_p2v(SSP0_BASE)));
	__raw_writel(SSP_CR1_SSP_ENABLE, SSP_CR1(io_p2v(SSP0_BASE)));
	__raw_writel(SSP_CPSR_CPDVSR(20), SSP_CPSR(io_p2v(SSP0_BASE)));

	/* Flush RX FIFO */
	while (__raw_readl(SSP_SR(io_p2v(SSP0_BASE))) & SSP_SR_RNE)
	{
		tmp = __raw_readl(SSP_DATA(io_p2v(SSP0_BASE)));
	}


	DEACTIVATE_CS;

#if defined (CONFIG_EA3250_QVGA_3_2_LCD)

	writeToReg (0x00,0x0001);
	mdelay(20);
	writeToReg (0x03,0xA2A4);
	writeToReg (0x0C,0x0004);
	writeToReg (0x0D,0x0308);
	writeToReg (0x0E,0x3000);
	mdelay(50);
	writeToReg (0x1E,0x00AF);
	writeToReg (0x01,0x2B3F);
	writeToReg (0x02,0x0600);
	writeToReg (0x10,0x0000);
	writeToReg (0x07,0x0233);
	writeToReg (0x0B,0x0039);
	writeToReg (0x0F,0x0000);
	mdelay(50);

	writeToReg (0x30,0x0707);
	writeToReg (0x31,0x0204);
	writeToReg (0x32,0x0204);
	writeToReg (0x33,0x0502);
	writeToReg (0x34,0x0507);
	writeToReg (0x35,0x0204);
	writeToReg (0x36,0x0204);
	writeToReg (0x37,0x0502);
	writeToReg (0x3A,0x0302);
	writeToReg (0x3B,0x0302);

	writeToReg (0x23,0x0000);
	writeToReg (0x24,0x0000);

	writeToReg (0x48,0x0000);
	writeToReg (0x49,0x013F);
	writeToReg (0x4A,0x0000);
	writeToReg (0x4B,0x0000);

	writeToReg (0x41,0x0000);
	writeToReg (0x42,0x0000);

	writeToReg (0x44,0xEF00);
	writeToReg (0x45,0x0000);
	writeToReg (0x46,0x013F);
	mdelay(50);

	writeToReg (0x44,0xEF00);
	writeToReg (0x45,0x0000);
	writeToReg (0x4E,0x0000);
	writeToReg (0x4F,0x0000);
	writeToReg (0x46,0x013F);

#elif defined (CONFIG_EA3250_QVGA_2_8_OLED)

	writeToReg(0,0x02);
	writeToDisp(0x0192);
  
	writeToReg(0,0x03);
	writeToDisp(0x0130);
  
  	// set standby off
	writeToReg(0,0x10);
	writeToDisp(0x0000);

	mdelay(100);

	// set display on
	writeToReg(0,0x05);
	writeToDisp(0x0001);

	// enable image data transfer
	writeToReg(0,0x22);
#endif	
}

void clcd_disable(struct clcd_fb *fb)
{
	/* Disable the backlight */
#if defined (CONFIG_EA3250_QVGA_3_2_LCD)
	__raw_writel(OUTP_STATE_GPO(14), GPIO_P3_OUTP_SET(GPIO_IOBASE));
#elif defined (CONFIG_EA3250_QVGA_2_8_OLED)
	__raw_writel(OUTP_STATE_GPO(14), GPIO_P3_OUTP_CLR(GPIO_IOBASE));
#endif
}

void clcd_enable(struct clcd_fb *fb)
{
	clcd_display_init();

	/* Enable the backlight */
#if defined (CONFIG_EA3250_QVGA_3_2_LCD)
	__raw_writel(OUTP_STATE_GPO(14), GPIO_P3_OUTP_CLR(GPIO_IOBASE));
#elif defined (CONFIG_EA3250_QVGA_2_8_OLED)
	__raw_writel(OUTP_STATE_GPO(14), GPIO_P3_OUTP_SET(GPIO_IOBASE));
#endif

}
struct clcd_board lpc32xx_clcd_data = {
#if defined (CONFIG_EA3250_QVGA_3_2_LCD)
	.name		= "Embedded Artists 3.2 inch LCD",
#elif defined (CONFIG_EA3250_QVGA_2_8_OLED)
	.name		= "Embedded Artists 2.8 inch OLED",
#else
	.name		= "Unknown Display",
#endif
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

/* MAC address is provided as a boot paramter (ea_ethaddr) via u-boot */

static u8 mac_address[6] = {0x00, 0x1a, 0xf1, 0x00, 0x00, 0x00};

static int __init ea_ethaddr(char *str)
{
	char *s, *e;
	int i;

	s = str;

	for (i = 0; i < 6; ++i) {
		mac_address[i] = s ? simple_strtoul (s, &e, 16) : 0;
		if (s)
			s = (*e) ? e + 1 : e;
	}

	return 1;
}

__setup("ea_ethaddr=", ea_ethaddr);

static int return_mac_address(u8 *mac)
{
	mac[0] = mac_address[0];
	mac[1] = mac_address[1];
	mac[2] = mac_address[2];
	mac[3] = mac_address[3];
	mac[4] = mac_address[4];
	mac[5] = mac_address[5];

	return 0;
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

#if defined (CONFIG_SENSORS_PCA9532) || defined (CONFIG_AT24)
static struct i2c_board_info __initdata ea3250_i2c_board_info [] = {
  #if defined (CONFIG_SENSORS_PCA9532)
	{
		I2C_BOARD_INFO("pca9532", I2C_PCA9532_ADDR),

	},
  #endif
  #if defined (CONFIG_AT24)
	{
		I2C_BOARD_INFO("24c256", I2C_24LC256_ADDR),
	},
  #endif
};
#endif


static struct platform_device* ea3250_devs[] __initdata = {
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
void __init ea3250_board_init(void)
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

#if defined (CONFIG_EA3250_DISPLAY_SUPPORT)
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

	/* Call chip specific init */
	lpc32xx_init();

	/* Add board platform devices */
	platform_add_devices (ea3250_devs, ARRAY_SIZE (ea3250_devs));


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

#if defined (CONFIG_SENSORS_PCA9532) || defined (CONFIG_AT24)
	i2c_register_board_info(0, ea3250_i2c_board_info,
				ARRAY_SIZE(ea3250_i2c_board_info));
#endif

}

MACHINE_START (LPC3XXX, "Embedded Artists LPC3250 OEM board with the LPC3250 Microcontroller")
	/* Maintainer: Embedded Artists */
	.phys_io	= UART5_BASE,
	.io_pg_offst	= ((io_p2v (UART5_BASE))>>18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= lpc32xx_map_io,
	.init_irq	= lpc32xx_init_irq,
	.timer		= &lpc32xx_timer,
	.init_machine	= ea3250_board_init,
MACHINE_END

