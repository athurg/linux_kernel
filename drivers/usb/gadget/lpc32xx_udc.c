/*
 * drivers/usb/gadget/lpc32xx_udc.c
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
 *
 * Copyright (C) 2009 NXP Semiconductors
 * Copyright (C) 2006 Mike James , Philips Semiconductors
 *
 * Note: This driver is based on original work done by Mike James for
 *       the LPC3180.
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

/*
 * Notes
 *
 * ISO functionality is untested. It probably works, but may not be reliable.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/i2c.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>

#include <asm/byteorder.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>

#include <mach/platform.h>
#include <mach/irqs.h>
#include <mach/lpc32xx_usbd.h>
#include <mach/board.h>
#include "lpc32xx_udc.h"
#ifdef CONFIG_USB_GADGET_DEBUG_FILES
#include <linux/seq_file.h>
#endif

#define UDCA_BUFF_SIZE (128)

#define USB_CTRL	IO_ADDRESS(CLK_PM_BASE + 0x64)
#define USB_CLOCK_MASK (AHB_M_CLOCK_ON| OTG_CLOCK_ON | DEV_CLOCK_ON | I2C_CLOCK_ON)

/* USB_CTRL bit defines */
#define USB_SLAVE_HCLK_EN	(1 << 24)
#define USB_HOST_NEED_CLK_EN	(1 << 21)
#define USB_DEV_NEED_CLK_EN	(1 << 22)

#define USB_OTG_CLK_CTRL	IO_ADDRESS(USB_BASE + 0xFF4)
#define USB_OTG_CLK_STAT	IO_ADDRESS(USB_BASE + 0xFF8)

/* USB_OTG_CLK_CTRL bit defines */
#define AHB_M_CLOCK_ON		(1 << 4)
#define OTG_CLOCK_ON		(1 << 3)
#define I2C_CLOCK_ON		(1 << 2)
#define DEV_CLOCK_ON		(1 << 1)
#define HOST_CLOCK_ON		(1 << 0)

#define USB_OTG_STAT_CONTROL	IO_ADDRESS(USB_BASE + 0x110)

/* USB_OTG_STAT_CONTROL bit defines */
#define TRANSPARENT_I2C_EN	(1 << 7)
#define HOST_EN			(1 << 0)

/* ISP1301 USB transceiver I2C registers */
#define	ISP1301_MODE_CONTROL_1		0x04	/* u8 read, set, +1 clear */

#define	MC1_SPEED_REG		(1 << 0)
#define	MC1_SUSPEND_REG		(1 << 1)
#define	MC1_DAT_SE0		(1 << 2)
#define	MC1_TRANSPARENT		(1 << 3)
#define	MC1_BDIS_ACON_EN	(1 << 4)
#define	MC1_OE_INT_EN		(1 << 5)
#define	MC1_UART_EN		(1 << 6)
#define	MC1_MASK		0x7f

#define	ISP1301_MODE_CONTROL_2		0x12	/* u8 read, set, +1 clear */

#define	MC2_GLOBAL_PWR_DN	(1 << 0)
#define	MC2_SPD_SUSP_CTRL	(1 << 1)
#define	MC2_BI_DI		(1 << 2)
#define	MC2_TRANSP_BDIR0	(1 << 3)
#define	MC2_TRANSP_BDIR1	(1 << 4)
#define	MC2_AUDIO_EN		(1 << 5)
#define	MC2_PSW_EN		(1 << 6)
#define	MC2_EN2V7		(1 << 7)

#define	ISP1301_OTG_CONTROL_1		0x06	/* u8 read, set, +1 clear */
#define	OTG1_DP_PULLUP		(1 << 0)
#define	OTG1_DM_PULLUP		(1 << 1)
#define	OTG1_DP_PULLDOWN	(1 << 2)
#define	OTG1_DM_PULLDOWN	(1 << 3)
#define	OTG1_ID_PULLDOWN	(1 << 4)
#define	OTG1_VBUS_DRV		(1 << 5)
#define	OTG1_VBUS_DISCHRG	(1 << 6)
#define	OTG1_VBUS_CHRG		(1 << 7)
#define	ISP1301_OTG_STATUS		0x10	/* u8 readonly */
#define	OTG_B_SESS_END		(1 << 6)
#define	OTG_B_SESS_VLD		(1 << 7)

#define INT_CR_INT		(1 << 7)
#define INT_BDIS_ACON		(1 << 6)
#define INT_ID_FLOAT		(1 << 5)
#define INT_DM_HI		(1 << 4)
#define INT_ID_GND		(1 << 3)
#define INT_DP_HI		(1 << 2)
#define INT_SESS_VLD		(1 << 1)
#define INT_VBUS_VLD		(1 << 0)

#define ISP1301_I2C_ADDR 0x2C

#define ISP1301_I2C_MODE_CONTROL_1 0x4
#define ISP1301_I2C_MODE_CONTROL_2 0x12
#define ISP1301_I2C_OTG_CONTROL_1 0x6
#define ISP1301_I2C_OTG_CONTROL_2 0x10
#define ISP1301_I2C_INTERRUPT_SOURCE 0x8
#define ISP1301_I2C_INTERRUPT_LATCH 0xA
#define ISP1301_I2C_INTERRUPT_FALLING 0xC
#define ISP1301_I2C_INTERRUPT_RISING 0xE
#define ISP1301_I2C_REG_CLEAR_ADDR 1

#define	DRIVER_VERSION	"$Revision: 1.01 $"
static const char driver_name [] = "lpc32xx_udc";

static void udc_set_address(struct lpc32xx_udc *udc, u32 addr);
#if defined(UDC_ENABLE_DMA)
static int udc_ep_in_req_dma(struct lpc32xx_udc *udc, struct lpc32xx_ep *ep);
static int udc_ep_out_req_dma(struct lpc32xx_udc *udc, struct lpc32xx_ep *ep);
#else
static int udc_ep_in_req(struct lpc32xx_udc *udc, struct lpc32xx_ep *ep);
static int udc_ep_out_req(struct lpc32xx_udc *udc, struct lpc32xx_ep *ep);
#endif
static int udc_ep0_in_req(struct lpc32xx_udc *udc);
static int udc_ep0_out_req(struct lpc32xx_udc *udc);

/*
 *
 * proc interface support
 *
 */
#ifdef CONFIG_USB_GADGET_DEBUG_FILES
static char *epnames[] = {"INT", "ISO", "BULK", "CTRL"};
static const char debug_filename[] = "driver/udc";

static void proc_ep_show(struct seq_file *s, struct lpc32xx_ep *ep)
{
	struct lpc32xx_request *req;
	unsigned long flags;

	local_irq_save(flags);

	seq_printf(s, "\n");
	seq_printf(s, "%12s, maxpacket %4d %3s",
			ep->ep.name, ep->ep.maxpacket,
			ep->is_in ? "in" : "out");
	seq_printf(s, " type %4s", epnames[ep->eptype]);
	seq_printf(s, " ints: %12d", ep->totalints);
	seq_printf(s, " stalls: %12d", ep->totalstalls);
	seq_printf(s, " NAKs: %12d\n", ep->totalnaks);

	if (list_empty (&ep->queue))
		seq_printf(s, "\t(queue empty)\n");
	else {
		list_for_each_entry (req, &ep->queue, queue) {
			u32 length = req->req.actual;

			seq_printf(s, "\treq %p len %d/%d buf %p\n",
				&req->req, length,
				req->req.length, req->req.buf);
		}
	}

	local_irq_restore(flags);
}

static int proc_udc_show(struct seq_file *s, void *unused)
{
	struct lpc32xx_udc *udc = s->private;
	struct lpc32xx_ep *ep;

	seq_printf(s, "%s: version %s\n", driver_name, DRIVER_VERSION);

	seq_printf(s, "vbus %s, pullup %s, %s powered%s, gadget %s\n\n",
		udc->vbus ? "present" : "off",
		udc->enabled
			? (udc->vbus ? "active" : "enabled")
			: "disabled",
		udc->selfpowered ? "self" : "VBUS",
		udc->suspended ? ", suspended" : "",
		udc->driver ? udc->driver->driver.name : "(none)");

	if (udc->enabled && udc->vbus) {
		proc_ep_show(s, &udc->ep[0]);
		list_for_each_entry (ep, &udc->gadget.ep_list, ep.ep_list) {
			if (ep->desc) {
				proc_ep_show(s, ep);
			}
		}
	}

	return 0;
}

static int proc_udc_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_udc_show, PDE(inode)->data);
}

static const struct file_operations proc_ops = {
	.owner		= THIS_MODULE,
	.open		= proc_udc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_debug_file(struct lpc32xx_udc *udc)
{
	udc->pde = proc_create_data(debug_filename, 0, NULL, &proc_ops, udc);
}

static void remove_debug_file(struct lpc32xx_udc *udc)
{
	if (udc->pde)
		remove_proc_entry(debug_filename, NULL);
}

#else
static inline void create_debug_file(struct lpc32xx_udc *udc) {}
static inline void remove_debug_file(struct lpc32xx_udc *udc) {}
#endif

/*
 *
 * ISP1301 transceiver support functions
 *
 */
struct i2c_driver isp1301_driver;
struct i2c_client *isp1301_i2c_client;

static int isp1301_probe(struct i2c_adapter *adap);
static int isp1301_detach(struct i2c_client *client);

static const unsigned short normal_i2c[] =
    { ISP1301_I2C_ADDR, I2C_CLIENT_END };
static const unsigned short dummy_i2c_addrlist[] = { I2C_CLIENT_END };

static struct i2c_client_address_data addr_data = {
	.normal_i2c = normal_i2c,
	.probe = dummy_i2c_addrlist,
	.ignore = dummy_i2c_addrlist,
};

struct i2c_driver isp1301_driver = {
	.driver = {
		.name = "isp1301_pnx",
	},
	.attach_adapter = isp1301_probe,
	.detach_client = isp1301_detach,
};

static int isp1301_attach(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *c;
	int err;

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	strlcpy(c->name, "isp1301_pnx", I2C_NAME_SIZE);
	c->flags = 0;
	c->addr = addr;
	c->adapter = adap;
	c->driver = &isp1301_driver;

	err = i2c_attach_client(c);
	if (err) {
		kfree(c);
		return err;
	}

	isp1301_i2c_client = c;

	return 0;
}

static int isp1301_probe(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, isp1301_attach);
}

static int isp1301_detach(struct i2c_client *client)
{
	i2c_detach_client(client);
	kfree(isp1301_i2c_client);
	return 0;
}

static void i2c_write(u8 buf, u8 subaddr)
{
	char tmpbuf[2];

	tmpbuf[0] = subaddr;	/*register number */
	tmpbuf[1] = buf;	/*register data */
	i2c_master_send(isp1301_i2c_client, &tmpbuf[0], 2);
}

static u16 i2c_read(u8 subaddr)
{
	u8 data;

	i2c_master_send(isp1301_i2c_client, &subaddr, 1);
	i2c_master_recv(isp1301_i2c_client, (u8 *) &data, 1);

	return data;
}

static u16 i2c_read16(u8 subaddr)
{
	u16 data;

	i2c_master_send(isp1301_i2c_client, &subaddr, 1);
	i2c_master_recv(isp1301_i2c_client, (u8 *) &data, 2);

	return data;
}

/* Primary initializion sequence for the ISP1301 transceiver */
static void isp1301_udc_configure(struct lpc32xx_udc *udc)
{
	/* LPC32XX only supports DAT_SE0 USB mode */
	/* This sequence is important */

	/* Disable transparent UART mode first */
	i2c_write(MC1_UART_EN, (ISP1301_I2C_MODE_CONTROL_1 |
		ISP1301_I2C_REG_CLEAR_ADDR));

	/* Set full speed and SE0 mode */
	i2c_write(~0, (ISP1301_I2C_MODE_CONTROL_1 | ISP1301_I2C_REG_CLEAR_ADDR));
	i2c_write((MC1_SPEED_REG | MC1_DAT_SE0), ISP1301_I2C_MODE_CONTROL_1);

	/* The PSW_OE enable bit state is reversed in the ISP1301 User's guide! */
	i2c_write(~0, (ISP1301_I2C_MODE_CONTROL_2 | ISP1301_I2C_REG_CLEAR_ADDR));
	i2c_write((MC2_BI_DI | MC2_SPD_SUSP_CTRL), ISP1301_I2C_MODE_CONTROL_2);

	/* Driver VBUS_DRV high or low depending on board setup */
	if (udc->board->vbus_drv_pol != 0) {
		i2c_write(OTG1_VBUS_DRV, ISP1301_I2C_OTG_CONTROL_1);
	}
	else {
		i2c_write(OTG1_VBUS_DRV, (ISP1301_I2C_OTG_CONTROL_1 |
			ISP1301_I2C_REG_CLEAR_ADDR));
	}

	/* Bi-derctional mode with suspend control */
	/* Enable both pulldowns for now - the pullup will be enable when VBUS is detected */
	i2c_write(~0, (ISP1301_I2C_OTG_CONTROL_1 | ISP1301_I2C_REG_CLEAR_ADDR));
	i2c_write((0 | OTG1_DM_PULLDOWN | OTG1_DP_PULLDOWN),
		ISP1301_I2C_OTG_CONTROL_1);

	/* Discharge VBUS (just in case) */
	i2c_write(OTG1_VBUS_DISCHRG, ISP1301_I2C_OTG_CONTROL_1);
	mdelay(1);
	i2c_write(OTG1_VBUS_DISCHRG,
		(ISP1301_I2C_OTG_CONTROL_1 | ISP1301_I2C_REG_CLEAR_ADDR));

	/* Clear and enable VBUS high edge interrupt */
	i2c_write(~0, ISP1301_I2C_INTERRUPT_LATCH | ISP1301_I2C_REG_CLEAR_ADDR);
	i2c_write(~0, ISP1301_I2C_INTERRUPT_FALLING | ISP1301_I2C_REG_CLEAR_ADDR);
	i2c_write(INT_VBUS_VLD, ISP1301_I2C_INTERRUPT_FALLING);
	i2c_write(~0, ISP1301_I2C_INTERRUPT_RISING | ISP1301_I2C_REG_CLEAR_ADDR);
	i2c_write(INT_VBUS_VLD, ISP1301_I2C_INTERRUPT_RISING);

	/* Enable usb_need_clk clock after transceiver is initialized */
	__raw_writel((__raw_readl(USB_CTRL) | (1 << 22)), USB_CTRL);

	dev_dbg(udc->dev, "ISP1301 Vendor ID  : 0x%04x\n", i2c_read16(0x00));
	dev_dbg(udc->dev, "ISP1301 Product ID : 0x%04x\n", i2c_read16(0x02));
	dev_dbg(udc->dev, "ISP1301 Version ID : 0x%04x\n", i2c_read16(0x14));
}

/* Enables or disables the USB device pullup via the ISP1301 transceiver */
static void isp1301_pullup_enable(int en_pullup)
{
	if (en_pullup) {
		/* Enable pullup for bus signalling */
		i2c_write(OTG1_DP_PULLUP, ISP1301_I2C_OTG_CONTROL_1);
	}
	else {
		/* Enable pullup for bus signalling */
		i2c_write(OTG1_DP_PULLUP,
			(ISP1301_I2C_OTG_CONTROL_1 | ISP1301_I2C_REG_CLEAR_ADDR));
	}
}

#ifdef CONFIG_PM
/* Powers up or down the ISP1301 transceiver */
static void isp1301_set_powerstate(int enable) {
	if (enable != 0) {
		/* Power up ISP1301 - this ISP1301 will automatically wakeup
		   when VBUS is detected */
		i2c_write(MC2_GLOBAL_PWR_DN, 
			(ISP1301_I2C_MODE_CONTROL_2 | ISP1301_I2C_REG_CLEAR_ADDR));
	}
	else {
		/* Power down ISP1301 */
		i2c_write(MC2_GLOBAL_PWR_DN, ISP1301_I2C_MODE_CONTROL_2);
	}
}
#endif

/*
 *
 * USB protocol engine command/data read/write helper functions
 *
 */
/* Issues a single command to the USB device state machine */
static void udc_protocol_cmd_w(struct lpc32xx_udc *udc, u32 cmd) {
	u32 pass = 0;
	int to;

	/* EP may lock on CLRI if this read isn't done */
	volatile u32 tmp = __raw_readl(USBD_DEVINTST(udc->udp_baseaddr));
	(void) tmp;

	while (pass == 0) {
		__raw_writel(USBD_CCEMPTY, USBD_DEVINTCLR(udc->udp_baseaddr));

		/* Write command code */
		__raw_writel(cmd, USBD_CMDCODE(udc->udp_baseaddr));
		to = 10000;
		while (((__raw_readl(USBD_DEVINTST(udc->udp_baseaddr)) &
			USBD_CCEMPTY) == 0) && (to > 0)) {
			to--;
		}

		if (to > 0) pass = 1;
	}
}

/* Issues 2 commands (or command and data) to the USB device state machine */
static inline void udc_protocol_cmd_data_w(struct lpc32xx_udc *udc, u32 cmd, u32 data) {
	udc_protocol_cmd_w(udc, cmd);
	udc_protocol_cmd_w(udc, data);
}

/* Issues a single command to the USB device state machine and reads
   response data */
static u32 udc_protocol_cmd_r(struct lpc32xx_udc *udc, u32 cmd) {
	/* Write a command and read data from the protocol engine */
	u32 tmp;

	__raw_writel((USBD_CDFULL | USBD_CCEMPTY),
		USBD_DEVINTCLR(udc->udp_baseaddr));

	/* Write command code */
	udc_protocol_cmd_w(udc, cmd);
	while ((__raw_readl(USBD_DEVINTST(udc->udp_baseaddr)) &
		USBD_CDFULL) == 0);

	tmp = __raw_readl(USBD_CMDDATA(udc->udp_baseaddr));

	return tmp;
}

/*
 *
 * USB device interrupt mask support functions
 *
 */
/* Enable one or more USB device interrupts */
static inline void uda_enable_devint(struct lpc32xx_udc *udc, u32 devmask) {
	udc->enabled_devints |= devmask;
	__raw_writel(udc->enabled_devints, USBD_DEVINTEN(udc->udp_baseaddr));
}

/* Disable one or more USB device interrupts */
static inline void uda_disable_devint(struct lpc32xx_udc *udc, u32 mask) {
	udc->enabled_devints &= ~mask;
	__raw_writel(udc->enabled_devints, USBD_DEVINTEN(udc->udp_baseaddr));
}

/* Clear one or more USB device interrupts */
static inline void uda_clear_devint(struct lpc32xx_udc *udc, u32 mask) {
	__raw_writel(mask, USBD_DEVINTCLR(udc->udp_baseaddr));
}

/*
 *
 * Endpoint interrupt disable/enable functions
 *
 */
/* Enable one or more USB endpoint interrupts */
static void uda_enable_hwepint(struct lpc32xx_udc *udc, u32 hwep) {
	udc->enabled_hwepints |= (1 << hwep);
	__raw_writel(udc->enabled_hwepints, USBD_EPINTEN(udc->udp_baseaddr));
}

/* Disable one or more USB endpoint interrupts */
static void uda_disable_hwepint(struct lpc32xx_udc *udc, u32 hwep) {
	udc->enabled_hwepints &= ~(1 << hwep);
	__raw_writel(udc->enabled_hwepints, USBD_EPINTEN(udc->udp_baseaddr));
}

/* Clear one or more USB endpoint interrupts */
static inline void uda_clear_hwepint(struct lpc32xx_udc *udc, u32 hwep) {
	__raw_writel((1 << hwep), USBD_EPINTCLR(udc->udp_baseaddr));
}

/* Enable DMA for the HW channel */
static inline void udc_ep_dma_enable(struct lpc32xx_udc *udc, u32 hwep) {
	__raw_writel((1 << hwep), USBD_EPDMAEN(udc->udp_baseaddr));
}

/* Disable DMA for the HW channel */
static inline void udc_ep_dma_disable(struct lpc32xx_udc *udc, u32 hwep) {
	__raw_writel((1 << hwep), USBD_EPDMADIS(udc->udp_baseaddr));
}

/*
 *
 * Endpoint realize/unrealize functions
 *
 */
/* Before an endpoint can be used, it needs to be realized
   in the USB protocol engine - this realizes the endpoint.
   The interrupt (FIFO or DMA) is not enabled with this function */
static void udc_realize_hwep(struct lpc32xx_udc *udc, u32 hwep,
				u32 maxpacket) {
	__raw_writel(USBD_EP_RLZED, USBD_DEVINTCLR(udc->udp_baseaddr));
	__raw_writel(hwep, USBD_EPIND(udc->udp_baseaddr));
	udc->realized_eps |= (1 << hwep);
	__raw_writel(udc->realized_eps, USBD_REEP(udc->udp_baseaddr));
	__raw_writel(maxpacket, USBD_EPMAXPSIZE(udc->udp_baseaddr));

	/* Wait until endpoint is realized in hardware */
	while (!(__raw_readl(USBD_DEVINTST(udc->udp_baseaddr)) & USBD_EP_RLZED));
	__raw_writel(USBD_EP_RLZED, USBD_DEVINTCLR(udc->udp_baseaddr));
}

/* Unrealize an EP */
static void udc_unrealize_hwep(struct lpc32xx_udc *udc, u32 hwep) {
	udc->realized_eps &= ~(1 << hwep);
	__raw_writel(udc->realized_eps, USBD_REEP(udc->udp_baseaddr));
}

/*
 *
 * Endpoint support functions
 *
 */
/* Select and clear endpoint interrupt */
static u32 udc_selep_clrint(struct lpc32xx_udc *udc, u32 hwep) {
	udc_protocol_cmd_w(udc, CMD_SEL_EP_CLRI(hwep));
	return udc_protocol_cmd_r(udc, DAT_SEL_EP_CLRI(hwep));
}

/* Disables the endpoint in the USB protocol engine */
static void udc_disable_hwep(struct lpc32xx_udc *udc, u32 hwep) {
	udc_protocol_cmd_data_w(udc, CMD_SET_EP_STAT(hwep),
		DAT_WR_BYTE(EP_STAT_DA));
}

/* Stalls the endpoint - endpoint will return STALL */
static void udc_stall_hwep(struct lpc32xx_udc *udc, u32 hwep) {
	udc_protocol_cmd_data_w(udc, CMD_SET_EP_STAT(hwep),
		DAT_WR_BYTE(EP_STAT_ST));
}

/* Clear stall or reset endpoint */
static void udc_clrstall_hwep(struct lpc32xx_udc *udc, u32 hwep) {
	udc_protocol_cmd_data_w(udc, CMD_SET_EP_STAT(hwep),
		DAT_WR_BYTE(0));
}

/* Select an endpoint for endpoint status, clear, validate */
static void udc_select_hwep(struct lpc32xx_udc *udc, u32 hwep) {
	udc_protocol_cmd_w(udc, CMD_SEL_EP(hwep));
}

/*
 *
 * Endpoint buffer management functions
 *
 */
/* Clear the current endpoint's buffer */
static void udc_clr_buffer_hwep(struct lpc32xx_udc *udc, u32 hwep) {
	udc_select_hwep(udc, hwep);
	udc_protocol_cmd_w(udc, CMD_CLR_BUF);
}

/* Validate the current endpoint's buffer */
static void udc_val_buffer_hwep(struct lpc32xx_udc *udc, u32 hwep) {
	udc_select_hwep(udc, hwep);
	udc_protocol_cmd_w(udc, CMD_VALID_BUF);
}

static inline u32 udc_clearep_getsts(struct lpc32xx_udc *udc, u32 hwep) {
	/* Clear EP interrupt */
	uda_clear_hwepint(udc, hwep);
	return udc_selep_clrint(udc, hwep);
}

#if defined(UDC_ENABLE_DMA)
/*
 *
 * USB EP DMA support
 *
 */
/* Allocate a DMA Descriptor */
static struct lpc32xx_usbd_dd_gad *udc_dd_alloc(struct lpc32xx_udc *udc) {
	dma_addr_t			dma;
	struct lpc32xx_usbd_dd_gad	*dd;

	dd = (struct lpc32xx_usbd_dd_gad *) dma_pool_alloc(
		udc->dd_cache, (GFP_KERNEL | GFP_DMA), &dma);
	if (dd) {
		dd->this_dma = dma;
	}

	return dd;
}

/* Free a DMA Descriptor */
static void udc_dd_free(struct lpc32xx_udc *udc, struct lpc32xx_usbd_dd_gad *dd)
{
	dma_pool_free(udc->dd_cache, dd, dd->this_dma);
}
#endif

/*
 *
 * USB setup and shutdown functions
 *
 */
/* Enables or disables most of the USB system clocks when low power mode is
   needed. Clocks are typically started on a connection event, and disabled
   when a cable is disconnected */
#define OTGOFF_CLK_MASK (AHB_M_CLOCK_ON | I2C_CLOCK_ON)
static void udc_clk_set(struct lpc32xx_udc *udc, int enable)
{
	if (enable != 0) {
		if (udc->clocked)
			return;

		udc->clocked = 1;

		/* 48MHz PLL up */
		clk_enable(udc->usb_pll_clk);

		/* Enable the USb device clock */
		__raw_writel(__raw_readl(USB_CTRL) | USB_DEV_NEED_CLK_EN, USB_CTRL);

		/* Set to enable all needed USB OTG clocks */
		__raw_writel(USB_CLOCK_MASK, USB_OTG_CLK_CTRL);

		while ((__raw_readl(USB_OTG_CLK_STAT) & USB_CLOCK_MASK) !=
		       USB_CLOCK_MASK);
	}
	else {
		if (!udc->clocked)
			return;

		udc->clocked = 0;
		udc->gadget.speed = USB_SPEED_UNKNOWN;

		/* Never disable the USB_HCLK during normal operation */

		/* 48MHz PLL dpwn */
		clk_disable(udc->usb_pll_clk);

		/* Enable the USb device clock */
		__raw_writel(__raw_readl(USB_CTRL) & ~USB_DEV_NEED_CLK_EN, USB_CTRL);

		/* Set to enable all needed USB OTG clocks */
		__raw_writel(OTGOFF_CLK_MASK, USB_OTG_CLK_CTRL);

		while ((__raw_readl(USB_OTG_CLK_STAT) & OTGOFF_CLK_MASK) !=
		       OTGOFF_CLK_MASK);
	}
}

static void udc_disable(struct lpc32xx_udc *udc) {
	u32 i;

	/* Disable device */
	udc_protocol_cmd_data_w(udc, CMD_CFG_DEV, DAT_WR_BYTE(0));
	udc_protocol_cmd_data_w(udc, CMD_SET_DEV_STAT, DAT_WR_BYTE(0));

	/* Disable all device interrupts (including EP0) */
	uda_disable_devint(udc, 0x3FF);

	/* Disable and reset all endpoint interrupts */
	for (i = 0; i < 32; i++) {
		uda_disable_hwepint(udc, i);
		uda_clear_hwepint(udc, i);
		udc_disable_hwep(udc, i);
		udc_unrealize_hwep(udc, i);
		udc->udca_v_base [i] = 0;

		/* Disable and clear all interrupts and DMA */
		udc_ep_dma_disable(udc, i);
		__raw_writel((1 << i), USBD_EOTINTCLR(udc->udp_baseaddr));
		__raw_writel((1 << i), USBD_NDDRTINTCLR(udc->udp_baseaddr));
		__raw_writel((1 << i), USBD_SYSERRTINTCLR(udc->udp_baseaddr));
		__raw_writel((1 << i), USBD_DMARCLR(udc->udp_baseaddr));
	}

	/* Disable DMA interrupts */
	__raw_writel(0, USBD_DMAINTEN(udc->udp_baseaddr));

	__raw_writel(0, USBD_UDCAH(udc->udp_baseaddr));
}

static void udc_enable(struct lpc32xx_udc *udc)
{
	u32 i;
	struct lpc32xx_ep *ep = &udc->ep[0];

	udc->gadget.speed = USB_SPEED_UNKNOWN;

	/* Start with known state */
	udc_disable(udc);

	/* Enable device */
	udc_protocol_cmd_data_w(udc, CMD_SET_DEV_STAT, DAT_WR_BYTE(DEV_CON));

	/* EP interrupts on high priority, FRAME interrupt on low priority */
	__raw_writel(USBD_EP_FAST, USBD_DEVINTPRI(udc->udp_baseaddr));
	__raw_writel(0xFFFF, USBD_EPINTPRI(udc->udp_baseaddr));

	/* Clear any pending device interrupts */
	__raw_writel(0x3FF, USBD_DEVINTCLR(udc->udp_baseaddr));

	/* Setup UDCA - not yet used (DMA) */
	__raw_writel((u32) udc->udca_p_base, USBD_UDCAH(udc->udp_baseaddr));

	/* Only enable EP0 in and out for now, EP0 only works in FIFO mode */
	for (i = 0; i <= 1; i++) {
		udc_realize_hwep(udc, i, ep->ep.maxpacket);
		uda_enable_hwepint(udc, i);
		udc_select_hwep(udc, i);
		udc_clrstall_hwep(udc, i);
		udc_clr_buffer_hwep(udc, i);
	}

	/* Device interrupt setup */
	uda_clear_devint(udc, (USBD_ERR_INT | USBD_DEV_STAT | USBD_EP_SLOW |
		USBD_EP_FAST));
	uda_enable_devint(udc, (USBD_ERR_INT | USBD_DEV_STAT | USBD_EP_SLOW |
		USBD_EP_FAST));

	/* Set device address to 0 - called twice to force a latch in the USB
	   engine without the need of a setup packet status closure */
	udc_set_address(udc, 0);
	udc_set_address(udc, 0);

#if defined(UDC_ENABLE_DMA)
	/* Enable master DMA interrupts */
	__raw_writel((USBD_SYS_ERR_INT | USBD_EOT_INT), USBD_DMAINTEN(udc->udp_baseaddr));
#endif

	udc->dev_status = 0;
}

/*
 *
 * USB device board specific events handled via callbacks
 *
 */
/* Connection change event - notify board function of change */
static void uda_power_event(struct lpc32xx_udc *udc, u32 conn) {
	/* Just notify of a connection change event (optional) */
	if (udc->board->conn_chgb != NULL) {
		udc->board->conn_chgb(conn);
	}
}

/* Suspend/resume event - notify board function of change */
static void uda_resm_susp_event(struct lpc32xx_udc *udc, u32 conn) {
	/* Just notify of a Suspend/resume change event (optional) */
	if (udc->board->susp_chgb != NULL) {
		udc->board->susp_chgb(conn);
	}

	if (conn)
		udc->suspended = 0;
	else
		udc->suspended = 1;
}

/* Remote wakeup enable/disable - notify board function of change */
static void uda_remwkp_cgh(struct lpc32xx_udc *udc) {
	if (udc->board->rmwk_chgb != NULL) {
		udc->board->rmwk_chgb(udc->dev_status &
			(1 << USB_DEVICE_REMOTE_WAKEUP));
	}
}

/* Reads data from FIFO, adjusts for alignment and data size */
static void udc_pop_fifo(struct lpc32xx_udc *udc, u8 *data, u32 bytes) {
	int n, i, bl;
	u16 *p16;
	u32 *p32, tmp, cbytes;

	/* Use optimal data transfer method based on source address and size */
	switch (((u32) data) & 0x3) {
	case 0: /* 32-bit aligned */
		p32 = (u32 *) data;
		cbytes = (bytes & ~0x3);

		/* Copy 32-bit aligned data first */
		for (n = 0; n < cbytes; n += 4)
			*p32++ = __raw_readl(USBD_RXDATA(udc->udp_baseaddr));

		/* Handle any remaining bytes */
		bl = bytes - cbytes;
		if (bl) {
			tmp = __raw_readl(USBD_RXDATA(udc->udp_baseaddr));
			for (n = 0; n < bl; n++)
				data[cbytes + n] = ((tmp >> (n * 8)) & 0xFF);

		}
		break;

	case 1: /* 8-bit aligned */
	case 3:
		/* Each byte has to be handled independently */
		for (n = 0; n < bytes; n += 4) {
			tmp = __raw_readl(USBD_RXDATA(udc->udp_baseaddr));

			bl = bytes - n;
			if (bl > 3)
				bl = 3;

			for (i = 0; i < bl; i++)
				data[n + i] = (u8) ((tmp >> (n * 8)) & 0xFF);
		}
		break;

	case 2: /* 16-bit aligned */
		p16 = (u16 *) data;
		cbytes = (bytes & ~0x3);

		/* Copy 32-bit sized objects first with 16-bit alignment */		
		for (n = 0; n < cbytes; n += 4) {
			tmp = __raw_readl(USBD_RXDATA(udc->udp_baseaddr));
			*p16++ = (u16) (tmp & 0xFFFF);
			*p16++ = (u16) ((tmp >> 16) & 0xFFFF);
		}

		/* Handle any remaining bytes */
		bl = bytes - cbytes;
		if (bl) {
			tmp = __raw_readl(USBD_RXDATA(udc->udp_baseaddr));
			for (n = 0; n < bl; n++)
				data[cbytes + n] = ((tmp >> (n * 8)) & 0xFF);
		}
		break;
	}
}

/* Read data from the FIFO for an endpoint. This function is for endpoints (such
   as EP0) that don't use DMA. This function should only be called if a packet
   is known to be ready to read for the endpopint. Note that the endpoint must
   be selected in the protocol engine prior to this call. */
static u32 udc_read_hwep(struct lpc32xx_udc *udc, u32 hwep, u32 *data,
			int bytes) {
	volatile u32 tmpv;
	u32 tmp, hwrep = ((hwep & 0x1E) << 1) | CTRL_RD_EN;

	/* Setup read of endpoint */
	__raw_writel(hwrep, USBD_CTRL(udc->udp_baseaddr));
	__raw_writel(hwrep, USBD_CTRL(udc->udp_baseaddr));
	while (__raw_readl(USBD_CTRL(udc->udp_baseaddr)) != hwrep) {
		__raw_writel(hwrep, USBD_CTRL(udc->udp_baseaddr));
	}

	/* Wait until packet is ready */
	tmpv = 0;
	while ((tmpv & PKT_RDY) == 0) {
		tmpv = __raw_readl(USBD_RXPLEN(udc->udp_baseaddr));
	}

	/* Mask out count */
	tmp = tmpv & PKT_LNGTH_MASK;
	if (bytes < tmp) {
		tmp = (u32) bytes;
	}

	if ((tmp > 0) && (data != NULL)) {
		udc_pop_fifo(udc, (u8 *) data, tmp);
	}

	__raw_writel(((hwep & 0x1E) << 1), USBD_CTRL(udc->udp_baseaddr));
	__raw_writel(((hwep & 0x1E) << 1), USBD_CTRL(udc->udp_baseaddr));

	/* Clear the buffer */
	udc_clr_buffer_hwep(udc, hwep);

	return tmp;
}

/* Stuffs data into the FIFO, adjusts for alignment and data size */
static void udc_stuff_fifo(struct lpc32xx_udc *udc, u8 *data, u32 bytes) {
	int n, i, bl;
	u16 *p16;
	u32 *p32, tmp, cbytes;

	/* Use optimal data transfer method based on source address and size */
	switch (((u32) data) & 0x3) {
	case 0: /* 32-bit aligned */
		p32 = (u32 *) data;
		cbytes = (bytes & ~0x3);

		/* Copy 32-bit aligned data first */
		for (n = 0; n < cbytes; n += 4)
			__raw_writel(*p32++, USBD_TXDATA(udc->udp_baseaddr));

		/* Handle any remaining bytes */
		bl = bytes - cbytes;
		if (bl) {
			tmp = 0;
			for (n = 0; n < bl; n++)
				tmp |= (u32) (data[cbytes + n] << (n * 8));

			__raw_writel(tmp, USBD_TXDATA(udc->udp_baseaddr));
		}
		break;

	case 1: /* 8-bit aligned */
	case 3:
		/* Each byte has to be handled independently */
		for (n = 0; n < bytes; n += 4) {
			bl = bytes - n;
			if (bl > 4)
				bl = 4;

			tmp = 0;
			for (i = 0; i < bl; i++)
				tmp |= (u32) (data[n + i] << (i * 8));

			__raw_writel(tmp, USBD_TXDATA(udc->udp_baseaddr));
		}
		break;

	case 2: /* 16-bit aligned */
		p16 = (u16 *) data;
		cbytes = (bytes & ~0x3);

		/* Copy 32-bit aligned data first */
		for (n = 0; n < cbytes; n += 4) {
			tmp = (u32) (*p16++ & 0xFFFF);
			tmp |= (u32) ((*p16++ & 0xFFFF) << 16);
			__raw_writel(tmp, USBD_TXDATA(udc->udp_baseaddr));
		}

		/* Handle any remaining bytes */
		bl = bytes - cbytes;
		if (bl) {
			tmp = 0;
			for (n = 0; n < bl; n++)
				tmp |= (u32) (data[cbytes + n] << (n * 8));

			__raw_writel(tmp, USBD_TXDATA(udc->udp_baseaddr));
		}
		break;
	}
}

/* Write data to the FIFO for an endpoint. This function is for endpoints (such
   as EP0) that don't use DMA. Note that the endpoint must be selected in the
   protocol engine prior to this call. */
static void udc_write_hwep(struct lpc32xx_udc *udc, u32 hwep,
				u32 *data, u32 bytes) {
	u32 hwwep = ((hwep & 0x1E) << 1) | CTRL_WR_EN;

	if ((bytes > 0) && (data == NULL)) {
		return;
	}

	/* Setup write of endpoint */
	__raw_writel(hwwep, USBD_CTRL(udc->udp_baseaddr));
	__raw_writel(hwwep, USBD_CTRL(udc->udp_baseaddr));
	while (__raw_readl(USBD_CTRL(udc->udp_baseaddr)) != hwwep) {
		__raw_writel(hwwep, USBD_CTRL(udc->udp_baseaddr));
	}

	__raw_writel(bytes, USBD_TXPLEN(udc->udp_baseaddr));

	/* Need at least 1 byte to rgigger TX, may not be needed */
	if (bytes == 0) {
		__raw_writel(0, USBD_TXDATA(udc->udp_baseaddr));
	}
	else {
		udc_stuff_fifo(udc, (u8 *) data, bytes);
	}

	__raw_writel(((hwep & 0x1E) << 1), USBD_CTRL(udc->udp_baseaddr));
	__raw_writel(((hwep & 0x1E) << 1), USBD_CTRL(udc->udp_baseaddr));

	udc_val_buffer_hwep(udc, hwep);
}

/*
 *
 * USB protocol high level support functions
 *
 */
/* Set/reset USB device address */
static void udc_set_address(struct lpc32xx_udc *udc, u32 addr) {
	/* Address will be latched at the end of the status phase, or
	   latched immediately if function is called twice */
	udc_protocol_cmd_data_w(udc, CMD_SET_ADDR,
		DAT_WR_BYTE(DEV_EN | addr));
}

/* USB device reset - resets USB to a default state with just EP0
   enabled */
static void uda_usb_reset(struct lpc32xx_udc *udc) {
	/* Re-init device controller and EP0 */
	udc_enable(udc);
	udc->gadget.speed = USB_SPEED_FULL;
}

/* Send a ZLP on EP0 */
static void udc_ep0_send_zlp(struct lpc32xx_udc *udc) {
	udc_write_hwep(udc, EP_IN, NULL, 0);
}

/* Get current frame number */
static u16 udc_get_current_frame(struct lpc32xx_udc *udc) {
	u16 flo, fhi;

	udc_protocol_cmd_w(udc, CMD_RD_FRAME);
	flo = (u16) udc_protocol_cmd_r(udc, DAT_RD_FRAME);
	fhi = (u16) udc_protocol_cmd_r(udc, DAT_RD_FRAME);

	return (fhi << 8) | flo;
}

/* Set the device as configured - enables all endpoints */
static inline void udc_set_device_configured(struct lpc32xx_udc *udc) {
	udc_protocol_cmd_data_w(udc, CMD_CFG_DEV, DAT_WR_BYTE(CONF_DVICE));
}

/* Set the device as unconfigured - disables all endpoints */
static inline void udc_set_device_unconfigured(struct lpc32xx_udc *udc) {
	udc_protocol_cmd_data_w(udc, CMD_CFG_DEV, DAT_WR_BYTE(0));
}

/* reinit == restore inital software state */
static void udc_reinit(struct lpc32xx_udc *udc)
{
	u32 i;

	INIT_LIST_HEAD(&udc->gadget.ep_list);
	INIT_LIST_HEAD(&udc->gadget.ep0->ep_list);

	for (i = 0; i < NUM_ENDPOINTS; i++) {
		struct lpc32xx_ep *ep = &udc->ep[i];

		if (i != 0) {
			list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);
		}
		ep->desc = NULL;
		ep->ep.maxpacket = ep->maxpacket;
		INIT_LIST_HEAD(&ep->queue);
		ep->req_pending = 0;
	}

	udc->ep0state = WAIT_FOR_SETUP;
}

static void done(struct lpc32xx_ep *ep, struct lpc32xx_request *req, int status)
{
	struct lpc32xx_udc *udc = ep->udc;

	list_del_init(&req->queue);
	if (req->req.status == -EINPROGRESS) {
		req->req.status = status;
	}
	else {
		status = req->req.status;
	}

#if defined(UDC_ENABLE_DMA)
	if (ep->uses_dma) {
		enum dma_data_direction direction;

		if (ep->is_in)
			direction = DMA_TO_DEVICE;
		else
			direction = DMA_FROM_DEVICE;

		if (req->mapped) {
			dma_unmap_single(ep->udc->gadget.dev.parent,
				req->req.dma, req->req.length, direction);
			req->req.dma = 0;
			req->mapped = 0;
		}
		else {
			dma_sync_single_for_cpu(ep->udc->gadget.dev.parent,
				req->req.dma, req->req.length, direction);
		}

		/* Free DDs */
		udc_dd_free(udc, req->dd_desc_ptr);
	}
#endif

	if (status && status != -ESHUTDOWN) {
		ep_dbg(ep, "%s done %p, status %d\n", ep->ep.name, req, status);
	}

	spin_unlock(&udc->lock);
	req->req.complete(&ep->ep, &req->req);
	spin_lock(&udc->lock);
}


static void nuke(struct lpc32xx_ep *ep, int status)
{
	struct lpc32xx_request *req;

	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct lpc32xx_request, queue);
		done(ep, req, status);
	}

	if (ep->desc) {
		if (status == -ESHUTDOWN) {
			uda_disable_hwepint(ep->udc, ep->hwep_num);
			udc_disable_hwep(ep->udc, ep->hwep_num);
		}
	}
}

static void stop_activity(struct lpc32xx_udc *udc)
{
	struct usb_gadget_driver *driver = udc->driver;
	int i;

	if (udc->gadget.speed == USB_SPEED_UNKNOWN)
		driver = NULL;

	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->suspended = 0;

	for (i = 0; i < NUM_ENDPOINTS; i++) {
		struct lpc32xx_ep *ep = &udc->ep[i];
		nuke(ep, -ESHUTDOWN);
	}
	if (driver)
		driver->disconnect(&udc->gadget);

	udc_disable(udc);
	udc_reinit(udc);
}

/*
 * Activate or kill host pullup
 */
static void pullup(struct lpc32xx_udc *udc, int is_on)
{
	if (!udc->enabled || !udc->vbus)
		is_on = 0;

	if (is_on) {
		udc_clk_set(udc, 1);
		isp1301_pullup_enable(1);
	} else {
		stop_activity(udc);
		isp1301_pullup_enable(0);
		udc_clk_set(udc, 0);
	}
}

static int lpc32xx_ep_disable (struct usb_ep * _ep)
{
	struct lpc32xx_ep *ep = container_of(_ep, struct lpc32xx_ep, ep);
	struct lpc32xx_udc *udc = ep->udc;
	unsigned long	flags;

	if ((ep->hwep_num_base == 0) || (ep->hwep_num == 0)) {
		return -EINVAL;
	}

	local_irq_save(flags);

	nuke(ep, -ESHUTDOWN);

	/* restore the endpoint's pristine config */
	ep->desc = NULL;

	/* Clear all DMA statuses for this EP */
	udc_ep_dma_disable(udc, ep->hwep_num);
	__raw_writel((1 << ep->hwep_num), USBD_EOTINTCLR(udc->udp_baseaddr));
	__raw_writel((1 << ep->hwep_num), USBD_NDDRTINTCLR(udc->udp_baseaddr));
	__raw_writel((1 << ep->hwep_num), USBD_SYSERRTINTCLR(udc->udp_baseaddr));
	__raw_writel((1 << ep->hwep_num), USBD_DMARCLR(udc->udp_baseaddr));

	/* Remove the DD pointer in the UDCA */
	udc->udca_v_base[ep->hwep_num] = 0;
	ep->uses_dma = 0;

	/* Disable and reset endpoint and interrupt */
	uda_clear_hwepint(udc, ep->hwep_num);
	udc_unrealize_hwep(udc, ep->hwep_num);

	ep->hwep_num = 0;

	local_irq_restore(flags);

	return 0;
}

static int lpc32xx_ep_enable(struct usb_ep *_ep,
				const struct usb_endpoint_descriptor *desc)
{
	struct lpc32xx_ep *ep = container_of(_ep, struct lpc32xx_ep, ep);
	struct lpc32xx_udc *udc = ep->udc;
	u16 maxpacket;
	u32 tmp;
	unsigned long flags;

	/* Verify EP data */
	if ((!_ep) || (!ep) || (!desc) || (ep->desc) ||
		(desc->bDescriptorType != USB_DT_ENDPOINT) ||
		((maxpacket = le16_to_cpu(desc->wMaxPacketSize)) == 0) ||
		(maxpacket > ep->maxpacket)) {
		dev_dbg(udc->dev, "bad ep or descriptor\n");
		return -EINVAL;
	}

	/* Don't touch EP0 */
	if (ep->hwep_num_base == 0) {
		dev_dbg(udc->dev, "Can't re-enable EP0!!!\n");
		return -EINVAL;
	}

	/* Is driver ready? */
	if ((!udc->driver) || (udc->gadget.speed == USB_SPEED_UNKNOWN)) {
		dev_dbg(udc->dev, "bogus device state\n");
		return -ESHUTDOWN;
	}

	tmp = desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
	switch (tmp) {
	case USB_ENDPOINT_XFER_CONTROL:
		return -EINVAL;

	case USB_ENDPOINT_XFER_INT:
		if (maxpacket > ep->maxpacket) {
			dev_dbg(udc->dev, "Bad INT endpoint maxpacket %d\n", maxpacket);
			return -EINVAL;
		}
		break;

	case USB_ENDPOINT_XFER_BULK:
		switch (maxpacket) {
		case 8:
		case 16:
		case 32:
		case 64:
			break;

		default:
			dev_dbg(udc->dev, "Bad BULK endpoint maxpacket %d\n", maxpacket);
			return -EINVAL;
		}
		break;

	case USB_ENDPOINT_XFER_ISOC:
		break;
	}

	local_irq_save(flags);

	/* Initialize endpoint to match the selected descriptor */
	ep->is_in = (desc->bEndpointAddress & USB_DIR_IN) != 0;
	ep->desc = desc;
	ep->ep.maxpacket = maxpacket;

	/* Map hardware endpoint from base and direction */
	if (ep->is_in) {
		/* IN endpoints are offset 1 from the OUT endpoint */
		ep->hwep_num = ep->hwep_num_base + EP_IN;
	}
	else {
		ep->hwep_num = ep->hwep_num_base;
	}

	ep_dbg(ep, "EP enabled: %s, HW:%d, MP:%d IN:%d\n", ep->ep.name, ep->hwep_num,
		maxpacket, (ep->is_in == 1));

	/* Realize the endpoint, interrupt is enabled later when
	   buffers are queued, IN EPs will NAK until buffers are ready */
	udc_realize_hwep(udc, ep->hwep_num, ep->ep.maxpacket);
	udc_clr_buffer_hwep(udc, ep->hwep_num);
	uda_disable_hwepint(udc, ep->hwep_num);
	udc_clrstall_hwep(udc, ep->hwep_num);

	/* Clear all DMA statuses for this EP */
	udc_ep_dma_disable(udc, ep->hwep_num);
	__raw_writel((1 << ep->hwep_num), USBD_EOTINTCLR(udc->udp_baseaddr));
	__raw_writel((1 << ep->hwep_num), USBD_NDDRTINTCLR(udc->udp_baseaddr));
	__raw_writel((1 << ep->hwep_num), USBD_SYSERRTINTCLR(udc->udp_baseaddr));
	__raw_writel((1 << ep->hwep_num), USBD_DMARCLR(udc->udp_baseaddr));

#if defined(UDC_ENABLE_DMA)
	ep->uses_dma = 1;
#endif

	local_irq_restore(flags);

	return 0;
}

/* Allocate a USB request list */
static struct usb_request *lpc32xx_ep_alloc_request(
			struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct lpc32xx_request *req;

	req = kzalloc(sizeof (struct lpc32xx_request), gfp_flags);
	if (!req)
		return NULL;

	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

/* De-allocate a USB request list */
static void lpc32xx_ep_free_request(struct usb_ep *_ep,
				struct usb_request *_req)
{
	struct lpc32xx_request *req;

	req = container_of(_req, struct lpc32xx_request, req);
	BUG_ON(!list_empty(&req->queue));
	kfree(req);
}

static int lpc32xx_ep_queue(struct usb_ep *_ep,
			struct usb_request *_req, gfp_t gfp_flags)
{
	struct lpc32xx_request *req;
	struct lpc32xx_ep *ep;
	struct lpc32xx_udc *udc;
	unsigned long flags;
	int status = 0;

	req = container_of(_req, struct lpc32xx_request, req);
	ep = container_of(_ep, struct lpc32xx_ep, ep);

	if (!_req || !_req->complete || !_req->buf || !list_empty(&req->queue)) {
		return -EINVAL;
	}

	if (!_ep || (!ep->desc && ep->hwep_num_base != 0)) {
		dev_dbg(udc->dev, "invalid ep\n");
		return -EINVAL;
	}

	udc = ep->udc;

	if ((!udc) || (!udc->driver) || (udc->gadget.speed == USB_SPEED_UNKNOWN)) {
		dev_dbg(udc->dev, "invalid device\n");
		return -EINVAL;
	}

#if defined(UDC_ENABLE_DMA)
	if (ep->uses_dma) {
		enum dma_data_direction direction;
		struct lpc32xx_usbd_dd_gad *dd;

		/* Map DMA pointer */
		if (ep->is_in)
			direction = DMA_TO_DEVICE;
		else
			direction = DMA_FROM_DEVICE;

		if (req->req.dma == 0) {
			req->req.dma = dma_map_single(
				ep->udc->gadget.dev.parent,
				req->req.buf, req->req.length, direction);
			req->mapped = 1;
		}
		else {
			dma_sync_single_for_device(
				ep->udc->gadget.dev.parent,
				req->req.dma, req->req.length, direction);
			req->mapped = 0;
		}

		/* For the request, build a list of DDs */
		dd = udc_dd_alloc(udc);
		if (!dd) {
			/* Error allocating DD */
			return -ENOMEM;
		}
		req->dd_desc_ptr = dd;

		/* Setup the DMA descriptor */
		dd->dd_next_phy = dd->dd_next_v = NULL;
		dd->dd_buffer_addr = (u32) req->req.dma;
		dd->dd_status = 0;

		/* Special handling for ISO EPs */
		if (ep->eptype == EP_ISO_TYPE) {
			dd->dd_setup = DD_SETUP_ISO_EP |
				DD_SETUP_PACKETLEN(ep->ep.maxpacket) |
				DD_SETUP_DMALENBYTES(req->req.length);
			dd->dd_iso_ps_mem_addr = (u32 *) ((u32) dd->this_dma + 20);
			dd->iso_status[0] = req->req.length;
		}
		else {
			dd->dd_setup = DD_SETUP_PACKETLEN(ep->ep.maxpacket) |
				DD_SETUP_DMALENBYTES(req->req.length);
		}
	}
#endif

	ep_dbg(ep, "%s queue req %p len %d buf %p (in=%d) z=%d\n", _ep->name, _req, _req->length,
	      _req->buf, ep->is_in, _req->zero);

	spin_lock_irqsave(&udc->lock, flags);

	_req->status = -EINPROGRESS;
	_req->actual = 0;
	req->send_zlp = _req->zero;

	/* Kickstart empty queues */
	if (list_empty(&ep->queue)) {
		list_add_tail(&req->queue, &ep->queue);

		if (ep->hwep_num_base == 0) {
			/* Handle expected data direction */
			if (ep->is_in) {
				/* IN packet to host */
				udc->ep0state = DATA_IN;
				status = udc_ep0_in_req(udc);
			}
			else {
				/* OUT packet from host */
				udc->ep0state = DATA_OUT;
				status = udc_ep0_out_req(udc);
			}
		}
		else if (ep->is_in) {
			/* IN packet to host and kick off transfer */
			if (!ep->req_pending) {
#if defined(UDC_ENABLE_DMA)
				udc_ep_in_req_dma(udc, ep);
#else
				uda_enable_hwepint(udc, ep->hwep_num);
				udc_ep_in_req(udc, ep);
#endif
			}
		}
		else {
			/* OUT packet from host and kick off list */
			if (!ep->req_pending) {
#if defined(UDC_ENABLE_DMA)
				udc_ep_out_req_dma(udc, ep);
#else
				uda_enable_hwepint(udc, ep->hwep_num);
				udc_ep_out_req(udc, ep);
#endif
			}
		}
	}
	else
		list_add_tail(&req->queue, &ep->queue);

	spin_unlock_irqrestore(&udc->lock, flags);

	return (status < 0) ? status : 0;
}

static int lpc32xx_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct lpc32xx_ep	*ep;
	struct lpc32xx_request	*req;
	unsigned long flags;

	ep = container_of(_ep, struct lpc32xx_ep, ep);
	if (!_ep || ep->hwep_num_base == 0)
		return -EINVAL;

	spin_lock_irqsave(&ep->udc->lock, flags);

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry (req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		spin_unlock_irqrestore(&ep->udc->lock, flags);
		return -EINVAL;
	}

	done(ep, req, -ECONNRESET);
	spin_unlock_irqrestore(&ep->udc->lock, flags);

	return 0;
}

static int lpc32xx_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct lpc32xx_ep *ep = container_of(_ep, struct lpc32xx_ep, ep);
	struct lpc32xx_udc *udc = ep->udc;
	unsigned long flags;

	if ((!ep) || (ep->desc == NULL) || (ep->hwep_num <= 1))
		return -EINVAL;

	/* Don't halt an IN EP */
	if (ep->is_in)
		return -EAGAIN;

	spin_lock_irqsave(&udc->lock, flags);

	if (value == 1) {
		/* stall */
		udc_protocol_cmd_data_w(udc, CMD_SET_EP_STAT(ep->hwep_num),
			DAT_WR_BYTE(EP_STAT_ST));
	}
	else {
		/* End stall */
		udc_protocol_cmd_data_w(udc, CMD_SET_EP_STAT(ep->hwep_num),
			DAT_WR_BYTE(0));
	}

	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static const struct usb_ep_ops lpc32xx_ep_ops = {
	.enable		= lpc32xx_ep_enable,
	.disable	= lpc32xx_ep_disable,
	.alloc_request	= lpc32xx_ep_alloc_request,
	.free_request	= lpc32xx_ep_free_request,
	.queue		= lpc32xx_ep_queue,
	.dequeue	= lpc32xx_ep_dequeue,
	.set_halt	= lpc32xx_ep_set_halt,
};

#if defined(UDC_ENABLE_DMA)
/* Setup up a IN request for DMA transfer - this consists of determining the
   list of DMA addresses for the transfer, allocating DMA Descriptors,
   installing the DD into the UDCA, and then enabling the DMA for that EP */
static int udc_ep_in_req_dma(struct lpc32xx_udc *udc, struct lpc32xx_ep *ep)
{
	struct lpc32xx_request *req;
	u32 hwep = ep->hwep_num;

	ep->req_pending = 1;

	/* There will always be a request waiting here */
	req = list_entry(ep->queue.next, struct lpc32xx_request, queue);

	/* Place the DD Descriptor into the UDCA */
	udc->udca_v_base[hwep] = (u32) req->dd_desc_ptr->this_dma;

	/* Enable DMA and interrupt for the HW EP */
	udc_ep_dma_enable(udc, hwep);

	return 0;
}

/* Setup up a OUT request for DMA transfer - this consists of determining the
   list of DMA addresses for the transfer, allocating DMA Descriptors,
   installing the DD into the UDCA, and then enabling the DMA for that EP */
static int udc_ep_out_req_dma(struct lpc32xx_udc *udc, struct lpc32xx_ep *ep)
{
	struct lpc32xx_request *req;
	u32 hwep = ep->hwep_num;

	ep->req_pending = 1;

	/* There will always be a request waiting here */
	req = list_entry(ep->queue.next, struct lpc32xx_request, queue);

	/* Place the DD Descriptor into the UDCA */
	udc->udca_v_base[hwep] = (u32) req->dd_desc_ptr->this_dma;

	/* Enable DMA and interrupt for the HW EP */
	udc_ep_dma_enable(udc, hwep);

	return 0;
}

/* This function is only called for IN ZLP transfer completions */
void udc_handle_eps(struct lpc32xx_udc *udc, u32 epints) {
	int hwep = -1, i = 0;
	u32 epstatus;
	struct lpc32xx_ep *ep;
	struct lpc32xx_request *req;
	const int ineps[] = {2, 5, 8, 11, 15, -1};

	/* Find the IN EP that generated the interrupt */
	while (ineps[i] != 0) {
		if (epints & (1 << ineps[i]))
			hwep = ineps[i];
		i++;
	}

	uda_disable_hwepint(udc, hwep);

	if (hwep <= 0)
		return;

	epstatus = udc_clearep_getsts(udc, hwep);

	ep = &udc->ep[hwep];
	req = list_entry(ep->queue.next, struct lpc32xx_request, queue);
	done(ep, req, 0);

	/* Start another request if ready */
	if (!list_entry(ep->queue.next, struct lpc32xx_request, queue)) {
		if (ep->is_in)
			udc_ep_in_req_dma(udc, ep);
		else
			udc_ep_out_req_dma(udc, ep);
	}
	else
		ep->req_pending = 0;
}

/* Send a ZLP on a non-0 IN EP */
void udc_send_in_zlp(struct lpc32xx_udc *udc, struct lpc32xx_ep *ep,
			struct lpc32xx_usbd_dd_gad *dd) {
	/* Set up EP interrupt status */
	uda_enable_hwepint(udc, ep->hwep_num);
	udc_clearep_getsts(udc, ep->hwep_num);

	/* Send ZLP */
	udc_write_hwep(udc, ep->hwep_num, NULL, 0);
}

/* DMA end of transfer completion */
static void udc_handle_dma_ep(struct lpc32xx_udc *udc, struct lpc32xx_ep *ep) {
	u32 status;
	struct lpc32xx_request *req;
	struct lpc32xx_usbd_dd_gad *dd;

#ifdef CONFIG_USB_GADGET_DEBUG_FILES
	ep->totalints++;
#endif

	req = list_entry(ep->queue.next, struct lpc32xx_request, queue);
	if (!req) {
		ep_err(ep, "DMA interrupt on no req!\n");
		return;
	}
	dd = req->dd_desc_ptr;

	/* Wait for end of descriptor to retire */
	while (!(dd->dd_status & DD_STATUS_DD_RETIRED));

	/* Disable DMA */
	udc_ep_dma_disable(udc, ep->hwep_num);
	__raw_writel((1 << ep->hwep_num), USBD_EOTINTCLR(udc->udp_baseaddr));
	__raw_writel((1 << ep->hwep_num), USBD_NDDRTINTCLR(udc->udp_baseaddr));

	/* System error? */
	if (__raw_readl(USBD_SYSERRTINTST(udc->udp_baseaddr)) & (1 << ep->hwep_num)) {
		__raw_writel((1 << ep->hwep_num), USBD_SYSERRTINTCLR(udc->udp_baseaddr));
		ep_err(ep, "AHB critical error!\n");
		ep->req_pending = 0;

		/* The error could of occurred on a packet of a multipacket transfer,
		   so recovering the transfer is not possible. Close the request with
		   an error */
		done(ep, req, -ECONNABORTED);

		return;
	}

	/* Handle the current DD's status */
	status = dd->dd_status;
	switch (status & DD_STATUS_STS_MASK) {
	case DD_STATUS_STS_NS:
		/* DD not serviced? This shouldn't happen! */
		ep->req_pending = 0;
		ep_err(ep, "DMA critical EP error: DD not serviced (0x%x)!\n", status);
		done(ep, req, -ECONNABORTED);
		return;

	case DD_STATUS_STS_BS:
		/* Interrupt only fires on EOT - This shouldn't happen! */
		ep->req_pending = 0;
		ep_err(ep, "DMA critical EP error: EOT prior to service completion (0x%x)!\n", status);
		done(ep, req, -ECONNABORTED);
		return;

	case DD_STATUS_STS_NC:
	case DD_STATUS_STS_DUR: /* Really just a short packet, not an underrun */
		/* This is a good status and what we expect */
		break;

	default:
		/* Data overrun, system error, or unknown */
		ep->req_pending = 0;
		ep_err(ep, "DMA critical EP error: System error (0x%x)!\n", status);
		done(ep, req, -ECONNABORTED);
		return;
	}

	/* Save transferred data size */
	req->req.actual += DD_STATUS_CURDMACNT(status);

	/* Work around the wierd underrun packet issue */
	if ((!ep->is_in) && (!(req->req.actual % ep->maxpacket)) &&
		((req->req.length - req->req.actual) > 0)) {
		ep_dbg(ep, "Short packet in unexpected situation!\n");

		// WTF is this? An underrun packet on OUT with no ZLP! Hardware issue? */
		dd->dd_next_phy = dd->dd_next_v = NULL;
		dd->dd_buffer_addr = (u32) req->req.dma;
		dd->dd_buffer_addr += req->req.actual;
		dd->dd_status = 0;

		/* Special handling for ISO EPs */
		dd->dd_setup = DD_SETUP_PACKETLEN(ep->ep.maxpacket) |
			DD_SETUP_DMALENBYTES(req->req.length - req->req.actual);

		/* Do the remainder of the req */
		udc_ep_out_req_dma(udc, ep);

		return;
	}

	/* ISO endpoints are handled differently */
	if (ep->eptype == EP_ISO_TYPE) {
		if (!ep->is_in)
			req->req.actual = dd->iso_status[0] & 0xFFFF;
	}

	/* For an Bulk IN EP, the DMA engine will only send data as specified in the
	   descriptor. If the total transfer size is a multiple of the max packet
	   size, then the transfer was completed, but no ZLP was sent. The ZLP needs
	   to be sent using the FIFO mechanism to terminate this transfer */
	if (req->send_zlp) {
		udc_send_in_zlp(udc, ep, dd);

		/* Just exit */
		return;
	}

	/* Transfer request is complete */
	done(ep, req, 0);

	udc_clearep_getsts(udc, ep->hwep_num);

	/* Start another request if ready */
	if (!list_entry(ep->queue.next, struct lpc32xx_request, queue)) {
		if (ep->is_in)
			udc_ep_in_req_dma(udc, ep);
		else
			udc_ep_out_req_dma(udc, ep);
	}
	else
		ep->req_pending = 0;
}

#else
/* This function was called if a new request is ready to be placed into the SEND FIFO
   for transfer to the host, or when a previous transfer to the host has completed. */
static int udc_ep_in_req(struct lpc32xx_udc *udc, struct lpc32xx_ep *ep) {
	struct lpc32xx_request *req;
	u32 ts, epstatus, bufst;
	int is_last;

	/* Select and clear EP interrupt */
	epstatus = udc_clearep_getsts(udc, ep->hwep_num);

	if (epstatus & EP_SEL_ST) {
		/* EP is stalled */
		ep->totalstalls++;
		return 0;
	}

	if (epstatus & EP_SEL_EPN) {
		/* NAK'd on other side */
		ep->totalnaks++;
	}

	bufst = (epstatus & EP_SEL_F);

	/* Are any requests available? */
	if (list_empty(&ep->queue)) {
		if (!bufst) {
			/* No reqs and the hardware is idle, disable IRQ */
			uda_disable_hwepint(udc, ep->hwep_num);
		}

		return 0;
	}

	/* If both buffers are currently full, just exit for now */
	if (bufst)
		return 0;

	/* A buffer is available in the hardware, so we can fill  it */
	req = list_entry(ep->queue.next, struct lpc32xx_request, queue);

	/* Limit packet size to the size of the EP */
	ts = req->req.length - req->req.actual;
	if (ts > ep->ep.maxpacket)
		ts = ep->ep.maxpacket;

	/* Write data to the EP0 FIFO and start transfer */
	ep_dbg(ep, "SEND %s 0x%x(%d)\n", ep->ep.name, (u32)(req->req.buf + req->req.actual), ts);
	udc_write_hwep(udc, ep->hwep_num, (req->req.buf + req->req.actual), ts);

	/* Increment data pointer */
	req->req.actual += ts;

	if (ts < ep->ep.maxpacket)
		is_last = 1;
	else if ((req->req.actual != req->req.length) || (req->send_zlp)) {
		req->send_zlp = 0;
		is_last = 0;
	}
	else
		is_last = 1;

	if (is_last) {
		/* Transfer request is complete */
		done(ep, req, 0);
		return 1;
	}

	/* Stay in data transfer state */
	return 0;
}

static int udc_ep_out_req(struct lpc32xx_udc *udc, struct lpc32xx_ep *ep) {
	struct lpc32xx_request *req;
	u32 tr, bufferspace, epstatus;

	/* Clear EP interrupt */
	epstatus = udc_clearep_getsts(udc, ep->hwep_num);

	if (epstatus & EP_SEL_ST) {
		/* EP is stalled */
		ep->totalstalls++;
	}

	if (epstatus & EP_SEL_EPN) {
		/* Sent NAK */
		ep->totalnaks++;
	}

	/* Are any requests available? */
	if (list_empty(&ep->queue)) {
		uda_disable_hwepint(udc, ep->hwep_num);

		return 0;
	}

	if (epstatus & EP_SEL_F) {
		req = list_entry(ep->queue.next, struct lpc32xx_request, queue);
		if (req->req.length == 0) {
			ep_dbg(ep, "%s OUT zero buffer length!\n", ep->ep.name);
			return 0;
		}

		/* Limit transfer size to size of endpoint */
		bufferspace = req->req.length - req->req.actual;
		if (bufferspace > ep->ep.maxpacket)
			bufferspace = ep->ep.maxpacket;

		/* Copy data to buffer from FIFO */
		prefetchw(req->req.buf + req->req.actual);
		tr = udc_read_hwep(udc, ep->hwep_num,
			(req->req.buf + req->req.actual), bufferspace);

		ep_dbg(ep, "RECV %s 0x%x(%d) %d %d\n", ep->ep.name, (u32)(req->req.buf + req->req.actual), tr,
			req->req.actual, req->req.length);
		req->req.actual += tr;

		if ((tr < ep->ep.maxpacket) || (req->req.actual == req->req.length)) {
			/* This is the last packet */
			done(ep, req, 0);

			return 1;
		}
	}

	return 0;
}

static void udc_handle_ep(struct lpc32xx_udc *udc, struct lpc32xx_ep *ep) {
	ep->totalints++;

	if (!ep->desc) {
		uda_disable_hwepint(udc, ep->hwep_num);
		return;
	}

	/* Nice and easy */
	if (ep->is_in) {
		/* Handle IN request */
		udc_ep_in_req(udc, ep);
	}
	else {
		/* Handle OUT request */
		udc_ep_out_req(udc, ep);
	}
}
#endif

/*
 * * Endpoint 0 functions
 *
 */
static void udc_handle_dev(struct lpc32xx_udc *udc) {
	u32 tmp;

	udc_protocol_cmd_w(udc, CMD_GET_DEV_STAT);
	tmp = udc_protocol_cmd_r(udc, DAT_GET_DEV_STAT);

	if (tmp & DEV_RST) {
		uda_usb_reset(udc);
	}
	else if (tmp & DEV_CON_CH) {
		uda_power_event(udc, (tmp & DEV_CON));
	}
	else if (tmp & DEV_SUS_CH) {
		if (tmp & DEV_SUS) {
			if (udc->vbus == 0) {
				stop_activity(udc);
			} else if ((udc->gadget.speed !=
				USB_SPEED_UNKNOWN) && udc->driver &&
				udc->driver->suspend) {
				udc->driver->suspend(&udc->gadget);
				uda_resm_susp_event(udc, 1);
			}
		}
		else {
			if ((udc->gadget.speed != USB_SPEED_UNKNOWN) &&
				udc->driver && udc->driver->resume &&
				udc->vbus) {
				udc->driver->resume(&udc->gadget);
				uda_resm_susp_event(udc, 0);
			}
		}
	}
}

/* IN endpoint 0 transfer */
static int udc_ep0_in_req(struct lpc32xx_udc *udc) {
	struct lpc32xx_request *req;
	struct lpc32xx_ep *ep0 = &udc->ep [0];
	u32 tsend, ts = 0;

	if (list_empty(&ep0->queue))
	{
		/* Nothing to send */
		return 0;
	}
	else {
		req = list_entry(ep0->queue.next,
			struct lpc32xx_request, queue);
	}

	tsend = ts = req->req.length - req->req.actual;
	if (ts == 0) {
		/* Send a ZLP */
		udc_ep0_send_zlp(udc);
		done(ep0, req, 0);
		return 1;
	}
	else if (ts > ep0->ep.maxpacket) {
		/* Just send what we can */
		ts = ep0->ep.maxpacket;
	}

	/* Write data to the EP0 FIFO and start transfer */
	udc_write_hwep(udc, EP_IN, (req->req.buf + req->req.actual), ts);

	/* Increment data pointer */
	req->req.actual += ts;

	if (tsend >= ep0->ep.maxpacket) {
		/* Stay in data transfer state */
		return 0;
	}

	/* Transfer request is complete */
	udc->ep0state = WAIT_FOR_SETUP;
	done(ep0, req, 0);
	return 1;
}

/* OUT endpoint 0 transfer */
static int udc_ep0_out_req(struct lpc32xx_udc *udc) {
	struct lpc32xx_request *req;
	struct lpc32xx_ep *ep0 = &udc->ep[0];
	u32 tr, bufferspace;

	if (list_empty(&ep0->queue)) {
		return 0;
	}
	else {
		req = list_entry(ep0->queue.next, struct lpc32xx_request, queue);
	}

	if (req) {
		if (req->req.length == 0) {
			/* Just dequeue request */
			done(ep0, req, 0);
			udc->ep0state = WAIT_FOR_SETUP;
			return 1;
		}

		/* Get data from FIFO */
		bufferspace = req->req.length - req->req.actual;
		if (bufferspace > ep0->ep.maxpacket) {
			bufferspace = ep0->ep.maxpacket;
		}

		/* Copy data to buffer */
		prefetchw(req->req.buf + req->req.actual);
		tr = udc_read_hwep(udc, EP_OUT,
			(req->req.buf + req->req.actual), bufferspace);
		req->req.actual += bufferspace;

		if (tr < ep0->ep.maxpacket) {
			/* This is the last packet */
			done(ep0, req, 0);
			udc->ep0state = WAIT_FOR_SETUP;
			return 1;
		}
	}

	return 0;
}

static int udc_get_status(struct lpc32xx_udc *udc, u16 reqtype, u16 wIndex) {
	struct lpc32xx_ep *ep;
	u32 ep0buff = 0, tmp;

	switch (reqtype) {
	case USB_RECIP_INTERFACE:
		/* Not supported */
		break;

	case USB_RECIP_DEVICE:
		ep0buff = (udc->selfpowered << USB_DEVICE_SELF_POWERED);
		if (udc->dev_status & (1 << USB_DEVICE_REMOTE_WAKEUP)) {
			ep0buff |= (1 << USB_DEVICE_REMOTE_WAKEUP);
		}
		break;

	case USB_RECIP_ENDPOINT:
		tmp = wIndex & USB_ENDPOINT_NUMBER_MASK;
		ep = &udc->ep[tmp];
		if ((tmp == 0) || (tmp >= NUM_ENDPOINTS) || (tmp && !ep->desc)) {
			return -EOPNOTSUPP;
		}

		if (wIndex & USB_DIR_IN) {
			if (!ep->is_in) {
				/* Somethings wrong */
				return -EOPNOTSUPP;
			}
		} else if (ep->is_in)
			/* Not an IN endpoint */
			return -EOPNOTSUPP;

		/* Get status of the endpoint */
		udc_protocol_cmd_w(udc, CMD_SEL_EP(ep->hwep_num));
		tmp = udc_protocol_cmd_r(udc, DAT_SEL_EP(ep->hwep_num));

		if (tmp & EP_SEL_ST) {
			ep0buff = (1 << USB_ENDPOINT_HALT);
		}
		else {
			ep0buff = 0;
		}
		break;

	default:
		break;
	}

	/* Return data */
	udc_write_hwep(udc, EP_IN, &ep0buff, 2);

	return 0;
}

static void udc_handle_ep0_setup(struct lpc32xx_udc *udc) {
	struct lpc32xx_ep *ep, *ep0 = &udc->ep[0];
	struct usb_ctrlrequest ctrlpkt;
	int i, bytes;
	u16 wIndex, wValue, wLength, reqtype, req, tmp;

	/* Nuke previous transfers */
	nuke(ep0, -EPROTO);

	/* Get setup packet */
	bytes = udc_read_hwep(udc, EP_OUT, (u32 *) &ctrlpkt, 8);
	if (bytes != 8) {
		ep_dbg(ep0, "Incorrectly sized setup packet (s/b 8, is %d!\n", bytes);
		return;
	}

	/* Native endianness */
	wIndex = le16_to_cpu(ctrlpkt.wIndex);
	wValue = le16_to_cpu(ctrlpkt.wValue);
	wLength = le16_to_cpu(ctrlpkt.wLength);
	reqtype = le16_to_cpu(ctrlpkt.bRequestType);

	/* Set direction of EP0 */
	if (likely(reqtype & USB_DIR_IN)) {
		ep0->is_in = 1;
	} else {
		ep0->is_in = 0;
	}

	/* Handle SETUP packet */
	req = le16_to_cpu(ctrlpkt.bRequest);
	switch (req) {
	case USB_REQ_CLEAR_FEATURE:
	case USB_REQ_SET_FEATURE:
		switch (reqtype) {
		case (USB_TYPE_STANDARD | USB_RECIP_DEVICE):
			if (wValue != USB_DEVICE_REMOTE_WAKEUP) {
				/* Nothing else handled */
				goto stall;
			}

			/* Tell board about event */
			if (req == USB_REQ_CLEAR_FEATURE)
				udc->dev_status &= ~(1 << USB_DEVICE_REMOTE_WAKEUP);
			else
				udc->dev_status |= (1 << USB_DEVICE_REMOTE_WAKEUP);
			uda_remwkp_cgh(udc);
			goto zlp_send;

		case (USB_TYPE_STANDARD | USB_RECIP_ENDPOINT):
			tmp = wIndex & USB_ENDPOINT_NUMBER_MASK;
			if ((wValue != USB_ENDPOINT_HALT) || (tmp >= NUM_ENDPOINTS))
				break;

			/* Find hardware endpoint from logical endpoint */
			ep = &udc->ep[tmp];
			tmp = ep->hwep_num;
			if (tmp == 0)
				break;

			if (req == USB_REQ_CLEAR_FEATURE)
				udc_stall_hwep(udc, tmp);
			else
				udc_clrstall_hwep(udc, tmp);

			goto zlp_send;

		default:
			break;
		}


	case USB_REQ_SET_ADDRESS:
		if (reqtype == (USB_TYPE_STANDARD | USB_RECIP_DEVICE)) {
			udc_set_address(udc, wValue);
			goto zlp_send;
		}
		break;

	case USB_REQ_GET_STATUS:
		udc_get_status(udc, reqtype, wIndex);
		return;

	default:
		/* Let GadgetFs handle the descriptor instead */
		break;
	}

	if (likely(udc->driver)) {
		/* device-2-host (IN) or no data setup command, process immediately */
		spin_unlock(&udc->lock);
		i = udc->driver->setup(&udc->gadget, &ctrlpkt);
		spin_lock(&udc->lock);
		if (req == USB_REQ_SET_CONFIGURATION) {
			/* Configuration is set after endpoints are realized */
			if (wValue) {
				/* Set configuration */
				udc_set_device_configured(udc);

				/* NAK EP interrupts do not need to be enabled for this
				   driver, but if you really want them for statistic
				   purposes, uncomment the following lines */
				udc_protocol_cmd_data_w(udc, CMD_SET_MODE, DAT_WR_BYTE(AP_CLK |
#if defined(UDC_ENABLE_DMA)
					INAK_BI | INAK_II));
#else
					INAK_BO | INAK_BI | INAK_IO | INAK_II));
#endif
			}
			else {
				/* Clear configuration */
				udc_set_device_unconfigured(udc);

				/* Disable NAK interrupts */
				udc_protocol_cmd_data_w(udc, CMD_SET_MODE, DAT_WR_BYTE(AP_CLK));
			}
		}

		if (i < 0) {
			/* setup processing failed, force stall */
			dev_err(udc->dev, "req %02x.%02x protocol STALL; stat %d\n",
				reqtype, req, i);
			udc->ep0state = WAIT_FOR_SETUP;
			goto stall;
		}
	}

	if (!ep0->is_in) {
		/* ZLP IN packet on on data phase */
		udc_ep0_send_zlp(udc);
	}

	return;

stall:
	udc_stall_hwep(udc, EP_IN);
	return;

zlp_send:
	udc_ep0_send_zlp(udc);
	return;
}

/* IN endpoint 0 transfer */
static void udc_handle_ep0_in(struct lpc32xx_udc *udc) {
	struct lpc32xx_ep *ep0 = &udc->ep [0];
	u32 epstatus;

	/* Clear EP interrupt */
	epstatus = udc_clearep_getsts(udc, EP_IN);

#ifdef CONFIG_USB_GADGET_DEBUG_FILES
	ep0->totalints++;
#endif

	/* Stalled? Clear stall and reset buffers */
	if (epstatus & EP_SEL_ST) {
		udc_clrstall_hwep(udc, EP_IN);
		nuke(ep0, -ECONNABORTED);
		udc->ep0state = WAIT_FOR_SETUP;
		return;
	}

	/* Is a buffer available? */
	if (!(epstatus & EP_SEL_F)) {
		/* Handle based on current state */
		if (udc->ep0state == DATA_IN) {
			udc_ep0_in_req(udc);
		}
		else {
			/* Unknown state for EP0 oe end of DATA IN phase */
			nuke(ep0, -ECONNABORTED);
			udc->ep0state = WAIT_FOR_SETUP;
		}
	}
}

/* OUT endpoint 0 transfer */
static void udc_handle_ep0_out(struct lpc32xx_udc *udc) {
	struct lpc32xx_ep *ep0 = &udc->ep[0];
	u32 epstatus;

	/* Clear EP interrupt */
	epstatus = udc_clearep_getsts(udc, EP_OUT);

#ifdef CONFIG_USB_GADGET_DEBUG_FILES
	ep0->totalints++;
#endif

	/* Stalled? */
	if (epstatus & EP_SEL_ST) {
		udc_clrstall_hwep(udc, EP_OUT);
		nuke(ep0, -ECONNABORTED);
		udc->ep0state = WAIT_FOR_SETUP;
		return;
	}

	/* A NAK may occur if a packet coudn't be received yet */
	if (epstatus & EP_SEL_EPN) {
		return;
	}
	/* Setup packet incoming? */
	if (epstatus & EP_SEL_STP) {
		nuke(ep0, 0);
		udc->ep0state = WAIT_FOR_SETUP;
	}

	/* Data available? */
	if (epstatus & EP_SEL_F) {
		/* Handle based on current state */
		switch (udc->ep0state) {
		case WAIT_FOR_SETUP:
			udc_handle_ep0_setup(udc);
			break;

		case DATA_OUT:
			udc_ep0_out_req(udc);
			break;

		default:
			/* Unknown state for EP0 */
			nuke(ep0, -ECONNABORTED);
			udc->ep0state = WAIT_FOR_SETUP;
		}
	}
}

static int lpc32xx_get_frame(struct usb_gadget *gadget)
{
	struct lpc32xx_udc *udc = to_udc(gadget);

	if (!to_udc(gadget)->clocked)
		return -EINVAL;

	return (int) udc_get_current_frame(udc);
}

static int lpc32xx_wakeup(struct usb_gadget *gadget)
{
	return -ENOTSUPP;
}

static int lpc32xx_set_selfpowered(struct usb_gadget *gadget, int is_on)
{
	struct lpc32xx_udc *udc = to_udc(gadget);

	/* Always self-powered */
	udc->selfpowered = (is_on != 0);

	return 0;
	return -ENOTSUPP;
}

/* vbus is here!  turn everything on that's ready */
static int lpc32xx_vbus_session(struct usb_gadget *gadget, int is_active)
{
	struct lpc32xx_udc *udc = to_udc(gadget);

	/* Doesn't need lock */
	if (udc->driver)
		pullup(udc, is_active);
	else
		pullup(udc, 0);

	return 0;
}

static int lpc32xx_pullup(struct usb_gadget *gadget, int is_on)
{
	struct lpc32xx_udc *udc = to_udc(gadget);

	/* Doesn't need lock */
	pullup(udc, is_on);

	return 0;
}

static const struct usb_gadget_ops lpc32xx_udc_ops = {
	.get_frame		= lpc32xx_get_frame,
	.wakeup			= lpc32xx_wakeup,
	.set_selfpowered	= lpc32xx_set_selfpowered,
	.vbus_session		= lpc32xx_vbus_session,
	.pullup			= lpc32xx_pullup,
};

static void nop_release(struct device *dev)
{
	/* nothing to free */
}

static struct lpc32xx_udc controller = {
	.gadget = {
		.ops	= &lpc32xx_udc_ops,
		.ep0	= &controller.ep[0].ep,
		.name	= driver_name,
		.dev	= {
			.bus_id = "gadget",
			.release = nop_release,
		}
	},
	.ep[0] = {
		.ep = {
			.name	= "ep0",
			.ops	= &lpc32xx_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 64,
		.hwep_num_base	= 0,
		.hwep_num	= 0, /* Can be 0 or 1, has special handling */
		.lep		= 0,
		.eptype		= EP_CTL_TYPE,
	},
	.ep[1] = {
		.ep = {
			.name	= "ep1-int",
			.ops	= &lpc32xx_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 64,
		.hwep_num_base	= 2,
		.hwep_num	= 0, /* 2 or 3, will be set later */
		.lep		= 1,
		.eptype		= EP_INT_TYPE,
	},
	.ep[2] = {
		.ep = {
			.name	= "ep2-bulk",
			.ops	= &lpc32xx_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 64,
		.hwep_num_base	= 4,
		.hwep_num	= 0, /* 4 or 5, will be set later */
		.doublebuff	= 1,
		.lep		= 2,
		.eptype		= EP_BLK_TYPE,
	},
	.ep[3] = {
		.ep = {
			.name	= "ep3-iso",
			.ops	= &lpc32xx_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 1023,
		.hwep_num_base	= 6,
		.hwep_num	= 0, /* 6 or 7, will be set later */
		.doublebuff	= 1,
		.lep		= 3,
		.eptype		= EP_ISO_TYPE,
	},
	.ep[4] = {
		.ep = {
			.name	= "ep4-int",
			.ops	= &lpc32xx_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 64,
		.hwep_num_base	= 8,
		.hwep_num	= 0, /* 8 or 9, will be set later */
		.lep		= 4,
		.eptype		= EP_INT_TYPE,
	},
	.ep[5] = {
		.ep = {
			.name	= "ep5-bulk",
			.ops	= &lpc32xx_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 64,
		.hwep_num_base	= 10,
		.hwep_num	= 0, /* 10 or 11, will be set later */
		.doublebuff	= 1,
		.lep		= 5,
		.eptype		= EP_BLK_TYPE,
	},
	.ep[6] = {
		.ep = {
			.name	= "ep6-iso",
			.ops	= &lpc32xx_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 1023,
		.hwep_num_base	= 12,
		.hwep_num	= 0, /* 12 or 13, will be set later */
		.doublebuff	= 1,
		.lep		= 6,
		.eptype		= EP_ISO_TYPE,
	},
	.ep[7] = {
		.ep = {
			.name	= "ep7-int",
			.ops	= &lpc32xx_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 64,
		.hwep_num_base	= 14,
		.hwep_num	= 0,
		.lep		= 7,
		.eptype		= EP_INT_TYPE,
	},
	.ep[8] = {
		.ep = {
			.name	= "ep8-bulk",
			.ops	= &lpc32xx_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 64,
		.hwep_num_base	= 16,
		.hwep_num	= 0,
		.doublebuff	= 1,
		.lep		= 8,
		.eptype		= EP_BLK_TYPE,
	},
	.ep[9] = {
		.ep = {
			.name	= "ep9-iso",
			.ops	= &lpc32xx_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 1023,
		.hwep_num_base	= 18,
		.hwep_num	= 0,
		.doublebuff	= 1,
		.lep		= 9,
		.eptype		= EP_ISO_TYPE,
	},
	.ep[10] = {
		.ep = {
			.name	= "ep10-int",
			.ops	= &lpc32xx_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 64,
		.hwep_num_base	= 20,
		.hwep_num	= 0,
		.lep		= 10,
		.eptype		= EP_INT_TYPE,
	},
	.ep[11] = {
		.ep = {
			.name	= "ep11-bulk",
			.ops	= &lpc32xx_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 64,
		.hwep_num_base	= 22,
		.hwep_num	= 0,
		.doublebuff	= 1,
		.lep		= 11,
		.eptype		= EP_BLK_TYPE,
	},
	.ep[12] = {
		.ep = {
			.name	= "ep12-iso",
			.ops	= &lpc32xx_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 1023,
		.hwep_num_base	= 24,
		.hwep_num	= 0,
		.doublebuff	= 1,
		.lep		= 12,
		.eptype		= EP_ISO_TYPE,
	},
	.ep[13] = {
		.ep = {
			.name	= "ep13-int",
			.ops	= &lpc32xx_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 64,
		.hwep_num_base	= 26,
		.hwep_num	= 0,
		.lep		= 13,
		.eptype		= EP_INT_TYPE,
	},
	.ep[14] = {
		.ep = {
			.name	= "ep14-bulk",
			.ops	= &lpc32xx_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 64,
		.hwep_num_base	= 28,
		.hwep_num	= 0,
		.doublebuff	= 1,
		.lep		= 14,
		.eptype		= EP_BLK_TYPE,
	},
	.ep[15] = {
		.ep = {
			.name	= "ep15-bulk",
			.ops	= &lpc32xx_ep_ops,
		},
		.udc		= &controller,
		.maxpacket	= 1023,
		.hwep_num_base	= 30,
		.hwep_num	= 0,
		.doublebuff	= 1,
		.lep		= 15,
		.eptype		= EP_BLK_TYPE,
	},
};

/* ISO and status interrupts */
static irqreturn_t lpc32xx_usb_lp_irq(int irq, void *_udc) {
	u32 tmp, devstat;
	struct lpc32xx_udc *udc = _udc;

	spin_lock(&udc->lock);

	/* Read the device status register */
	devstat = __raw_readl(USBD_DEVINTST(udc->udp_baseaddr));
	devstat &= ~USBD_EP_FAST;
	__raw_writel(devstat, USBD_DEVINTCLR(udc->udp_baseaddr));
	devstat = devstat & udc->enabled_devints;

	/* Device specific handling needed? */
	if (devstat & USBD_DEV_STAT) {
		udc_handle_dev(udc);
	}

	/* Start of frame? */
	if (devstat & FRAME_INT) {
		/* The frame interrupt isn't really needed for ISO support,
		   as the driver will queue the necessary packets */
		dev_dbg(udc->dev, "Device frame interrupt not supported\n");
	}

	/* Error? */
	if (devstat & ERR_INT) {
		/* All types of errors, from cable removal during transfer to
		   misc protocol and bit errors. These are mostly for just info,
		   as the USB hardware will work around these */
		udc_protocol_cmd_w(udc, CMD_RD_ERR_STAT);
		tmp = udc_protocol_cmd_r(udc, DAT_RD_ERR_STAT);
		dev_err(udc->dev, "Device error (0x%x)!\n", tmp);
	}

	spin_unlock(&udc->lock);
	return IRQ_HANDLED;
}

/* EP interrupts */
static irqreturn_t lpc32xx_usb_hp_irq(int irq, void *_udc)
{
	u32 tmp;
	struct lpc32xx_udc *udc = _udc;

	spin_lock(&udc->lock);

	/* Read the device status register */
	tmp = __raw_readl(USBD_DEVINTST(udc->udp_baseaddr));
	__raw_writel(USBD_EP_FAST, USBD_DEVINTCLR(udc->udp_baseaddr));

	/* Endpoints */
	tmp = __raw_readl(USBD_EPINTST(udc->udp_baseaddr));

	/* Special handling for EP0 */
	if (tmp & (EP_MASK_SEL(0, EP_OUT) | EP_MASK_SEL(0, EP_IN))) {
		/* Handle EP0 IN */
		if (tmp & (EP_MASK_SEL(0, EP_IN)))
			udc_handle_ep0_in(udc);

		/* Handle EP0 OUT */
		if (tmp & (EP_MASK_SEL(0, EP_OUT)))
			udc_handle_ep0_out(udc);
	}

	/* All other EPs */
	if (tmp & ~(EP_MASK_SEL(0, EP_OUT) | EP_MASK_SEL(0, EP_IN))) {
#if defined(UDC_ENABLE_DMA)
		udc_handle_eps(udc, tmp);

#else
		int i;

		/* Handle other EP interrupts */
		for (i = 1; i < NUM_ENDPOINTS; i++) {
			if (tmp & (1 << udc->ep [i].hwep_num))
				udc_handle_ep(udc, &udc->ep[i]);
		}
#endif
	}

	spin_unlock(&udc->lock);
	return IRQ_HANDLED;
}

static irqreturn_t lpc32xx_usb_devdma_irq(int irq, void *_udc)
{
	struct lpc32xx_udc *udc = _udc;

#if defined(UDC_ENABLE_DMA)
	int i;
	u32 tmp;

	spin_lock(&udc->lock);

	/* Handle EP DMA EOT interrupts */
	tmp = __raw_readl(USBD_EOTINTST(udc->udp_baseaddr)) |
		__raw_readl(USBD_NDDRTINTST(udc->udp_baseaddr)) |
		__raw_readl(USBD_SYSERRTINTST(udc->udp_baseaddr));
	for (i = 1; i < NUM_ENDPOINTS; i++) {
		if (tmp & (1 << udc->ep [i].hwep_num))
			udc_handle_dma_ep(udc, &udc->ep[i]);
	}

	spin_unlock(&udc->lock);
#else
	disable_irq(udc->udp_irq[IRQ_USB_DEVDMA]);
#endif

	return IRQ_HANDLED;
}

/*
 *
 * VBUS detection, pullup handler, and Gadget cable state notification
 *
 */
static int vbus_handler_thread(void *udc_)
{
	struct lpc32xx_udc *udc = udc_;
	u8 value;

	/* The main loop */
        while (!kthread_should_stop()) {
		/* Get the interrupt from the transceiver */
		value = i2c_read(ISP1301_I2C_INTERRUPT_LATCH);

		/* Discharge VBUS real quick */
		i2c_write(OTG1_VBUS_DISCHRG, ISP1301_I2C_OTG_CONTROL_1);

		/* Give VBUS some time (200mS) to discharge */
                set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(200 / (1000 / HZ));
                set_current_state(TASK_INTERRUPTIBLE);

		/* Disable VBUS discharge resistor */
		i2c_write(OTG1_VBUS_DISCHRG,
			(ISP1301_I2C_OTG_CONTROL_1 | ISP1301_I2C_REG_CLEAR_ADDR));

		if (udc->enabled != 0) {
			/* Get the VBUS status from the transceiver */
			value = i2c_read(ISP1301_I2C_OTG_CONTROL_2);

			/* VBUS on or off? */
			if (value & OTG_B_SESS_VLD) {
				udc->vbus = 1;

				/* Enable USB clocks */
				udc_clk_set(udc, 1);

				/* Setup the UDC and ep0 */
				udc_enable(udc);
			}
			else {
				/* Will force disconnect */
				udc->vbus = 0;
			}

			/* VBUS changed? */
			if (udc->last_vbus != udc->vbus) {
				lpc32xx_vbus_session(&udc->gadget, udc->vbus);
				udc->last_vbus = udc->vbus;
			}
		}

		/* Clear interrupt */
		i2c_write(~0, ISP1301_I2C_INTERRUPT_LATCH | ISP1301_I2C_REG_CLEAR_ADDR);

		if (udc->irq_asrtd == 0) {
			udc->irq_asrtd = 1;
			enable_irq(udc->udp_irq[IRQ_USB_ATX]);
		}

		/* sleep if nothing to send */
                set_current_state(TASK_INTERRUPTIBLE);
                schedule();
	}

	udc->thread_task = NULL;

	return 0;
}

static irqreturn_t lpc32xx_usb_vbus_irq(int irq, void *_udc)
{
	struct lpc32xx_udc *udc = _udc;

	/* Kick off the VBUS handler thread - it will re-enable the
	   VBUS interrupt when it is done */
	disable_irq(udc->udp_irq[IRQ_USB_ATX]);
	udc->irq_asrtd = 0;
	udc->thread_wakeup_needed = 1;
	wake_up_process(udc->thread_task);

	return IRQ_HANDLED;
}

int usb_gadget_register_driver (struct usb_gadget_driver *driver)
{
	struct lpc32xx_udc *udc = &controller;
	int retval;

	if (!driver || driver->speed < USB_SPEED_FULL ||
		!driver->bind || !driver->setup) {
		dev_err(udc->dev, "bad parameter.\n");
		return -EINVAL;
	}

	if (udc->driver) {
		dev_err(udc->dev, "UDC already has a gadget driver\n");
		return -EBUSY;
	}

	udc->driver = driver;
	udc->gadget.dev.driver = &driver->driver;
	udc->gadget.dev.driver_data = &driver->driver;
	udc->enabled = 1;
	udc->selfpowered = 1;
	udc->vbus = 0;

	retval = driver->bind(&udc->gadget);
	if (retval) {
		dev_err(udc->dev, "driver->bind() returned %d\n", retval);
		udc->driver = NULL;
		udc->gadget.dev.driver = NULL;
		udc->gadget.dev.driver_data = NULL;
		udc->enabled = 0;
		udc->selfpowered = 0;
		return retval;
	}

	dev_dbg(udc->dev, "bound to %s\n", driver->driver.name);

	/* Force VBUS process once to check for cable insertion */
	udc->last_vbus = udc->vbus = 0;
	wake_up_process(udc->thread_task);

	return 0;
}
EXPORT_SYMBOL (usb_gadget_register_driver);

int usb_gadget_unregister_driver (struct usb_gadget_driver *driver)
{
	struct lpc32xx_udc *udc = &controller;

	if (!driver || driver != udc->driver || !driver->unbind)
		return -EINVAL;

	local_irq_disable();
	udc->enabled = 0;
	pullup(udc, 0);
	local_irq_enable();

	driver->unbind(&udc->gadget);
	udc->gadget.dev.driver = NULL;
	udc->gadget.dev.driver_data = NULL;
	udc->driver = NULL;

	dev_dbg(udc->dev, "unbound from %s\n", driver->driver.name);
	return 0;
}
EXPORT_SYMBOL (usb_gadget_unregister_driver);

/*-------------------------------------------------------------------------*/

static void lpc32xx_udc_shutdown(struct platform_device *dev)
{
	/* Force disconnect on reboot */
	struct lpc32xx_udc *udc = &controller;

	pullup(udc, 0);
}

static int __init lpc32xx_udc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lpc32xx_udc *udc = &controller;
	int retval, i;
	struct resource *res;
	dma_addr_t dma_handle;

	/* init software state */
	udc->gadget.dev.parent = dev;
	udc->pdev = pdev;
	udc->dev = &pdev->dev;
	udc->enabled = 0;

	if (!dev->platform_data) {
		dev_err(udc->dev, "missing platform_data\n");
		return -ENODEV;
	}
	udc->board = (struct lpc32xx_usbd_cfg *) dev->platform_data;

	/*
	 * Resources are mapped as follows:
	 *  [0] = IORESOURCE_MEM, base address and size of USB space
	 *  [1] = IORESOURCE_IRQ, USB device low priority interrupt number
	 *  [2] = IORESOURCE_IRQ, USB device high priority interrupt number
	 *  [3] = IORESOURCE_IRQ, USB device interrupt number
	 *  [4] = IORESOURCE_IRQ, USB transciever interrupt number
	 */
	if (pdev->num_resources != 5) {
		dev_err(udc->dev, "invalid num_resources\n");
		return -ENODEV;
	}
	if (pdev->resource[0].flags != IORESOURCE_MEM) {
		dev_err(udc->dev, "invalid resource type\n");
		return -ENODEV;
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	spin_lock_init(&udc->lock);

	udc->io_p_start = res->start;
	udc->io_p_size = res->end - res->start + 1;
	if (!request_mem_region(udc->io_p_start, udc->io_p_size, driver_name)) {
		dev_err(udc->dev, "someone's using UDC memory\n");
		return -EBUSY;
	}

	/* Get IRQs */
	for (i = 0; i < 4; i++) {
		if (pdev->resource[1].flags != IORESOURCE_IRQ) {
			dev_err(udc->dev, "invalid resource type\n");
			return -ENODEV;
		}

		udc->udp_irq[i] = platform_get_irq(pdev, i);
	}

	/* Enable AHB slave USB clock, needed for further USB clock control */
	__raw_writel(USB_SLAVE_HCLK_EN | (1 << 19), USB_CTRL);

	retval = i2c_add_driver(&isp1301_driver);
	if (retval < 0) {
		dev_err(udc->dev, "failed to connect I2C to ISP1301 USB Transceiver");
		return -ENODEV;
	}
	if (isp1301_i2c_client == NULL)
	{
		/* Client not found */
		dev_err(udc->dev, "I2C slave device not found");
		return -ENODEV;
	}
	dev_info(udc->dev, "I2C device at address 0x%x", isp1301_i2c_client->addr);

	/* ISP1301 transceiver configuration */
	isp1301_udc_configure(udc);

	/* Get required clocks */
	udc->usb_pll_clk = clk_get(&pdev->dev, "ck_pll5");
	if (IS_ERR(udc->usb_pll_clk)) {
		dev_err(udc->dev, "failed to acquire USB PLL");
		retval = PTR_ERR(udc->usb_pll_clk);
		goto pll_get_fail;
	}
	udc->usb_slv_clk = clk_get(&pdev->dev, "ck_usbd");
	if (IS_ERR(udc->usb_slv_clk)) {
		dev_err(udc->dev, "failed to acquire USB device clock");
		retval = PTR_ERR(udc->usb_slv_clk);
		goto usb_clk_get_fail;
	}

	/* Setup PLL clock to 48MHz */
	retval = clk_enable(udc->usb_pll_clk);
	if (retval < 0) {
		dev_err(udc->dev, "failed to start USB PLL");
		goto pll_enable_fail;
	}

	retval = clk_set_rate(udc->usb_pll_clk, 48000);
	if (retval < 0) {
		dev_err(udc->dev, "failed to set USB clock rate");
		goto pll_set_fail;
	}

	__raw_writel(__raw_readl(USB_CTRL) | USB_DEV_NEED_CLK_EN, USB_CTRL);

	/* Enable USB device clock */
	retval = clk_enable(udc->usb_slv_clk);
	if (retval < 0) {
		dev_err(udc->dev, "failed to start USB device clock");
		goto usb_clk_enable_fail;
	}

	/* Set to enable all needed USB OTG clocks */
	__raw_writel(USB_CLOCK_MASK, USB_OTG_CLK_CTRL);

	while ((__raw_readl(USB_OTG_CLK_STAT) & USB_CLOCK_MASK) !=
	       USB_CLOCK_MASK);

	/* All clocks are now on */
	udc->clocked = 1;

	/* Map register space */
	udc->udp_baseaddr = ioremap(udc->io_p_start, udc->io_p_size);
	if (!udc->udp_baseaddr) {
		retval = -ENOMEM;
		dev_err(udc->dev, "IO map failure");
		goto io_map_fail;
	}

	/* Allocate memory for the UDCA */
	udc->udca_v_base = dma_alloc_coherent(&pdev->dev, UDCA_BUFF_SIZE,
		&dma_handle, (GFP_KERNEL | GFP_DMA));
	if (!udc->udca_v_base)
	{
		dev_err(udc->dev, "error getting UDCA region");
		retval = -ENOMEM;
		goto dma_alloc_fail;
	}
	udc->udca_p_base = (void *) dma_handle;
	dev_dbg(udc->dev, "DMA buffer(0x%x bytes), P:0x%08x, V:0x%08x",
		UDCA_BUFF_SIZE, (u32) udc->udca_p_base, (u32) udc->udca_v_base);

	/* Setup the DD DMA memory pool */
	udc->dd_cache = dma_pool_create ("udc_dd", udc->dev,
		sizeof (struct lpc32xx_usbd_dd_gad), sizeof (u32), 0);
	if (!udc->dd_cache) {
		dev_err(udc->dev, "error getting DD DMA region");
		retval = -ENOMEM;
		goto dma2_alloc_fail;
	}

	/* Clear USB peripheral and initialize gadget endpoints */
	udc_disable(udc);
	udc_reinit(udc);

	retval = device_register(&udc->gadget.dev);
	if (retval < 0) {
		dev_err(udc->dev, "Device registration failure");
		goto dev_register_fail;
	}

	/* Request IRQs - low and high priority USB device IRQs are routed to the
	   same handler, while the DMA interrupt is routed elsewhere */
	retval = request_irq(udc->udp_irq[IRQ_USB_LP], lpc32xx_usb_lp_irq,
			IRQF_DISABLED, driver_name, udc);
	if (retval < 0) {
		dev_err(udc->dev, "LP request irq %d failed", udc->udp_irq[IRQ_USB_LP]);
		goto irq_lp_fail;
	}
	retval = request_irq(udc->udp_irq[IRQ_USB_HP], lpc32xx_usb_hp_irq,
			IRQF_DISABLED, driver_name, udc);
	if (retval < 0) {
		dev_err(udc->dev, "HP request irq %d failed", udc->udp_irq[IRQ_USB_HP]);
		goto irq_hp_fail;
	}
	retval = request_irq(udc->udp_irq[IRQ_USB_DEVDMA], lpc32xx_usb_devdma_irq,
			IRQF_DISABLED, driver_name, udc);
	if (retval < 0) {
		dev_err(udc->dev, "DEV request irq %d failed", udc->udp_irq[IRQ_USB_DEVDMA]);
		goto irq_dev_fail;
	}

	/* Create VBUS handler thread */
	udc->thread_wakeup_needed = 0;
	udc->thread_task = kthread_create(vbus_handler_thread, udc,
			"vbus_handler_thread");
	if (IS_ERR(udc->thread_task)) {
		retval = PTR_ERR(udc->thread_task);
		dev_err(udc->dev, "VBUS handler thread failures");
		goto vbus_thread_fail;
	}

	/* The transceiver interrupt is used for VBUS detection and will
	   kick off the VBUS handler thread */
	retval = request_irq(udc->udp_irq[IRQ_USB_ATX], lpc32xx_usb_vbus_irq,
			IRQF_DISABLED, driver_name, udc);
	if (retval < 0) {
		dev_err(udc->dev, "VBUS request irq %d failed\n", udc->udp_irq[IRQ_USB_ATX]);
		goto irq_xcvr_fail;
	}

	/* Keep VBUS IRQ disabled until GadgetFS starts up */
	disable_irq(udc->udp_irq[IRQ_USB_ATX]);

	dev_set_drvdata(dev, udc);
	device_init_wakeup(dev, 1);
	create_debug_file(udc);

	/* Disable clocks for now */
	udc_clk_set(udc, 0);

	dev_info(udc->dev, "%s version %s\n", driver_name, DRIVER_VERSION);
	return 0;

irq_xcvr_fail:
	kthread_stop(udc->thread_task);
vbus_thread_fail:
	free_irq(udc->udp_irq[IRQ_USB_DEVDMA], udc);
irq_dev_fail:
	free_irq(udc->udp_irq[IRQ_USB_HP], udc);
irq_hp_fail:
	free_irq(udc->udp_irq[IRQ_USB_LP], udc);
irq_lp_fail:
	device_unregister(&udc->gadget.dev);
dev_register_fail:
	dma_pool_destroy(udc->dd_cache);
dma2_alloc_fail:
	dma_free_coherent(&pdev->dev, UDCA_BUFF_SIZE,
		udc->udca_v_base, (dma_addr_t) udc->udca_p_base);
dma_alloc_fail:
	iounmap(udc->udp_baseaddr);
io_map_fail:
	clk_disable(udc->usb_slv_clk);
usb_clk_enable_fail:
pll_set_fail:
	clk_disable(udc->usb_pll_clk);
pll_enable_fail:
	clk_put(udc->usb_slv_clk);
usb_clk_get_fail:
	clk_put(udc->usb_pll_clk);
pll_get_fail:
	i2c_del_driver(&isp1301_driver);

	release_mem_region(res->start, res->end - res->start + 1);
	dev_err(udc->dev, "%s probe failed, %d\n", driver_name, retval);
	return retval;
}

static int __exit lpc32xx_udc_remove(struct platform_device *pdev)
{
	struct lpc32xx_udc *udc = platform_get_drvdata(pdev);

	if (udc->driver)
		return -EBUSY;

	udc_clk_set(udc, 1);
	udc_disable(udc);
	pullup(udc, 0);

	if (udc->irq_asrtd == 1)
		disable_irq(udc->udp_irq[IRQ_USB_ATX]);
	free_irq(udc->udp_irq[IRQ_USB_ATX], udc);

	device_init_wakeup(&pdev->dev, 0);
	remove_debug_file(udc);

	dma_pool_destroy(udc->dd_cache);
	dma_free_coherent(&pdev->dev, UDCA_BUFF_SIZE,
		udc->udca_v_base, (dma_addr_t) udc->udca_p_base);
	kthread_stop(udc->thread_task);
	free_irq(udc->udp_irq[IRQ_USB_DEVDMA], udc);
	free_irq(udc->udp_irq[IRQ_USB_HP], udc);
	free_irq(udc->udp_irq[IRQ_USB_LP], udc);

	device_unregister(&udc->gadget.dev);

	clk_disable(udc->usb_slv_clk);
	clk_put(udc->usb_slv_clk);
	clk_disable(udc->usb_pll_clk);
	clk_put(udc->usb_pll_clk);
	iounmap(udc->udp_baseaddr);
	i2c_del_driver(&isp1301_driver);
	release_mem_region(udc->io_p_start, udc->io_p_size);

	return 0;
}

#ifdef CONFIG_PM
static int lpc32xx_udc_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct lpc32xx_udc *udc = platform_get_drvdata(pdev);

	if (udc->clocked) {
		/* Power down ISP */
		isp1301_set_powerstate(0);

		/* Disable clocking */
		udc_clk_set(udc, 0);

		/* Keep clock flag on, so we know to re-enable clocks
		   on resume */
		udc->clocked = 1;

		/* Kill OTG and I2C clocks */
		__raw_writel(0, USB_OTG_CLK_CTRL);
		while ((__raw_readl(USB_OTG_CLK_STAT) & OTGOFF_CLK_MASK) !=
		       OTGOFF_CLK_MASK);

		/* Kill global USB clock */
		clk_disable(udc->usb_slv_clk);
	}

	return 0;
}

static int lpc32xx_udc_resume(struct platform_device *pdev)
{
	struct lpc32xx_udc *udc = platform_get_drvdata(pdev);

	if (udc->clocked) {
		/* Enable global USB clock */
		clk_enable(udc->usb_slv_clk);

		/* Enable clocking */
		udc_clk_set(udc, 1);

		/* ISP back to normal power mode */
		isp1301_set_powerstate(1);
	}

	return 0;
}
#else
#define	lpc32xx_udc_suspend	NULL
#define	lpc32xx_udc_resume	NULL
#endif

static struct platform_driver lpc32xx_udc_driver = {
	.probe		= lpc32xx_udc_probe,
	.remove		= __exit_p(lpc32xx_udc_remove),
	.shutdown	= lpc32xx_udc_shutdown,
	.suspend	= lpc32xx_udc_suspend,
	.resume		= lpc32xx_udc_resume,
	.driver		= {
		.name	= (char *) driver_name,
		.owner	= THIS_MODULE,
	},
};

static int __init udc_init_module(void)
{
	return platform_driver_register(&lpc32xx_udc_driver);
}
module_init(udc_init_module);

static void __exit udc_exit_module(void)
{
	platform_driver_unregister(&lpc32xx_udc_driver);
}
module_exit(udc_exit_module);

MODULE_DESCRIPTION("LPC32XX udc driver");
MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lpc32xx_udc");

