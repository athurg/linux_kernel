/*
 * drivers/usb/gadget/lpc32xx_udc.h
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
 *
 * Copyright (C) 2009 NXP Semiconductors
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

#ifndef LPC32XX_UDC_H
#define LPC32XX_UDC_H

/*
 * Although the driver supports DMA, it is not yet working completely. It
 * seems to work ok for serial ACM, but eventually bonks out for MSC and
 * ether classes. Disabling this define will use FIFO mode for all EPs
 */
//#define UDC_ENABLE_DMA

/*
 * controller driver data structures
 */

/* 16 endpoints (not to be confused with 32 hardware endpoints) */
#define	NUM_ENDPOINTS	16

/*
 * IRQ indices make reading the code a little easier
 */
#define IRQ_USB_LP	0
#define IRQ_USB_HP	1
#define IRQ_USB_DEVDMA	2
#define IRQ_USB_ATX	3

#define EP_OUT 0 /* RX (from host) */
#define EP_IN 1 /* TX (to host) */

/* Returns the interrupt mask for the selected hardware endpoint */
#define EP_MASK_SEL(ep, dir) (1 << (((ep) * 2) + dir))

#define EP_INT_TYPE 0
#define EP_ISO_TYPE 1
#define EP_BLK_TYPE 2
#define EP_CTL_TYPE 3

/* EP0 states */
#define WAIT_FOR_SETUP 0 /* Wait for setup packet */
#define DATA_IN        1 /* Expect dev->host transfer */
#define DATA_OUT       2 /* Expect host->dev transfer */
#define WAIT_OUT       3 /* ??? */

/* DD (DMA Descriptor) structure, requires word alignment, this is already defined
   in the LPC32XX USB device header file, but this version si slightly modified to
   tag some work data with each DMA descriptor. */
struct lpc32xx_usbd_dd_gad;
struct lpc32xx_usbd_dd_gad
{
	struct lpc32xx_usbd_dd_gad *dd_next_phy;
	u32 dd_setup;
	u32 dd_buffer_addr;
	u32 dd_status;
	u32 *dd_iso_ps_mem_addr;
	dma_addr_t this_dma;
	u32 iso_status[5];
	struct lpc32xx_usbd_dd_gad *dd_next_v;
};

/*
 * Logical endpoint structure
 */
struct lpc32xx_ep {
	struct usb_ep		ep;
	struct list_head	queue;
	struct lpc32xx_udc	*udc;

	u32			hwep_num_base; /* Physical hardware EP */
	u32			hwep_num; /* Maps to hardware endpoint */
	u32			maxpacket;
	u32			doublebuff;
	u32			lep;

	u32			is_in:1;
	u32			uses_dma:1;
	volatile u32		req_pending:1;
	u32			eptype;

	/* Statuses for proc, NAK and stall aren't used */
	u32                     totalints;
	u32			totalnaks;
	u32			totalstalls;

	const struct usb_endpoint_descriptor *desc;
};

/*
 * Common UDC structure
 */
struct lpc32xx_udc {
	struct usb_gadget	gadget;
	struct usb_gadget_driver *driver;
	struct platform_device	*pdev;
	struct device		*dev;
	struct proc_dir_entry	*pde;
	spinlock_t		lock;

	/* Board and device specific */
	struct lpc32xx_usbd_cfg	*board;
	u32			io_p_start;
	u32			io_p_size;
	void __iomem		*udp_baseaddr;
	int			udp_irq[4];
	struct clk		*usb_pll_clk;
	struct clk		*usb_slv_clk;

	/* DMA support */
	u32			*udca_v_base;
	u32			*udca_p_base;
	struct dma_pool		*dd_cache;

	/* Common EP and control data */
	u32			enabled_devints;
	u32			enabled_hwepints;
	u32			dev_status;
	u32			realized_eps;

	/* VBUS thread support */
	struct task_struct	*thread_task;
	volatile int 		thread_wakeup_needed;
	u8			vbus;
	u8			last_vbus;
	volatile int		irq_asrtd;

	/* USB device peripheral - various */
	struct lpc32xx_ep	ep[NUM_ENDPOINTS];
	u32			enabled:1;
	u32			clocked:1;
	u32			suspended:1;
	u32			selfpowered:1;
	int                     ep0state;
};

/*
 * Endpoint request
 */
struct lpc32xx_request {
	struct usb_request	req;
	struct list_head	queue;
	struct lpc32xx_usbd_dd_gad *dd_desc_ptr;
	u32			mapped:1;
	u32			send_zlp:1;
};

static inline struct lpc32xx_udc *to_udc(struct usb_gadget *g)
{
	return container_of(g, struct lpc32xx_udc, gadget);
}

#define ep_dbg(epp, fmt, arg...) \
	dev_dbg(epp->udc->dev, "%s:%s: " fmt, epp->ep.name, __func__, ## arg)
#define ep_err(epp, fmt, arg...) \
	dev_err(epp->udc->dev, "%s:%s: " fmt, epp->ep.name, __func__, ## arg)
#define ep_info(epp, fmt, arg...) \
	dev_info(epp->udc->dev, "%s:%s: " fmt, epp->ep.name, __func__, ## arg)
#define ep_warn(epp, fmt, arg...) \
	dev_warn(epp->udc->dev, "%s:%s:" fmt, epp->ep.name, __func__, ## arg)

#endif

