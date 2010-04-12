/*
 * asm-arm/arch-lpc32xx/dma.h
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

#ifndef __ASM_ARCH_DMA_H
#define __ASM_ARCH_DMA_H

#include <mach/platform.h>

#define MAX_DMA_CHANNELS 8

#define DMA_CH_SDCARD_TX 0
#define DMA_CH_SDCARD_RX 1
#define DMA_CH_I2S_TX 2
#define DMA_CH_I2S_RX 3

enum {
	DMA_INT_UNKNOWN = 0,
	DMA_ERR_INT = 1,
	DMA_TC_INT = 2,
};

/*
 * DMA channel control structure
 */
struct dma_config {
	int ch;		/* Channel # to use */
	int tc_inten;	/* !0 = Enable TC interrupts for this channel */
	int err_inten;	/* !0 = Enable error interrupts for this channel */
	int src_size;	/* Source xfer size - must be 1, 2, or 4 */
	int src_inc;	/* !0 = Enable source address increment */
	int src_ahb1;	/* !0 = Use AHB1 for source transfer */
	int src_bsize;	/* Source burst size (ie, DMAC_CHAN_SRC_BURST_xxx) */
	u32 src_prph;	/* Source peripheral (ie, DMA_PERID_xxxx) */
	int dst_size;	/* Destination xfer size - must be 1, 2, or 4 */
	int dst_inc;	/* !0 = Enable destination address increment */
	int dst_ahb1;	/* !0 = Use AHB1 for destination transfer */
	int dst_bsize;	/* Destination burst size (ie, DMAC_CHAN_DEST_BURST_xxx) */
	u32 dst_prph;	/* Destination peripheral (ie, DMA_PERID_xxxx) */
	u32 flowctrl;	/* Flow control (ie, DMAC_CHAN_FLOW_xxxxxx) */
};

/*
 * Channel enable and disable functions
 */
extern int lpc32xx_dma_ch_enable(int ch);
extern int lpc32xx_dma_ch_disable(int ch);

/*
 * Channel allocation and deallocation functions
 */
extern int lpc32xx_dma_ch_get(struct dma_config *dmachcfg,
				char *name,
				void *irq_handler,
				void *data);
extern int lpc32xx_dma_ch_put(int ch);
extern int lpc32xx_dma_ch_pause_unpause(int ch, int pause);

/*
 * Setup or start an unbound DMA transfer
 */
extern int lpc32xx_dma_start_pflow_xfer(int ch,
					void *src,
					void *dst,
					int enable);

/*
 * DMA linked list support
 */
extern u32 lpc32xx_dma_alloc_llist(int ch,
				   int entries);
extern void lpc32xx_dma_dealloc_llist(int ch);
extern u32 lpc32xx_dma_llist_v_to_p(int ch,
				    u32 vlist);
extern u32 lpc32xx_dma_llist_p_to_v(int ch,
				    u32 plist);
extern u32 lpc32xx_dma_get_llist_head(int ch);
extern void lpc32xx_dma_flush_llist(int ch);
extern u32 lpc32xx_dma_queue_llist_entry(int ch,
					 void *src,
					 void *dst,
					 int size);
extern u32 lpc32xx_get_free_llist_entry(int ch);

#endif /* _ASM_ARCH_DMA_H */

