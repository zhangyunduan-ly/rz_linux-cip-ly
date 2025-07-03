/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Renesas SPI driver
 *
 * Copyright (C) 2012  Renesas Solutions Corp.
 */

#ifndef __LINUX_SPI_RENESAS_SPI_H__
#define __LINUX_SPI_RENESAS_SPI_H__

enum {
	RSPI_SPI_MASTER,
	RSPI_SPI_SLAVE,
};
#define mDataPktBufNum		(4)

struct rspi_plat_data {
	unsigned int dma_tx_id;
	unsigned int dma_rx_id;

	u16 num_chipselect;
};

struct rspi_data {
	void __iomem *addr;
	u32 speed_hz;
	struct spi_controller *ctlr;
	struct platform_device *pdev;
	wait_queue_head_t wait;
	spinlock_t lock;		/* Protects RMW-access to RSPI_SSLP */
	struct clk *clk;
	u16 spcmd;
	u8 spsr;
	u8 sppcr;
	int rx_irq, tx_irq;
	int bits_per_word;
	const struct spi_ops *ops;

	unsigned dma_callbacked:1;
	unsigned byte_access:1;
	struct reset_control *rstc;

	/* Ring buffer for slave idle RX */
	u32 ulDataPktLen;
	void *ucpRxBuf;
	struct scatterlist		sg_rx[mDataPktBufNum];	
	int			active_rx;	
	size_t ulRxBufSize;
	volatile size_t ulRxHead;
	volatile size_t ulRxTail;
	
};
#endif
