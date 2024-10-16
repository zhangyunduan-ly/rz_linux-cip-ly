// SPDX-License-Identifier: GPL-2.0
/*
 * xHCI host controller driver for R-Car SoCs
 *
 * Copyright (C) 2014 Renesas Electronics Corporation
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/usb/phy.h>

#include "xhci.h"
#include "xhci-plat.h"
#include "xhci-rzv2h.h"

/* Interrupt Enable */
#define USB3_HOST_INTEN				0x1044

#define USB3_HOST_INTEN_XHC			BIT(0)
#define USB3_HOST_INTEN_HSE			BIT(2)
#define USB3_HOST_INTEN_ENA			(USB3_HOST_INTEN_XHC \
						| USB3_HOST_INTEN_HSE)

#define USBTEST_ADDR_BASE			0x20000
#define USBTEST_ADDR_SIZE			0x10000

/* USB2TEST Registers */
#define USB2TEST_RESET				0x0
#define USB2TEST_OTGR				0x600
#define USB2TEST_UTMICTRL2			0xB04

/* USB3TEST Registers */
#define USB3TEST_RSTCTRL			0x1000
#define USB3TEST_CLKCTRL			0x1004
#define USB3TEST_RAMCTRL			0x100C
#define USB3TEST_RAMCTRL_SRAM_INIT_DONE		BIT(2)
#define USB3TEST_CREGCTRL			0x1010
#define USB3TEST_CREGCTRL_PARA_SEL		BIT(8)
#define USB3TEST_LANECONFIG0			0x1030

void xhci_rzv2h_start(struct usb_hcd *hcd)
{
	u32 int_en;

	if (hcd->regs) {
		/* Interrupt Enable */
		int_en = readl(hcd->regs + USB3_HOST_INTEN);
		int_en |= USB3_HOST_INTEN_ENA;
		writel(int_en, hcd->regs + USB3_HOST_INTEN);
	}
}
