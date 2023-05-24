// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RZ/T2H WDT Watchdog Driver
 *
 * Copyright (C) 2023 Renesas Electronics Corporation
 */
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/units.h>
#include <linux/watchdog.h>

#define WDTRR			0x00	/* RW,8 */
#define WDTCR			0x02	/* RW,16 */
#define WDTSR			0x04	/* RW,16 */
#define WDTRCR			0x06	/* RW,8 */

#define WDTCR_TOP_1024		0x00
#define WDTCR_TOP_4096		0x01
#define WDTCR_TOP_8192		0x02
#define WDTCR_TOP_16384		0x03

#define WDTCR_CKS_CLK_4		0x10
#define WDTCR_CKS_CLK_64	0x40
#define WDTCR_CKS_CLK_128	0xF0
#define WDTCR_CKS_CLK_512	0x60
#define WDTCR_CKS_CLK_2048	0x70
#define WDTCR_CKS_CLK_8192	0x80

#define WDTCR_RPES_75		0x00
#define WDTCR_RPES_50		0x100
#define WDTCR_RPES_25		0x200
#define WDTCR_RPES_0		0x300

#define WDTCR_RPSS_25		0x00
#define WDTCR_RPSS_50		0x1000
#define WDTCR_RPSS_75		0x2000
#define WDTCR_RPSS_100		0x3000

#define WDTSR_UNDFF		BIT(14)
#define WDTSR_REFEF		BIT(15)
#define WDTRCR_RSTIRQS		BIT(7)

#define WDTDCR_WDTSTOPCTRL	BIT(0)
#define WDTDCR_WDTSTOPMASK	BIT(16)

#define CLOCK_DEVIDED_BY_8192	8192

#define WDT_DEFAULT_TIMEOUT	60U

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
				__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

struct rzt2h_wdt_priv {
	void __iomem *base;
	void __iomem *base_dbg;
	struct watchdog_device wdev;
	unsigned long pclkl_rate;
	struct clk *pclkl;
};

static u32 rzt2h_wdt_get_cycle_usec(struct rzt2h_wdt_priv *priv,
						unsigned long cycle,
						u16 wdttime)
{
	int clock_division_ratio;

	u64 timer_cycle_us;

	clock_division_ratio = CLOCK_DEVIDED_BY_8192;

	timer_cycle_us = clock_division_ratio * (wdttime + 1) * MICRO;

	return div64_ul(timer_cycle_us, cycle);
}

static void rzt2h_wdt_refresh_counter(struct watchdog_device *wdev)
{
	struct rzt2h_wdt_priv *priv = watchdog_get_drvdata(wdev);
	unsigned long delay;

	writeb(0x0, priv->base + WDTRR);
	writeb(0xFF, priv->base + WDTRR);

	/* Refreshing the down-counter requires up to 4 cycles
	 * of the signal for counting
	 */
	delay = 4 * rzt2h_wdt_get_cycle_usec(priv, priv->pclkl_rate, 0);
	udelay(delay);
}

static void rzt2h_wdt_setup(struct watchdog_device *wdev)
{
	struct rzt2h_wdt_priv *priv = watchdog_get_drvdata(wdev);

	u16 reg;

	/* Setup WDTCR
	 * - CKS[7:4] - Clock Division Ratio Select - 1000b: pclkl/8192
	 * - RPSS[13:12] - Window Start Position Select - 11b: 100%
	 * - RPES[9:8] - Window End Position Select - 11b: 0%
	 * - TOPS[1:0] - Timeout Period Select - 11b: 16384 cycles (3FFFh)
	 */
	reg = WDTCR_CKS_CLK_8192 | WDTCR_RPSS_100 | WDTCR_RPES_0 | WDTCR_TOP_16384;
	writew(reg, priv->base + WDTCR);

	/* Setup RSTIRQS
	 * RSTIRQS[7] - Reset Interrupt Request Selection: Error notification to ICU is permitted.
	 */
	reg &= 0;
	reg &= ~WDTRCR_RSTIRQS;
	writeb(reg, priv->base + WDTRCR);
}

static int rzt2h_wdt_start(struct watchdog_device *wdev)
{
	struct rzt2h_wdt_priv *priv = watchdog_get_drvdata(wdev);
	u32 reg;

	pm_runtime_get_sync(wdev->parent);

	/* Initialize WDT Control*/
	rzt2h_wdt_setup(wdev);

	/* Start the count in case of wdt has been stopped by WDTSTOPCTRL */
	reg = readl(priv->base_dbg);
	reg &= ~WDTDCR_WDTSTOPCTRL;

	writel(reg, priv->base_dbg);

	rzt2h_wdt_refresh_counter(wdev);

	return 0;
}

static int rzt2h_wdt_stop(struct watchdog_device *wdev)
{
	struct rzt2h_wdt_priv *priv = watchdog_get_drvdata(wdev);
	u32 reg;

	pm_runtime_put(wdev->parent);

	/* Stop the count using WDTSTOPCTRL */
	reg = readl(priv->base_dbg);
	reg |= WDTDCR_WDTSTOPCTRL;

	writel(reg, priv->base_dbg);

	return 0;
}


static const struct watchdog_info rzt2h_wdt_ident = {
	.options = WDIOF_MAGICCLOSE | WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT,
	.identity = "Renesas RZ/T2H WDT Watchdog",
};

static int rzt2h_wdt_ping(struct watchdog_device *wdev)
{
	rzt2h_wdt_refresh_counter(wdev);

	return 0;
}

static const struct watchdog_ops rzt2h_wdt_ops = {
	.owner = THIS_MODULE,
	.start = rzt2h_wdt_start,
	.stop = rzt2h_wdt_stop,
	.ping = rzt2h_wdt_ping,
};

static void rzt2h_wdt_pm_disable(void *data)
{
	struct watchdog_device *wdev = data;

	pm_runtime_disable(wdev->parent);
}

static int rzt2h_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rzt2h_wdt_priv *priv;
	unsigned long rate;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_platform_ioremap_resource_byname(pdev, "wdt");
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->base_dbg = devm_platform_ioremap_resource_byname(pdev, "dbg");
	if (IS_ERR(priv->base_dbg))
		return PTR_ERR(priv->base_dbg);

	/* Get watchdog pclkl clock */
	priv->pclkl = devm_clk_get(&pdev->dev, "pclkl");
	if (IS_ERR(priv->pclkl))
		return dev_err_probe(&pdev->dev, PTR_ERR(priv->pclkl), "no pclkl");

	priv->pclkl_rate = clk_get_rate(priv->pclkl);
	if (!priv->pclkl_rate)
		return dev_err_probe(&pdev->dev, -EINVAL, "pclkl rate is 0");

	pm_runtime_enable(&pdev->dev);

	priv->wdev.info = &rzt2h_wdt_ident;
	priv->wdev.ops = &rzt2h_wdt_ops;
	priv->wdev.parent = dev;
	/*
	 * Since the max possible timeout of our 14-bit count
	 * register is less than 3 seconds, so we use
	 * max_hw_heartbeat_ms.
	 */
	rate = priv->pclkl_rate / 8192;
	priv->wdev.max_hw_heartbeat_ms = (1000 * 16384) / rate;
	dev_dbg(dev, "max hw timeout of %dms\n",
		priv->wdev.max_hw_heartbeat_ms);

	priv->wdev.min_timeout = 1;
	priv->wdev.timeout = WDT_DEFAULT_TIMEOUT;

	watchdog_set_drvdata(&priv->wdev, priv);
	ret = devm_add_action_or_reset(&pdev->dev,
					rzt2h_wdt_pm_disable,
					&priv->wdev);
	if (ret < 0)
		return ret;

	watchdog_set_nowayout(&priv->wdev, nowayout);
	watchdog_stop_on_unregister(&priv->wdev);

	ret = watchdog_init_timeout(&priv->wdev, 0, dev);
	if (ret)
		dev_warn(dev, "Specified timeout invalid, using default");

	return devm_watchdog_register_device(&pdev->dev, &priv->wdev);
}

static const struct of_device_id rzt2h_wdt_ids[] = {
	{ .compatible = "renesas,r9a09g077-wdt", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rzt2h_wdt_ids);

static struct platform_driver rzt2h_wdt_driver = {
	.driver = {
		.name = "rzt2h_wdt",
		.of_match_table = rzt2h_wdt_ids,
	},
	.probe = rzt2h_wdt_probe,
};
module_platform_driver(rzt2h_wdt_driver);
MODULE_AUTHOR("Renesas");
MODULE_DESCRIPTION("Renesas RZ/T2H WDT Watchdog Driver");
