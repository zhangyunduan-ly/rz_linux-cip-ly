/* SPDX-License-Identifier: GPL-2.0 OR MIT */

#ifndef __RZT2H_ETHSS_H__
#define __RZT2H_ETHSS_H__

/**
 * struct ethss - MII converter structure
 * @base: base address of the MII converter
 * @dev: Device associated to the MII converter
 * @lock: Lock used for read-modify-write access
 */
struct ethss {
	void __iomem *base;
	struct device *dev;
	struct clk *clk;
	struct reset_control *rst_ethss;
	struct reset_control *rst_conv;
	spinlock_t lock;
};

/**
 * struct ethss_port - Per port MII converter struct
 * @ethss: backiling to MII converter structure
 * @port: port number
 * @interface: interface mode of the port
 */
struct ethss_port {
	struct ethss *ethss;
	int port;
	phy_interface_t interface;
};

void ethss_destroy(struct ethss_port *ethss_port);
struct ethss_port *ethss_create(struct device *dev, struct device_node *np);
void ethss_link_up(struct ethss_port *ethss_port, phy_interface_t interface,
		   int speed, int duplex);
int ethss_config(struct ethss_port *ethss_port, phy_interface_t interface);

#endif
