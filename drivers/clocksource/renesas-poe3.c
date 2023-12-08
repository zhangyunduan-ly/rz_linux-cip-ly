// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas Port Output Enable 3 (POE3) driver
 *
 * This program is free software; you can redistribute and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Copyright (C) 2021 Renesas Electronics Corp.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/reset.h>
#include <linux/of_device.h>

#define ICSR1		0x00 /* Input level control/status register 1 */
#define OCSR1		0x02 /* Output level control/status register 1 */
#define ICSR2		0x04 /* Input level control/status register 2 */
#define OCSR2		0x06 /* Output level control/status register 1 */
#define ICSR3		0x08 /* Input level control/status register 3 */
#define SPOER		0x0A /* Software port output enable register */
#define POECR1		0x0B /* Port output enable control register 1 */
#define POECR2		0x0C /* Port output enable control register 2 */
#define POECR4		0x10 /* Port output enable control register 4 */
#define POECR5		0x12 /* Port output enable control register 5 */
#define POECR7		0x36 /* Port output enable control register 7 */
#define POECR8		0x38 /* Port output enable control register 8 */
#define POECR9		0x3A /* Port output enable control register 9 */
#define POECR10		0x3C /* Port output enable control register 10 */
#define POECR11		0x3E /* Port output enable control register 11 */
#define POECR12		0x40 /* Port output enable control register 12 */
#define ICSR4		0x16 /* Input level control/status register 4 */
#define ICSR5		0x18 /* Input level control/status register 5 */
#define ICSR6		0x1C /* Input level control/status register 6 */
#define ICSR9		0x32 /* Input level control/status register 9 */
#define ICSR10		0x34 /* Input level control/status register 10 */
#define ALR1		0x1A /* Active level setting register 1 */
#define M0SELR1		0x24 /* MTU0 pin select register 1 */
#define M0SELR2		0x25 /* MTU0 pin select register 2 */
#define M3SELR		0x26 /* MTU3 pin select register */
#define M4SELR1		0x27 /* MTU4 pin select register 1 */
#define M4SELR2		0x28 /* MTU4 pin select register 2 */
#define M6SELR		0x2A /* MTU6 pin select register */
#define M7SELR1		0x2B /* MTU7 pin select register 1 */
#define M7SELR2		0x2C /* MTU7 pin select register 2 */

#define OCSR_OCE		BIT(9) /* Output Short High-Impedance Enable */
#define OCSR_OIE		BIT(8) /* Output Short Interrupt */
#define OCSR_OSF		BIT(15) /* Output Short Flag */
#define ICSR_PIE		BIT(8) /* Port Interrupt Enable */
#define ICSR_POEF		BIT(12) /* High-impedance request by input */
#define ICSR34_POEE		BIT(9) /* High-impedance Enable of ICSR3 and ICSR4 */
#define ICSR5_POEE		BIT(5) /* High-impedance Enable of ICSR5 */
#define ICSR6_OSTSTE		BIT(9) /* Oscillation Stop High-Impedance Enable */
#define ICSR6_OSTSTF		BIT(12) /* Oscillation Stop High-Impedance Flag */
#define ICSR9_D0ERR0IE		BIT(0) /* DSMIF0 Error 0 Interrupt Enable */
#define ICSR10_D0ERR1IE		BIT(0) /* DSMIF0 Error 1 Interrupt Enable */
#define SPOER_MTUCH0HIZ		BIT(2) /* Place MTU0 output in High-Z state */
#define SPOER_MTUCH67HIZ	BIT(1) /* Place MTU67 output in High-Z state */
#define SPOER_MTUCH34HIZ	BIT(0) /* Place MTU34 output in High-Z state */
#define POECR1_MTU0AZE		0x0001 /* MTIOC0A High-impedance Enable */
#define POECR2_MTU3BDZE		0x0400 /* MTIOC3B/D High-impedance Enable */
#define POECR2_MTU6BDZE		0x0004 /* MTIOC6B/D High-impedance Enable */
#define POECR4_IC2ADDMT34	BIT(2) /* MTU34 High-impedance add POE4F */
#define POECR4_IC3ADDMT34	BIT(3) /* MTU34 High-impedance add POE8F */
#define POECR4_IC4ADDMT34	BIT(4) /* MTU34 High-impedance add POE10F */
#define POECR4_IC5ADDMT34	BIT(5) /* MTU34 High-impedance add POE11F */
#define POECR4_IC1ADDMT67	BIT(9) /* MTU67 High-impedance add POE0F */
#define POECR4_IC3ADDMT67	BIT(11) /* MTU67 High-impedance add POE8F */
#define POECR4_IC4ADDMT67	BIT(12) /* MTU67 High-impedance add POE10F */
#define POECR4_IC5ADDMT67	BIT(13) /* MTU67 High-impedance add POE11F */
#define POECR5_IC1ADDMT0	BIT(1) /* MTU0 High-impedance add POE0F */
#define POECR5_IC2ADDMT0	BIT(2) /* MTU0 High-impedance add POE4F */
#define POECR5_IC4ADDMT0	BIT(4) /* MTU0 High-impedance add POE10F */
#define POECR5_IC5ADDMT0	BIT(5) /* MTU0 High-impedance add POE11F */
#define POECR7_D0E0ADDMT0	BIT(0) /* MTU0 High-Impedance D0ERR0ST Add */
#define POECR8_D0E1ADDMT0	BIT(0) /* MTU0 High-Impedance D0ERR1ST Add */
#define POECR9_D0E0ADDMT34	BIT(0) /* MTU34 High-Impedance D0ERR0ST Add */
#define POECR10_D0E1ADDMT34	BIT(0) /* MTU34 High-Impedance D0ERR1ST Add */
#define POECR11_D0E0ADDMT67	BIT(0) /* MTU67 High-Impedance D0ERR0ST Add */
#define POECR12_D0E1ADDMT67	BIT(0) /* MTU67 High-Impedance D0ERR1ST Add */
#define ALR1_OLSEN		BIT(7) /* Active Level Setting Enable */
#define ALR1_OLSG0AB		0x0003 /* MTIOC3BD active level high */
#define ALR1_OLSG1AB		0x000C /* MTIOC4AC active level high */
#define ALR1_OLSG2AB		0x0030 /* MTIOC4BD active level high */
#define M0SELR1_M0ASEL		BIT(0) /* Control High-Z of P01_6 when its used as MTIOC0A pin */
#define M0SELR1_M0BSEL		BIT(4) /* Control High-Z of P01_7 when its used as MTIOC0B pin */
#define M0SELR2_M0CSEL		BIT(0) /* Control High-Z of P04_5 when its used as MTIOC0C pin */
#define M0SELR2_M0DSEL		BIT(4) /* Control High-Z of P04_6 when its used as MTIOC0D pin */
#define M3SELR_M3BSEL		BIT(0) /* Control High-Z of P02_5 when its used as MTIOC3B pin */
#define M3SELR_M3DSEL		BIT(4) /* Control High-Z of P02_6 when its used as MTIOC3D pin */
#define M4SELR1_M4ASEL		BIT(0) /* Control High-Z of P02_7 when its used as MTIOC4A pin */
#define M4SELR1_M4CSEL		BIT(4) /* Control High-Z of P03_0 when its used as MTIOC4C pin */
#define M4SELR2_M4BSEL		BIT(0) /* Control High-Z of P03_1 when its used as MTIOC4B pin */
#define M4SELR2_M4DSEL		BIT(4) /* Control High-Z of P03_2 when its used as MTIOC4D pin */
#define M6SELR_M6BSEL		BIT(0) /* Control High-Z of P03_7 when its used as MTIOC6B pin */
#define M6SELR_M6DSEL		BIT(4) /* Control High-Z of P04_0 when its used as MTIOC6D pin */
#define M7SELR1_M7ASEL		BIT(0) /* Control High-Z of P04_1 when its used as MTIOC7A pin */
#define M7SELR1_M7CSEL		BIT(4) /* Control High-Z of P04_2 when its used as MTIOC7C pin */
#define M7SELR2_M7BSEL		BIT(0) /* Control High-Z of P04_3 when its used as MTIOC7B pin */
#define M7SELR2_M7DSEL		BIT(4) /* Control High-Z of P04_4 when its used as MTIOC7D pin */

enum mtu3_channel_port_output {
	MTU3_CHANNEL_0,
	MTU3_CHANNEL_34,
	MTU3_CHANNEL_67,
};

struct renesas_poe3 {
	struct platform_device *pdev;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *rstc;
	struct mutex mutex;
	int dev_base;
	bool is_rzt2h;
};

static inline unsigned int renesas_poe3_read(struct renesas_poe3 *poe3,
					     unsigned int reg_nr)
{
	if (reg_nr == SPOER || reg_nr == POECR1 ||
	    reg_nr == M0SELR1 || reg_nr == M0SELR2 ||
	    reg_nr == M3SELR || reg_nr == M4SELR1 ||
	    reg_nr == M4SELR2 || reg_nr == M6SELR ||
	    reg_nr == M7SELR1 || reg_nr == M7SELR2)
		return ioread8(poe3->base + reg_nr);
	return ioread16(poe3->base + reg_nr);
}

static inline void renesas_poe3_write(struct renesas_poe3 *poe3,
				      unsigned int reg_nr, u16 value)
{
	if (reg_nr == SPOER || reg_nr == POECR1 ||
	    reg_nr == M0SELR1 || reg_nr == M0SELR2 ||
	    reg_nr == M3SELR || reg_nr == M4SELR1 ||
	    reg_nr == M4SELR2 || reg_nr == M6SELR ||
	    reg_nr == M7SELR1 || reg_nr == M7SELR2)
		iowrite8((u8)value, poe3->base + reg_nr);
	else
		iowrite16((u16)value, poe3->base + reg_nr);
}

static int renesas_poe3_clear_Hi_z_state(struct renesas_poe3 *poe3,
					 enum mtu3_channel_port_output ch)
{
	unsigned int poecr12_reg, poecr45_reg, poecr45_offset, ocsr_reg, icsr_reg;
	int i, val;

	val = renesas_poe3_read(poe3, SPOER);

	switch (ch) {
	case MTU3_CHANNEL_0:
		if ((val & SPOER_MTUCH0HIZ) != 0)
			renesas_poe3_write(poe3, SPOER,
				(val & ~SPOER_MTUCH0HIZ));
		poecr12_reg = POECR1;
		poecr45_reg = POECR5;
		poecr45_offset = 0x0001;
		break;
	case MTU3_CHANNEL_34:
		if ((val & SPOER_MTUCH34HIZ) != 0)
			renesas_poe3_write(poe3, SPOER,
				(val & ~SPOER_MTUCH34HIZ));
		poecr12_reg = POECR2;
		poecr45_reg = POECR4;
		poecr45_offset = 0x0001;
		ocsr_reg = OCSR1;
		break;
	case MTU3_CHANNEL_67:
		if ((val & SPOER_MTUCH67HIZ) != 0)
			renesas_poe3_write(poe3, SPOER,
				(val & ~SPOER_MTUCH67HIZ));
		poecr12_reg = POECR2;
		poecr45_reg = POECR4;
		poecr45_offset = 0x0100;
		ocsr_reg = OCSR2;
		break;
	default:
		goto error_ports;
	}

	/* Clear POExF bits*/
	for (i = 1; i <= 5; i++) {
		val = renesas_poe3_read(poe3, poecr45_reg);
		if ((val && (poecr45_offset << i)) != 0) {
			switch (i) {
			case 1:
				icsr_reg = ICSR1;
				break;
			case 2:
				icsr_reg = ICSR2;
				break;
			case 3:
				icsr_reg = ICSR3;
				break;
			case 4:
				icsr_reg = ICSR4;
				break;
			case 5:
				icsr_reg = ICSR5;
				break;
			}

			val = renesas_poe3_read(poe3, icsr_reg);
			if ((val & ICSR_POEF) != 0)
				renesas_poe3_write(poe3,
					icsr_reg, (val & 0x0FFF));
		}
	}

	/* Clear OSF bits */
	if ((ch == MTU3_CHANNEL_34) || (ch == MTU3_CHANNEL_67)) {
		val = renesas_poe3_read(poe3, ocsr_reg);
		if ((val & OCSR_OSF) != 0)
			renesas_poe3_write(poe3, ocsr_reg,
					   (val & 0x0FFF));
	}

	/* Clear OSTSTF bit */
	if (poe3->is_rzt2h) {
		if ((ch == MTU3_CHANNEL_0) || (ch == MTU3_CHANNEL_34) ||
		    (ch == MTU3_CHANNEL_67)) {
			val = renesas_poe3_read(poe3, poecr12_reg);
			if ((val & ICSR6_OSTSTF) != 0)
				renesas_poe3_write(poe3, poecr12_reg, (val & 0x0FFF));
		}
	}

	return 0;

error_ports:
	return -EINVAL;
}

static ssize_t mtu0_output_enable_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct renesas_poe3 *poe3 = platform_get_drvdata(pdev);
	unsigned int val1, val2;
	int ret;

	ret = kstrtouint(buf, 0, &val1);
	if (ret == 1)
		return ret;

	mutex_lock(&poe3->mutex);

	val2 = renesas_poe3_read(poe3, SPOER);
	if (val1 == 0)
		renesas_poe3_write(poe3, SPOER,
				  (val2 | SPOER_MTUCH0HIZ));
	else if (val1 == 1)
		ret = renesas_poe3_clear_Hi_z_state(poe3, MTU3_CHANNEL_0);

	mutex_unlock(&poe3->mutex);

	return ret ? : size;
}

static DEVICE_ATTR_WO(mtu0_output_enable);

static ssize_t mtu34_output_enable_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{

	struct platform_device *pdev = to_platform_device(dev);
	struct renesas_poe3 *poe3 = platform_get_drvdata(pdev);
	unsigned int val1, val2;
	int ret;

	ret = kstrtouint(buf, 0, &val1);
	if (ret == 1)
		return ret;

	mutex_lock(&poe3->mutex);

	val2 = renesas_poe3_read(poe3, SPOER);
	if (val1 == 0)
		renesas_poe3_write(poe3, SPOER,
				  (val2 | SPOER_MTUCH34HIZ));
	else if (val1 == 1)
		ret = renesas_poe3_clear_Hi_z_state(poe3, MTU3_CHANNEL_34);

	mutex_unlock(&poe3->mutex);

	return ret ? : size;
}

static DEVICE_ATTR_WO(mtu34_output_enable);

static ssize_t mtu67_output_enable_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{

	struct platform_device *pdev = to_platform_device(dev);
	struct renesas_poe3 *poe3 = platform_get_drvdata(pdev);
	unsigned int val1, val2;
	int ret;

	ret = kstrtouint(buf, 0, &val1);
	if (ret == 1)
		return ret;

	mutex_lock(&poe3->mutex);

	val2 = renesas_poe3_read(poe3, SPOER);
	if (val1 == 0)
		renesas_poe3_write(poe3, SPOER,
				  (val2 | SPOER_MTUCH67HIZ));
	else if (val1 == 1)
		ret = renesas_poe3_clear_Hi_z_state(poe3, MTU3_CHANNEL_67);

	mutex_unlock(&poe3->mutex);

	return ret ? : size;
}

static DEVICE_ATTR_WO(mtu67_output_enable);

static void renesas_poe3_setup(struct renesas_poe3 *poe3)
{
	struct device *dev = &poe3->pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *child;
	unsigned int poecr1_val, poecr2_val, poecr4_val, poecr5_val, val, val1;
	unsigned int poecr7_val, poecr8_val, poecr9_val, poecr10_val, poecr11_val, poecr12_val;
	unsigned int mtu0ab = 0, mtu0cd = 0, mtu3bd = 0, mtu4ac = 0, mtu4bd = 0;
	unsigned int mtu6bd = 0, mtu7ac = 0, mtu7bd = 0, alr1 = 0;
	u32 tmp, tmp1, num, ch;
	int i, ret;

	if (!of_get_property(np, "poe3_pins_mode", &tmp))
		goto poe3_assign;

	num = tmp/(sizeof(u32)*2);

	for (i = 0; i < num; i++) {
		of_property_read_u32_index(np, "poe3_pins_mode",
					   i*2, &ch);
		of_property_read_u32_index(np, "poe3_pins_mode",
					   i*2 + 1, &tmp);
		if (tmp < 4) {
			switch (ch) {
			case 0:
				val = renesas_poe3_read(poe3, ICSR1);
				val = (val | (ICSR_PIE | tmp)) & 0x0FFF;
				renesas_poe3_write(poe3, ICSR1, val);
				break;
			case 4:
				val = renesas_poe3_read(poe3, ICSR2);
				val = (val | (ICSR_PIE | tmp)) & 0x0FFF;
				renesas_poe3_write(poe3, ICSR2, val);
				break;
			case 8:
				val = renesas_poe3_read(poe3, ICSR3);
				val = (val | (ICSR_PIE | ICSR34_POEE | tmp)) &
					0x0FFF;
				renesas_poe3_write(poe3, ICSR3, val);
				break;
			case 10:
				val = renesas_poe3_read(poe3, ICSR4);
				val = (val | (ICSR_PIE | ICSR34_POEE | tmp)) &
					0x0FFF;
				renesas_poe3_write(poe3, ICSR4, val);
				break;
			case 11:
				val = renesas_poe3_read(poe3, ICSR4);
				val = (val | (ICSR_PIE | ICSR5_POEE | tmp)) &
					0x0FFF;
				renesas_poe3_write(poe3, ICSR5, val);
				break;
			default:
				dev_err(&poe3->pdev->dev, "Invalid pin POE%d: %d\n",
					ch, -EINVAL);
				break;
			}
		} else
			dev_err(&poe3->pdev->dev,
				"Invalid mode %d, must be smaller than 4: %d\n",
				tmp, -EINVAL);
	}

poe3_assign:
	poecr1_val = 0;
	poecr2_val = 0;
	poecr4_val = 0;
	poecr5_val = 0;
	poecr7_val = 0;
	poecr8_val = 0;
	poecr9_val = 0;
	poecr10_val = 0;
	poecr11_val = 0;
	poecr12_val = 0;

	if (poe3->is_rzt2h) {
		for (i = 0; i < num; i++) {
			of_property_read_u32_index(np, "oscillation_detection", i, &tmp);
			if (tmp == 1) {
				val1 = renesas_poe3_read(poe3, ICSR6);
				val1 = (val1 | ICSR6_OSTSTE) & 0x0FFF;
				renesas_poe3_write(poe3, ICSR6, val1);
			}
		}
	}

	for_each_child_of_node(np, child) {
		if (of_get_property(child, "mtu3_outputs", &tmp)) {
			num = tmp/sizeof(u32);
			for (i = 0; i < num; i++) {
				of_property_read_u32_index(child,
					"mtu3_outputs", i, &tmp);
				of_property_read_u32_index(child,
					"active_level", i, &tmp1);
				if (!strcmp(child->name, "mtu3_ch0") &&
				    (tmp < 4))
					poecr1_val |= (POECR1_MTU0AZE << tmp);
				else if (!strcmp(child->name, "mtu3_ch34") &&
					(tmp < 3)) {
					poecr2_val |= (POECR2_MTU3BDZE >> tmp);
					if ((poe3->is_rzt2h) && (tmp1 == 1)) {
						switch (tmp) {
						case 0:
							alr1 |= ALR1_OLSG0AB;
							break;
						case 1:
							alr1 |= ALR1_OLSG1AB;
							break;
						case 2:
							alr1 |= ALR1_OLSG2AB;
							break;
						}
					}
				} else if (!strcmp(child->name, "mtu3_ch67") &&
					(tmp < 3))
					poecr2_val |= (POECR2_MTU6BDZE >>  tmp);
			}

			if (poe3->is_rzt2h) {
				/* Select the MTIOC0A/B pins as targets for high-z control*/
				of_get_property(child, "mtu3_outputs", &tmp);
				num = tmp/sizeof(u32);
				for (i = 0; i < num; i++) {
					of_property_read_u32_index(child,
						"mtioc0a_pin_select", i, &tmp);
					of_property_read_u32_index(child,
						"mtu3_outputs", i, &tmp1);
					if (!strcmp(child->name, "mtu3_ch0") && (tmp < 3) &&
					   (tmp1 == 0)) {
						switch (tmp) {
						case 0:
							mtu0ab |= (M0SELR1_M0ASEL >> 1);
							break;
						case 1:
							mtu0ab |= M0SELR1_M0ASEL;
							break;
						case 2:
							mtu0ab |= (M0SELR1_M0ASEL << 1);
							break;
						}
					}
				}

				of_get_property(child, "mtu3_outputs", &tmp);
				num = tmp/sizeof(u32);
				for (i = 0; i < num; i++) {
					of_property_read_u32_index(child,
						"mtioc0b_pin_select", i, &tmp);
					of_property_read_u32_index(child,
						"mtu3_outputs", i, &tmp1);
					if (!strcmp(child->name, "mtu3_ch0") && (tmp < 3) &&
					   (tmp1 == 1)) {
						switch (tmp) {
						case 0:
							mtu0ab |= (M0SELR1_M0BSEL >> 1);
							break;
						case 1:
							mtu0ab |= M0SELR1_M0BSEL;
							break;
						case 2:
							mtu0ab |= (M0SELR1_M0BSEL << 1);
							break;
						}
					}
				}

				/* Select the MTIOC0C/D pins as targets for high-z control*/
				of_get_property(child, "mtu3_outputs", &tmp);
				num = tmp/sizeof(u32);
				for (i = 0; i < num; i++) {
					of_property_read_u32_index(child,
						"mtioc0c_pin_select", i, &tmp);
					of_property_read_u32_index(child,
						"mtu3_outputs", i, &tmp1);
					if (!strcmp(child->name, "mtu3_ch0") && (tmp < 2) &&
					   (tmp1 == 0)) {
						switch (tmp) {
						case 0:
							mtu0cd |= (M0SELR2_M0CSEL >> 1);
							break;
						case 1:
							mtu0cd |= M0SELR2_M0CSEL;
							break;
						}
					}
				}

				of_get_property(child, "mtu3_outputs", &tmp);
				num = tmp/sizeof(u32);
				for (i = 0; i < num; i++) {
					of_property_read_u32_index(child,
						"mtioc0d_pin_select", i, &tmp);
					of_property_read_u32_index(child,
						"mtu3_outputs", i, &tmp1);
					if (!strcmp(child->name, "mtu3_ch0") && (tmp < 2) &&
					   (tmp1 == 0)) {
						switch (tmp) {
						case 0:
							mtu0cd |= (M0SELR2_M0DSEL >> 5);
							break;
						case 1:
							mtu0cd |= M0SELR2_M0DSEL;
							break;
						}
					}
				}

				/* Select the MTIOC3B/D pins as targets for high-z control*/
				of_get_property(child, "mtu3_outputs", &tmp);
				num = tmp/sizeof(u32);
				for (i = 0; i < num; i++) {
					of_property_read_u32_index(child,
						"mtioc3b_pin_select", i, &tmp);
					of_property_read_u32_index(child,
						"mtu3_outputs", i, &tmp1);
					if (!strcmp(child->name, "mtu3_ch34") && (tmp < 2) &&
					   (tmp1 == 0)) {
						switch (tmp) {
						case 0:
							mtu3bd |= (M3SELR_M3BSEL >> 1);
							break;
						case 1:
							mtu3bd |= M3SELR_M3BSEL;
							break;
						}
					}
				}

				of_get_property(child, "mtu3_outputs", &tmp);
				num = tmp/sizeof(u32);
				for (i = 0; i < num; i++) {
					of_property_read_u32_index(child,
						"mtioc3d_pin_select", i, &tmp);
					of_property_read_u32_index(child,
						"mtu3_outputs", i, &tmp1);
					if (!strcmp(child->name, "mtu3_ch34") && (tmp < 2) &&
					   (tmp1 == 0)) {
						switch (tmp) {
						case 0:
							mtu3bd |= (M3SELR_M3DSEL >> 5);
							break;
						case 1:
							mtu3bd |= M3SELR_M3DSEL;
							break;
						}
					}
				}

				/* Select the MTIOC4A/C pins as targets for high-z control*/
				of_get_property(child, "mtu3_outputs", &tmp);
				num = tmp/sizeof(u32);
				for (i = 0; i < num; i++) {
					of_property_read_u32_index(child,
						"mtioc4a_pin_select", i, &tmp);
					of_property_read_u32_index(child,
						"mtu3_outputs", i, &tmp1);
					if (!strcmp(child->name, "mtu3_ch34") && (tmp < 2) &&
					   (tmp1 == 1)) {
						switch (tmp) {
						case 0:
							mtu4ac |= (M4SELR1_M4ASEL >> 1);
							break;
						case 1:
							mtu4ac |= M4SELR1_M4ASEL;
							break;
						}
					}
				}

				of_get_property(child, "mtu3_outputs", &tmp);
				num = tmp/sizeof(u32);
				for (i = 0; i < num; i++) {
					of_property_read_u32_index(child,
						"mtioc4c_pin_select", i, &tmp);
					of_property_read_u32_index(child,
						"mtu3_outputs", i, &tmp1);
					if (!strcmp(child->name, "mtu3_ch34") && (tmp < 2) &&
					   (tmp1 == 1)) {
						switch (tmp) {
						case 0:
							mtu4ac |= (M4SELR1_M4CSEL >> 5);
							break;
						case 1:
							mtu4ac |= M4SELR1_M4CSEL;
							break;
						}
					}
				}

				/* Select the MTIOC4B/D pins as targets for high-z control*/
				of_get_property(child, "mtu3_outputs", &tmp);
				num = tmp/sizeof(u32);
				for (i = 0; i < num; i++) {
					of_property_read_u32_index(child,
						"mtioc4b_pin_select", i, &tmp);
					of_property_read_u32_index(child,
						"mtu3_outputs", i, &tmp1);
					if (!strcmp(child->name, "mtu3_ch34") && (tmp < 2) &&
					   (tmp1 == 2)) {
						switch (tmp) {
						case 0:
							mtu4bd |= (M4SELR2_M4BSEL >> 1);
							break;
						case 1:
							mtu4bd |= M4SELR2_M4BSEL;
							break;
						}
					}
				}

				of_get_property(child, "mtu3_outputs", &tmp);
				num = tmp/sizeof(u32);
				for (i = 0; i < num; i++) {
					of_property_read_u32_index(child,
						"mtioc4d_pin_select", i, &tmp);
					of_property_read_u32_index(child,
						"mtu3_outputs", i, &tmp1);
					if (!strcmp(child->name, "mtu3_ch34") && (tmp < 2) &&
					   (tmp1 == 2)) {
						switch (tmp) {
						case 0:
							mtu4bd |= (M4SELR2_M4DSEL >> 5);
							break;
						case 1:
							mtu4bd |= M4SELR2_M4DSEL;
							break;
						}
					}
				}

				/* Select the MTIOC6B/D pins as targets for high-z control*/
				of_get_property(child, "mtu3_outputs", &tmp);
				num = tmp/sizeof(u32);
				for (i = 0; i < num; i++) {
					of_property_read_u32_index(child,
						"mtioc6b_pin_select", i, &tmp);
					of_property_read_u32_index(child,
						"mtu3_outputs", i, &tmp1);
					if (!strcmp(child->name, "mtu3_ch67") && (tmp < 3) &&
					   (tmp1 == 0)) {
						switch (tmp) {
						case 0:
							mtu6bd |= (M6SELR_M6BSEL >> 1);
							break;
						case 1:
							mtu6bd |= M6SELR_M6BSEL;
							break;
						case 2:
							mtu6bd |= (M6SELR_M6BSEL << 1);
							break;
						}
					}
				}

				of_get_property(child, "mtu3_outputs", &tmp);
				num = tmp/sizeof(u32);
				for (i = 0; i < num; i++) {
					of_property_read_u32_index(child,
						"mtioc6d_pin_select", i, &tmp);
					of_property_read_u32_index(child,
						"mtu3_outputs", i, &tmp1);
					if (!strcmp(child->name, "mtu3_ch67") && (tmp < 3) &&
					   (tmp1 == 0)) {
						switch (tmp) {
						case 0:
							mtu6bd |= (M6SELR_M6DSEL >> 5);
							break;
						case 1:
							mtu6bd |= M6SELR_M6DSEL;
							break;
						case 2:
							mtu6bd |= (M6SELR_M6DSEL << 1);
							break;
						}
					}
				}

				/* Select the MTIOC7A/C pins as targets for high-z control*/
				of_get_property(child, "mtu3_outputs", &tmp);
				num = tmp/sizeof(u32);
				for (i = 0; i < num; i++) {
					of_property_read_u32_index(child,
						"mtioc7a_pin_select", i, &tmp);
					of_property_read_u32_index(child,
						"mtu3_outputs", i, &tmp1);
					if (!strcmp(child->name, "mtu3_ch67") && (tmp < 3) &&
					   (tmp1 == 1)) {
						switch (tmp) {
						case 0:
							mtu7ac |= (M7SELR1_M7ASEL >> 1);
							break;
						case 1:
							mtu7ac |= M7SELR1_M7ASEL;
							break;
						case 2:
							mtu7ac |= (M7SELR1_M7ASEL << 1);
							break;
						}
					}
				}

				of_get_property(child, "mtu3_outputs", &tmp);
				num = tmp/sizeof(u32);
				for (i = 0; i < num; i++) {
					of_property_read_u32_index(child,
						"mtioc7c_pin_select", i, &tmp);
					of_property_read_u32_index(child,
						"mtu3_outputs", i, &tmp1);
					if (!strcmp(child->name, "mtu3_ch67") && (tmp < 3) &&
					   (tmp1 == 1)) {
						switch (tmp) {
						case 0:
							mtu7ac |= (M7SELR1_M7CSEL >> 5);
							break;
						case 1:
							mtu7ac |= M7SELR1_M7CSEL;
							break;
						case 2:
							mtu7ac |= (M7SELR1_M7CSEL << 1);
							break;
						}
					}
				}

				/* Select the MTIOC7B/D pins as targets for high-z control*/
				of_get_property(child, "mtu3_outputs", &tmp);
				num = tmp/sizeof(u32);
				for (i = 0; i < num; i++) {
					of_property_read_u32_index(child,
						"mtioc7b_pin_select", i, &tmp);
					of_property_read_u32_index(child,
						"mtu3_outputs", i, &tmp1);
					if (!strcmp(child->name, "mtu3_ch67") && (tmp < 3) &&
					   (tmp1 == 2)) {
						switch (tmp) {
						case 0:
							mtu7bd |= (M7SELR2_M7BSEL >> 1);
							break;
						case 1:
							mtu7bd |= M7SELR2_M7BSEL;
							break;
						case 2:
							mtu7bd |= (M7SELR2_M7BSEL << 1);
							break;
						}
					}
				}

				of_get_property(child, "mtu3_outputs", &tmp);
				num = tmp/sizeof(u32);
				for (i = 0; i < num; i++) {
					of_property_read_u32_index(child,
						"mtioc7d_pin_select", i, &tmp);
					of_property_read_u32_index(child,
						"mtu3_outputs", i, &tmp1);
					if (!strcmp(child->name, "mtu3_ch67") && (tmp < 3) &&
					   (tmp1 == 2)) {
						switch (tmp) {
						case 0:
							mtu7bd |= (M7SELR2_M7DSEL >> 5);
							break;
						case 1:
							mtu7bd |= M7SELR2_M7DSEL;
							break;
						case 2:
							mtu7bd |= (M7SELR2_M7DSEL << 1);
							break;
						}
					}
				}
			}

			of_get_property(child, "addition_poe3_inputs", &tmp);
			num = tmp/sizeof(u32);
			for (i = 0; i < num; i++) {
				of_property_read_u32_index(child,
					"addition_poe3_inputs", i, &tmp);
				if (!strcmp(child->name, "mtu3_ch0")) {
					switch (tmp) {
					case 0:
						poecr5_val |= POECR5_IC1ADDMT0;
						break;
					case 4:
						poecr5_val |= POECR5_IC2ADDMT0;
						break;
					case 10:
						poecr5_val |= POECR5_IC4ADDMT0;
						break;
					case 11:
						poecr5_val |= POECR5_IC5ADDMT0;
						break;
					}
				} else if (!strcmp(child->name, "mtu3_ch34")) {
					switch (tmp) {
					case 4:
						poecr4_val |= POECR4_IC2ADDMT34;
						break;
					case 8:
						poecr4_val |= POECR4_IC3ADDMT34;
						break;
					case 10:
						poecr4_val |= POECR4_IC4ADDMT34;
						break;
					case 11:
						poecr4_val |= POECR4_IC5ADDMT34;
						break;
					}
				} else if (!strcmp(child->name, "mtu3_ch67")) {
					switch (tmp) {
					case 0:
						poecr4_val |= POECR4_IC1ADDMT67;
						break;
					case 8:
						poecr4_val |= POECR4_IC3ADDMT67;
						break;
					case 10:
						poecr4_val |= POECR4_IC4ADDMT67;
						break;
					case 11:
						poecr4_val |= POECR4_IC5ADDMT67;
						break;
					}
				}
			}

			if (poe3->is_rzt2h) {
				of_get_property(child, "dsmif_error_detection", &tmp);
				num = tmp/sizeof(u32);
				for (i = 0; i < num; i++) {
					of_property_read_u32_index(child,
						"dsmif_error_detection", i*2, &tmp);
					of_property_read_u32_index(child,
						"dsmif_error_detection", i*2 + 1, &tmp1);
					if (tmp == 0) {
						val = renesas_poe3_read(poe3, ICSR9);
						val = (val | (ICSR9_D0ERR0IE << tmp1));
						renesas_poe3_write(poe3, ICSR9, val);
						if (!strcmp(child->name, "mtu3_ch0") &&
						   (tmp1 < 10))
							poecr7_val |= (POECR7_D0E0ADDMT0
									<< tmp1);
						else if (!strcmp(child->name, "mtu3_ch34") &&
						   (tmp1 < 10))
							poecr9_val |= (POECR9_D0E0ADDMT34
									<< tmp1);
						else if (!strcmp(child->name, "mtu3_ch67") &&
						   (tmp1 < 10))
							poecr11_val |= (POECR11_D0E0ADDMT67
									<< tmp1);
					} else if (tmp == 1) {
						val = renesas_poe3_read(poe3, ICSR10);
						val = (val | (ICSR10_D0ERR1IE << tmp1));
						renesas_poe3_write(poe3, ICSR10, val);
						if (!strcmp(child->name, "mtu3_ch0") &&
						   (tmp1 < 10))
							poecr8_val |= (POECR8_D0E1ADDMT0
									<< tmp1);
						else if (!strcmp(child->name, "mtu3_ch34") &&
						   (tmp1 < 10))
							poecr10_val |= (POECR10_D0E1ADDMT34
									<< tmp1);
						else if (!strcmp(child->name, "mtu3_ch67") &&
						   (tmp1 < 10))
							poecr12_val |= (POECR12_D0E1ADDMT67
									<< tmp1);
					}
				}
			}
		}

		if (!strcmp(child->name, "mtu3_ch0"))
			ret = device_create_file(&poe3->pdev->dev,
					&dev_attr_mtu0_output_enable);
		else if (!strcmp(child->name, "mtu3_ch34")) {
			renesas_poe3_write(poe3, OCSR1, OCSR_OCE | OCSR_OIE);
			ret = device_create_file(&poe3->pdev->dev,
					&dev_attr_mtu34_output_enable);
		} else if (!strcmp(child->name, "mtu3_ch67")) {
			renesas_poe3_write(poe3, OCSR2, OCSR_OCE | OCSR_OIE);
			ret = device_create_file(&poe3->pdev->dev,
					&dev_attr_mtu67_output_enable);
		} else
			ret = 0;

		if (ret < 0)
			dev_err(&poe3->pdev->dev, "Failed to create poe3 sysfs for %s\n",
				child->name);
	}

	renesas_poe3_write(poe3, POECR1, poecr1_val);
	renesas_poe3_write(poe3, POECR2, poecr2_val);
	renesas_poe3_write(poe3, POECR4, poecr4_val);
	renesas_poe3_write(poe3, POECR5, poecr5_val);
	if (poe3->is_rzt2h) {
		renesas_poe3_write(poe3, POECR7, poecr7_val);
		renesas_poe3_write(poe3, POECR8, poecr8_val);
		renesas_poe3_write(poe3, POECR9, poecr9_val);
		renesas_poe3_write(poe3, POECR10, poecr10_val);
		renesas_poe3_write(poe3, POECR11, poecr11_val);
		renesas_poe3_write(poe3, POECR12, poecr12_val);
		renesas_poe3_write(poe3, ALR1, alr1 | ALR1_OLSEN);
		renesas_poe3_write(poe3, M0SELR1, mtu0ab);
		renesas_poe3_write(poe3, M0SELR2, mtu0cd);
		renesas_poe3_write(poe3, M3SELR, mtu3bd);
		renesas_poe3_write(poe3, M4SELR1, mtu4ac);
		renesas_poe3_write(poe3, M4SELR2, mtu4bd);
		renesas_poe3_write(poe3, M6SELR, mtu6bd);
		renesas_poe3_write(poe3, M7SELR1, mtu7ac);
		renesas_poe3_write(poe3, M7SELR2, mtu7bd);
	}
}

static int renesas_poe3_probe(struct platform_device *pdev)
{
	struct renesas_poe3 *poe3;
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	int ret;

	poe3 = devm_kzalloc(&pdev->dev, sizeof(*poe3), GFP_KERNEL);
	if (poe3 == NULL)
		return -ENOMEM;

	poe3->pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	poe3->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(poe3->base))
		return PTR_ERR(poe3->base);

	if (of_device_is_compatible(np, "renesas,r9a09g077-poe3"))
		poe3->is_rzt2h = true;
	else
		poe3->is_rzt2h = false;

	if (!poe3->is_rzt2h) {
		poe3->rstc = devm_reset_control_get(&pdev->dev, NULL);
		if (IS_ERR(poe3->rstc)) {
			dev_err(&pdev->dev, "failed to get reset control\n");
			return PTR_ERR(poe3->rstc);
		}
		reset_control_deassert(poe3->rstc);
	}

	poe3->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(poe3->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		return PTR_ERR(poe3->clk);
	}

	ret = clk_prepare_enable(poe3->clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable clock\n");
		return ret;
	}

	renesas_poe3_setup(poe3);

	platform_set_drvdata(pdev, poe3);
	dev_info(&pdev->dev, "Renesas POE3 driver probed\n");
	return 0;
}

static int renesas_poe3_remove(struct platform_device *pdev)
{
	struct renesas_poe3 *poe = platform_get_drvdata(pdev);

	clk_disable_unprepare(poe->clk);
	return 0;
}

static const struct of_device_id renesas_poe3_of_table[] = {
	{ .compatible = "renesas,poe3", },
	{ .compatible = "renesas,rz-poe3", },
	{ .compatible = "renesas,r9a09g077-poe3", },
	{ },
};

MODULE_DEVICE_TABLE(of, poe3_of_table);

static struct platform_driver renesas_poe3_device_driver = {
	.probe		= renesas_poe3_probe,
	.remove		= renesas_poe3_remove,
	.driver		= {
		.name	= "renesas_poe3",
		.of_match_table = of_match_ptr(renesas_poe3_of_table),
	}
};

module_platform_driver(renesas_poe3_device_driver);

MODULE_DESCRIPTION("Renesas POE3 Driver");
MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_LICENSE("GPL v2");
