/* SPDX-License-Identifier: GPL-2.0 */
/*
 * RZ/G2L Clock Pulse Generator
 *
 * Copyright (C) 2021 Renesas Electronics Corp.
 *
 */

#ifndef __RENESAS_RZG2L_CPG_H__
#define __RENESAS_RZG2L_CPG_H__

#define CPG_PL1_DDIV		(0x200)
#define CPG_PL2_DDIV		(0x204)
#define CPG_PL3A_DDIV		(0x208)
#define CPG_PL3B_DDIV		(0x20C)
#define CPG_PL6_DDIV		(0x210)
#define CPG_PL2SDHI_DSEL	(0x218)
#define CPG_CLKSTATUS		(0x280)
#define CPG_PL3_SSEL		(0x408)
#define CPG_PL6_SSEL		(0x414)
#define CPG_PL6_ETH_SSEL	(0x418)
#define CPG_PL5_SDIV		(0x420)
#define CPG_OTHERFUNC1_REG	(0xBE8)

#define ACPU_MSTOP		(0xB60)
#define MCPU1_MSTOP		(0xB64)
#define MCPU2_MSTOP		(0xB68)
#define PERI_COM_MSTOP		(0xB6C)
#define PERI_CPU_MSTOP		(0xB70)
#define PERI_DDR_MSTOP		(0xB74)
#define PERI_VIDEO_MSTOP	(0xB78)
#define REG0_MSTOP		(0xB7C)
#define REG1_MSTOP		(0xB80)
#define TZCDDR_MSTOP		(0xB84)
#define MHU_MSTOP		(0xB88)
#define PERI_STP_MSTOP		(0xB8C)

#define CPG_CLKSTATUS_SELSDHI0_STS	BIT(28)
#define CPG_CLKSTATUS_SELSDHI1_STS	BIT(29)

#define CPG_SDHI_CLK_SWITCH_STATUS_TIMEOUT_US	20000

/* n = 0/1/2 for PLL1/4/6 */
#define CPG_SAMPLL_CLK1(n)	(0x04 + (16 * n))
#define CPG_SAMPLL_CLK2(n)	(0x08 + (16 * n))

#define PLL146_CONF(n)	(CPG_SAMPLL_CLK1(n) << 22 | CPG_SAMPLL_CLK2(n) << 12)

/* Registers only for RZ/V2H */
#define CPG_SSEL0		(0x300)
#define CPG_SSEL1		(0x304)
#define CPG_SSEL2		(0x308)
#define CPG_CDDIV0		(0x400)
#define CPG_CDDIV1		(0x404)
#define CPG_CDDIV2		(0x408)
#define CPG_CDDIV3		(0x40C)
#define CPG_CDDIV4		(0x410)
#define CPG_CSDIV0		(0x500)
#define CPG_CSDIV1		(0x504)


#define DDIV_PACK(offset, bitpos, size) \
		(((offset) << 20) | ((bitpos) << 12) | ((size) << 8))
#define DIVPL1A		DDIV_PACK(CPG_PL1_DDIV, 0, 2)
#define DIVPL2A		DDIV_PACK(CPG_PL2_DDIV, 0, 3)
#define DIVDSILPCLK	DDIV_PACK(CPG_PL2_DDIV, 12, 2)
#define DIVPL3A		DDIV_PACK(CPG_PL3A_DDIV, 0, 3)
#define DIVPL3B		DDIV_PACK(CPG_PL3A_DDIV, 4, 3)
#define DIVPL3C		DDIV_PACK(CPG_PL3A_DDIV, 8, 3)
#define DIVPL3CLK200FIX	DDIV_PACK(CPG_PL3B_DDIV, 0, 3)
#define DIVGPU		DDIV_PACK(CPG_PL6_DDIV, 0, 2)
#define DIVDSIA		DDIV_PACK(CPG_PL5_SDIV, 0, 2)
#define DIVDSIB		DDIV_PACK(CPG_PL5_SDIV, 8, 4)

/* Divider only for RZ/V2H */
#define CDDIV0_DIVCTL0	DDIV_PACK(CPG_CDDIV0,  0, 3)
#define CDDIV0_DIVCTL1	DDIV_PACK(CPG_CDDIV0,  4, 3)
#define CDDIV0_DIVCTL2	DDIV_PACK(CPG_CDDIV0,  8, 3)
#define CDDIV0_DIVCTL3	DDIV_PACK(CPG_CDDIV0, 12, 3)
#define CDDIV1_DIVCTL0	DDIV_PACK(CPG_CDDIV1,  0, 2)
#define CDDIV1_DIVCTL1	DDIV_PACK(CPG_CDDIV1,  4, 2)
#define CDDIV1_DIVCTL2	DDIV_PACK(CPG_CDDIV1,  8, 2)
#define CDDIV1_DIVCTL3	DDIV_PACK(CPG_CDDIV1, 12, 2)
#define CDDIV2_DIVCTL0	DDIV_PACK(CPG_CDDIV1,  0, 3)
#define CDDIV2_DIVCTL1	DDIV_PACK(CPG_CDDIV1,  4, 3)
#define CDDIV2_DIVCTL2	DDIV_PACK(CPG_CDDIV1,  8, 3)
#define CDDIV2_DIVCTL3	DDIV_PACK(CPG_CDDIV1, 12, 3)
#define CDDIV3_DIVCTL0	DDIV_PACK(CPG_CDDIV3,  0, 3)
#define CDDIV3_DIVCTL1	DDIV_PACK(CPG_CDDIV3,  4, 3)
#define CDDIV3_DIVCTL2	DDIV_PACK(CPG_CDDIV3,  8, 3)
#define CDDIV3_DIVCTL3	DDIV_PACK(CPG_CDDIV3, 12, 1)
#define CDDIV4_DIVCTL0	DDIV_PACK(CPG_CDDIV4,  0, 1)
#define CDDIV4_DIVCTL1	DDIV_PACK(CPG_CDDIV4,  4, 1)
#define CDDIV4_DIVCTL2	DDIV_PACK(CPG_CDDIV4,  8, 1)
#define CSDIV0_DIVCTL0	DDIV_PACK(CPG_CSDIV0,  0, 2)
#define CSDIV0_DIVCTL1	DDIV_PACK(CPG_CSDIV0,  4, 2)
#define CSDIV0_DIVCTL2	DDIV_PACK(CPG_CSDIV0,  8, 2)
#define CSDIV0_DIVCTL3	DDIV_PACK(CPG_CSDIV0, 12, 2)
#define CSDIV1_DIVCTL0	DDIV_PACK(CPG_CSDIV1,  0, 1)
#define CSDIV1_DIVCTL1	DDIV_PACK(CPG_CSDIV1,  4, 2)
#define CSDIV1_DIVCTL2	DDIV_PACK(CPG_CSDIV1,  8, 4)

#define SEL_PLL_PACK(offset, bitpos, size) \
		(((offset) << 20) | ((bitpos) << 12) | ((size) << 8))

#define SEL_PLL3_3	SEL_PLL_PACK(CPG_PL3_SSEL, 8, 1)
#define SEL_PLL5_4	SEL_PLL_PACK(CPG_OTHERFUNC1_REG, 0, 1)
#define SEL_PLL6_2	SEL_PLL_PACK(CPG_PL6_ETH_SSEL, 0, 1)
#define SEL_GPU2	SEL_PLL_PACK(CPG_PL6_SSEL, 12, 1)

#define SEL_SDHI0	DDIV_PACK(CPG_PL2SDHI_DSEL, 0, 2)
#define SEL_SDHI1	DDIV_PACK(CPG_PL2SDHI_DSEL, 4, 2)

/* Clock selection only for RZ/V2H */
#define SSEL0_SELCTL0	SEL_PLL_PACK(CPG_SSEL0,  0, 1)
#define SSEL0_SELCTL1	SEL_PLL_PACK(CPG_SSEL0,  4, 1)
#define SSEL0_SELCTL2	SEL_PLL_PACK(CPG_SSEL0,  8, 1)
#define SSEL0_SELCTL3	SEL_PLL_PACK(CPG_SSEL0, 12, 1)
#define SSEL1_SELCTL0	SEL_PLL_PACK(CPG_SSEL1,  0, 1)
#define SSEL1_SELCTL1	SEL_PLL_PACK(CPG_SSEL1,  4, 1)
#define SSEL1_SELCTL2	SEL_PLL_PACK(CPG_SSEL1,  8, 1)
#define SSEL1_SELCTL3	SEL_PLL_PACK(CPG_SSEL1, 12, 1)

#define MSTOP(off, bit)	((off & 0xffff) << 16 | bit)
#define MSTOP_OFF(val)	((val >> 16) & 0xffff)
#define MSTOP_BIT(val)	(val & 0xffff)

/**
 * Definitions of CPG Core Clocks
 *
 * These include:
 *   - Clock outputs exported to DT
 *   - External input clocks
 *   - Internal CPG clocks
 */
struct cpg_core_clk {
	const char *name;
	unsigned int id;
	unsigned int parent;
	unsigned int div;
	unsigned int mult;
	unsigned int type;
	unsigned int conf;
	unsigned int conf_a;
	unsigned int conf_b;
	const struct clk_div_table *dtable;
	const struct clk_div_table *dtable_a;
	const struct clk_div_table *dtable_b;
	const char * const *parent_names;
	int flag;
	int mux_flags;
	int num_parents;
};

enum clk_types {
	/* Generic */
	CLK_TYPE_IN,		/* External Clock Input */
	CLK_TYPE_FF,		/* Fixed Factor Clock */
	CLK_TYPE_SAM_PLL,
	CLK_TYPE_RZV2H_SAM_PLL,

	/* Clock with divider */
	CLK_TYPE_DIV,
	CLK_TYPE_2DIV,

	/* Clock with clock source selector */
	CLK_TYPE_MUX,

	/* Clock with SD clock source selector */
	CLK_TYPE_SD_MUX,
};

#define DEF_TYPE(_name, _id, _type...) \
	{ .name = _name, .id = _id, .type = _type }
#define DEF_BASE(_name, _id, _type, _parent...) \
	DEF_TYPE(_name, _id, _type, .parent = _parent)
#define DEF_SAMPLL(_name, _id, _parent, _conf) \
	DEF_TYPE(_name, _id, CLK_TYPE_SAM_PLL, .parent = _parent, .conf = _conf)
#define DEF_RZV2H_SAMPLL(_name, _id, _parent, _conf) \
	DEF_TYPE(_name, _id, CLK_TYPE_RZV2H_SAM_PLL, .parent = _parent, .conf = _conf)
#define DEF_INPUT(_name, _id) \
	DEF_TYPE(_name, _id, CLK_TYPE_IN)
#define DEF_FIXED(_name, _id, _parent, _mult, _div) \
	DEF_BASE(_name, _id, CLK_TYPE_FF, _parent, .div = _div, .mult = _mult)
#define DEF_DIV(_name, _id, _parent, _conf, _dtable, _flag) \
	DEF_TYPE(_name, _id, CLK_TYPE_DIV, .conf = _conf, \
		 .parent = _parent, .dtable = _dtable, .flag = _flag)
#define DEF_2DIV(_name, _id, _parent, _conf_a, _conf_b, _dtable_a, _dtable_b, _flag) \
	DEF_TYPE(_name, _id, CLK_TYPE_2DIV, .parent = _parent, \
		.conf_a = _conf_a, .conf_b = _conf_b, \
		.dtable_a = _dtable_a, .dtable_b = _dtable_b, .flag = _flag)
#define DEF_MUX(_name, _id, _conf, _parent_names, _num_parents, _flag, \
		_mux_flags) \
	DEF_TYPE(_name, _id, CLK_TYPE_MUX, .conf = _conf, \
		 .parent_names = _parent_names, .num_parents = _num_parents, \
		 .flag = _flag, .mux_flags = _mux_flags)
#define DEF_SD_MUX(_name, _id, _conf, _parent_names, _num_parents) \
	DEF_TYPE(_name, _id, CLK_TYPE_SD_MUX, .conf = _conf, \
		 .parent_names = _parent_names, .num_parents = _num_parents)

/**
 * struct rzg2l_mod_clk - Module Clocks definitions
 *
 * @name: handle between common and hardware-specific interfaces
 * @id: clock index in array containing all Core and Module Clocks
 * @parent: id of parent clock
 * @off: register offset
 * @bit: ON/MON bit
 * @is_coupled: flag to indicate coupled clock
 */
struct rzg2l_mod_clk {
	const char *name;
	unsigned int id;
	unsigned int parent;
	u16 off;
	u8 bit;
	u32 mstop;
	bool is_coupled;
};

#define DEF_MOD_BASE(_name, _id, _parent, _off, _bit, _mstop, _is_coupled)	\
	{ \
		.name = _name, \
		.id = MOD_CLK_BASE + (_id), \
		.parent = (_parent), \
		.off = (_off), \
		.bit = (_bit), \
		.mstop = (_mstop), \
		.is_coupled = (_is_coupled), \
	}


#define DEF_MOD(_name, _id, _parent, _off, _bit, _mstop)	\
	DEF_MOD_BASE(_name, _id, _parent, _off, _bit, _mstop, false)

#define DEF_COUPLED(_name, _id, _parent, _off, _bit, _mstop)	\
	DEF_MOD_BASE(_name, _id, _parent, _off, _bit, _mstop, true)

/**
 * struct rzg2l_reset - Reset definitions
 *
 * @off: register offset
 * @bit: reset bit
 */
struct rzg2l_reset {
	u16 off;
	u8 bit;
};

#define DEF_RST(_id, _off, _bit)	\
	[_id] = { \
		.off = (_off), \
		.bit = (_bit) \
	}

/**
 * struct clk_mon - Clks Monitoring
 *
 * @clk_off: clk monitoring register offset
 * @clk_bit: clk monitoring bit
 */
struct clk_mon {
	s16 clk_off;
	u8 clk_bit;
};

#define DEF_CLK_MON(_id, _clk_off, _clk_bit)        \
        [_id] = { \
                .clk_off = (_clk_off), \
                .clk_bit = (_clk_bit) \
        }

/**
 * struct rst_mon - Resets Monitoring
 *
 * @rst_off: reset monitoring register offset
 * @rst_bit: reset monitoring bit
 */
struct rst_mon {
	u16 rst_off;
	u8 rst_bit;
};

#define DEF_RST_MON(_id, _rst_off, _rst_bit)        \
        [_id] = { \
                .rst_off = (_rst_off), \
                .rst_bit = (_rst_bit) \
        }

/**
 * struct rzg2l_cpg_info - SoC-specific CPG Description
 *
 * @core_clks: Array of Core Clock definitions
 * @num_core_clks: Number of entries in core_clks[]
 * @last_dt_core_clk: ID of the last Core Clock exported to DT
 * @num_total_core_clks: Total number of Core Clocks (exported + internal)
 *
 * @mod_clks: Array of Module Clock definitions
 * @num_mod_clks: Number of entries in mod_clks[]
 * @num_hw_mod_clks: Number of Module Clocks supported by the hardware
 *
 * @resets: Array of Module Reset definitions
 * @num_resets: Number of entries in resets[]
 *
 * @crit_mod_clks: Array with Module Clock IDs of critical clocks that
 *                 should not be disabled without a knowledgeable driver
 * @num_crit_mod_clks: Number of entries in crit_mod_clks[]
 *
 * @clk_mon: Array of Module Clocks Monitoring definitions
 * @num_clk_mon: Number of entries in clk_mon[]
 *
 * @rst_mon: Array of Module Resets Monitoring definitions
 * @num_rst_mon: Number of entries in rst_mon[]
 */
struct rzg2l_cpg_info {
	/* Core Clocks */
	const struct cpg_core_clk *core_clks;
	unsigned int num_core_clks;
	unsigned int last_dt_core_clk;
	unsigned int num_total_core_clks;

	/* Module Clocks */
	const struct rzg2l_mod_clk *mod_clks;
	unsigned int num_mod_clks;
	unsigned int num_hw_mod_clks;

	/* Resets */
	const struct rzg2l_reset *resets;
	unsigned int num_resets;

	/* Critical Module Clocks that should not be disabled */
	const unsigned int *crit_mod_clks;
	unsigned int num_crit_mod_clks;

	/* Clocks Monitoring */
	const struct clk_mon *clk_mons;
	unsigned int num_clk_mon;

	/* Resets Monitoring */
	const struct rst_mon *rst_mons;
	unsigned int num_rst_mon;
};

extern const struct rzg2l_cpg_info r9a07g043_cpg_info;
extern const struct rzg2l_cpg_info r9a07g043f_cpg_info;
extern const struct rzg2l_cpg_info r9a07g044_cpg_info;
extern const struct rzg2l_cpg_info r9a07g054_cpg_info;
extern const struct rzg2l_cpg_info r9a09g057_cpg_info;

#endif
