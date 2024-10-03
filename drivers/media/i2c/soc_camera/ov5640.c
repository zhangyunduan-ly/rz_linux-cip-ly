/*
 * Copyright (c) 2015 iWave Systems Technologies Pvt. Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/module.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>
#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>


/*
 * window size list
 */
#define QVGA_WIDTH	320
#define QVGA_HEIGHT	240
#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define HD_WIDTH	1280
#define HD_HEIGHT	720
#define FHD_WIDTH	1920
#define FHD_HEIGHT	1080
#define MAX_WIDTH   	HD_WIDTH
#define MAX_HEIGHT  	HD_HEIGHT

/*
 * ID
 */
#define OV5640  0x5640
#define VERSION(pid, ver) ((pid<<8)|(ver&0xFF))

static int debug;
module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "Debug level (0-2)");


/*
 * struct
 */
struct regval_list {
	u16 reg_num;
	unsigned char value;
};

struct ov5640_color_format {
	u32 code;
	enum v4l2_colorspace colorspace;
};

static const struct v4l2_frmsize_discrete ov5640_frmsizes[] = {
	{QVGA_WIDTH, QVGA_HEIGHT},
	{VGA_WIDTH, VGA_HEIGHT},
	{HD_WIDTH, HD_HEIGHT},
	{FHD_WIDTH, FHD_HEIGHT},
};

struct ov5640_win_size {
	char                     *name;
	u32                     width;
	u32                     height;
	const struct regval_list *regs;
};

struct ov5640 {
	struct v4l2_subdev                sd;
	const struct ov5640_color_format *cfmt;
	const struct ov5640_win_size     *win;
	int                               model;
	unsigned short                    vflip:1;
	unsigned short                    hflip:1;
	/* band_filter = COM8[5] ? 256 - BDBASE : 0 */
	unsigned short                    band_filter;
	struct v4l2_ctrl_handler          hdl;
	struct soc_camera_subdev_desc	  ssdd_dt;
};

#define ENDMARKER { 0xff, 0xff }

/*
 * register setting for window size
 */
static const struct regval_list ov5640_qvga_regs[] = {
	{0x3008, 0x82},
	{0x4730, 0x01},
	{0x3103, 0x03}, {0x3017, 0xff}, {0x3018, 0xff},
	{0x3034, 0x1a}, {0x3037, 0x13}, {0x3108, 0x01},
	{0x3630, 0x36}, {0x3631, 0x0e}, {0x3632, 0xe2},
	{0x3633, 0x12}, {0x3621, 0xe0}, {0x3704, 0xa0},
	{0x3703, 0x5a}, {0x3715, 0x78}, {0x3717, 0x01},
	{0x370b, 0x60}, {0x3705, 0x1a}, {0x3905, 0x02},
	{0x3906, 0x10}, {0x3901, 0x0a}, {0x3731, 0x12},
	{0x3600, 0x08}, {0x3601, 0x33}, {0x302d, 0x60},
	{0x3620, 0x52}, {0x371b, 0x20}, {0x471c, 0x50},
	{0x3a13, 0x43}, {0x3a18, 0x00}, {0x3a19, 0x7c},
	{0x3635, 0x13}, {0x3636, 0x03}, {0x3634, 0x40},
	{0x3622, 0x01}, {0x3c01, 0x34}, {0x3c04, 0x28},
	{0x3c05, 0x98}, {0x3c06, 0x00}, {0x3c07, 0x08},
	{0x3c08, 0x00}, {0x3c09, 0x1c}, {0x3c0a, 0x9c},
	{0x3c0b, 0x40}, {0x3810, 0x00}, {0x3811, 0x10},
	{0x3812, 0x00}, {0x3708, 0x64}, {0x4001, 0x02},
	{0x4005, 0x1a}, {0x3000, 0x00}, {0x3004, 0xff},
	{0x300e, 0x58}, //Bit[2]:mipi_en 0:DVP enable 1:MIPI enable
	{0x302e, 0x00}, {0x4300, 0x32},
	{0x501f, 0x00}, //ISP format control registers - 000: ISP YUV422
	{0x440e, 0x00}, {0x5000, 0xa7},
	{0x3008, 0x02},
	{0x4730, 0x01},

	{0x3c07, 0x08}, {0x3820, 0x41}, {0x3821, 0x07},
	{0x3814, 0x31}, {0x3815, 0x31}, {0x3800, 0x00},
	{0x3801, 0x00}, {0x3802, 0x00}, {0x3803, 0x04},
	{0x3804, 0x0a}, {0x3805, 0x3f}, {0x3806, 0x07},
	{0x3807, 0x9b}, {0x3808, 0x01}, {0x3809, 0x40},
	{0x380a, 0x00}, {0x380b, 0xf0}, {0x380c, 0x07},
	{0x380d, 0x68}, {0x380e, 0x03}, {0x380f, 0xd8},
	{0x3813, 0x06}, {0x3618, 0x00}, {0x3612, 0x29},
	{0x3709, 0x52}, {0x370c, 0x03}, {0x3a02, 0x0b},
	{0x3a03, 0x88}, {0x3a14, 0x0b}, {0x3a15, 0x88},
	{0x4004, 0x02}, {0x3002, 0x1c}, {0x3006, 0xc3},
	{0x4713, 0x03}, {0x4407, 0x04}, {0x460b, 0x35},
	{0x460c, 0x22}, {0x4837, 0x22}, {0x3824, 0x02},
	{0x5001, 0xa3}, {0x3034, 0x1a}, {0x3035, 0x11},
	{0x3036, 0x46}, {0x3037, 0x13},

	ENDMARKER,
};

//ov5640_setting_30fps_VGA_640x480
static const struct regval_list ov5640_vga_regs[] = {
	{0x3103,0x11}, {0x3008,0x82}, {0x4730,0x01},
	{0x3008,0x42}, {0x4730,0x01}, {0x3103,0x03},
	{0x3017,0xff}, {0x3018,0xff}, {0x3034,0x1a},
	{0x3037,0x13}, {0x3108,0x01}, {0x3630,0x36},
	{0x3631,0x0e}, {0x3632,0xe2}, {0x3633,0x12},
	{0x3621,0xe0}, {0x3704,0xa0}, {0x3703,0x5a},
	{0x3715,0x78}, {0x3717,0x01}, {0x370b,0x60},
	{0x3705,0x1a}, {0x3905,0x02}, {0x3906,0x10},
	{0x3901,0x0a}, {0x3731,0x12}, {0x3600,0x08},
	{0x3601,0x33}, {0x302d,0x60}, {0x3620,0x52},
	{0x371b,0x20}, {0x471c,0x50}, {0x3a13,0x43},
	{0x3a18,0x00}, {0x3a19,0x7c}, {0x3635,0x13},
	{0x3636,0x03}, {0x3634,0x40}, {0x3622,0x01},
	{0x3c01,0x34}, {0x3c04,0x28}, {0x3c05,0x98},
	{0x3c06,0x00}, {0x3c07,0x07}, {0x3c08,0x00},
	{0x3c09,0x1c}, {0x3c0a,0x9c}, {0x3c0b,0x40},
	{0x3810,0x00}, {0x3811,0x10}, {0x3812,0x00},
	{0x3708,0x64}, {0x4001,0x02}, {0x4005,0x1a},
	{0x3000,0x00}, {0x3004,0xff}, {0x300e,0x58},
	{0x302e,0x00}, {0x4300,0x32}, {0x501f,0x00},
	{0x440e,0x00}, {0x5000,0xa7},
	{0x3008,0x02},
	{0x4730,0x01},
	{0x3008,0x42},
	{0x4730,0x01},
	{0x3103,0x03}, {0x3017,0xff}, {0x3018,0xff},
	{0x3034,0x1a}, {0x3035,0x11}, {0x3036,0x46},
	{0x3037,0x13}, {0x3108,0x01}, {0x3630,0x36},
	{0x3631,0x0e}, {0x3632,0xe2}, {0x3633,0x12},
	{0x3621,0xe0}, {0x3704,0xa0}, {0x3703,0x5a},
	{0x3715,0x78}, {0x3717,0x01}, {0x370b,0x60},
	{0x3705,0x1a}, {0x3905,0x02}, {0x3906,0x10},
	{0x3901,0x0a}, {0x3731,0x12}, {0x3600,0x08},
	{0x3601,0x33}, {0x302d,0x60}, {0x3620,0x52},
	{0x371b,0x20}, {0x471c,0x50}, {0x3a13,0x43},
	{0x3a18,0x00}, {0x3a19,0xf8}, {0x3635,0x13},
	{0x3636,0x03}, {0x3634,0x40}, {0x3622,0x01},
	{0x3c01,0x34}, {0x3c04,0x28}, {0x3c05,0x98},
	{0x3c06,0x00}, {0x3c07,0x08}, {0x3c08,0x00},
	{0x3c09,0x1c}, {0x3c0a,0x9c}, {0x3c0b,0x40},
	{0x3820,0x41}, {0x3821,0x07}, {0x3814,0x31},
	{0x3815,0x31}, {0x3800,0x00}, {0x3801,0x00},
	{0x3802,0x00}, {0x3803,0x04}, {0x3804,0x0a},
	{0x3805,0x3f}, {0x3806,0x07}, {0x3807,0x9b},
	{0x3808,0x02}, {0x3809,0x80}, {0x380a,0x01},
	{0x380b,0xe0}, {0x380c,0x07}, {0x380d,0x68},
	{0x380e,0x03}, {0x380f,0xd8}, {0x3810,0x00},
	{0x3811,0x10}, {0x3812,0x00}, {0x3813,0x06},
	{0x3618,0x00}, {0x3612,0x29}, {0x3708,0x64},
	{0x3709,0x52}, {0x370c,0x03}, {0x3a02,0x03},
	{0x3a03,0xd8}, {0x3a08,0x01}, {0x3a09,0x27},
	{0x3a0a,0x00}, {0x3a0b,0xf6}, {0x3a0e,0x03},
	{0x3a0d,0x04}, {0x3a14,0x03}, {0x3a15,0xd8},
	{0x4001,0x02}, {0x4004,0x02}, {0x3000,0x00},
	{0x3002,0x1c}, {0x3004,0xff}, {0x3006,0xc3},
	{0x300e,0x58}, {0x302e,0x00}, {0x4300,0x32},
	{0x501f,0x00}, {0x4713,0x03}, {0x4407,0x04},
	{0x440e,0x00}, {0x460b,0x35}, {0x460c,0x22},
	{0x4837,0x22}, {0x3824,0x02}, {0x5000,0xa7},
	{0x5001,0xa3}, {0x5180,0xff}, {0x5181,0xf2},
	{0x5182,0x00}, {0x5183,0x14}, {0x5184,0x25},
	{0x5185,0x24}, {0x5186,0x09}, {0x5187,0x09},
	{0x5188,0x09}, {0x5189,0x88}, {0x518a,0x54},
	{0x518b,0xee}, {0x518c,0xb2}, {0x518d,0x50},
	{0x518e,0x34}, {0x518f,0x6b}, {0x5190,0x46},
	{0x5191,0xf8}, {0x5192,0x04}, {0x5193,0x70},
	{0x5194,0xf0}, {0x5195,0xf0}, {0x5196,0x03},
	{0x5197,0x01}, {0x5198,0x04}, {0x5199,0x6c},
	{0x519a,0x04}, {0x519b,0x00}, {0x519c,0x09},
	{0x519d,0x2b}, {0x519e,0x38}, {0x5381,0x1e},
	{0x5382,0x5b}, {0x5383,0x08}, {0x5384,0x0a},
	{0x5385,0x7e}, {0x5386,0x88}, {0x5387,0x7c},
	{0x5388,0x6c}, {0x5389,0x10}, {0x538a,0x01},
	{0x538b,0x98}, {0x5300,0x08}, {0x5301,0x30},
	{0x5302,0x10}, {0x5303,0x00}, {0x5304,0x08},
	{0x5305,0x30}, {0x5306,0x08}, {0x5307,0x16},
	{0x5309,0x08}, {0x530a,0x30}, {0x530b,0x04},
	{0x530c,0x06}, {0x5480,0x01}, {0x5481,0x08},
	{0x5482,0x14}, {0x5483,0x28}, {0x5484,0x51},
	{0x5485,0x65}, {0x5486,0x71}, {0x5487,0x7d},
	{0x5488,0x87}, {0x5489,0x91}, {0x548a,0x9a},
	{0x548b,0xaa}, {0x548c,0xb8}, {0x548d,0xcd},
	{0x548e,0xdd}, {0x548f,0xea}, {0x5490,0x1d},
	{0x5580,0x02}, {0x5583,0x40}, {0x5584,0x10},
	{0x5589,0x10}, {0x558a,0x00}, {0x558b,0xf8},
	{0x5800,0x23}, {0x5801,0x14}, {0x5802,0x0f},
	{0x5803,0x0f}, {0x5804,0x12}, {0x5805,0x26},
	{0x5806,0x0c}, {0x5807,0x08}, {0x5808,0x05},
	{0x5809,0x05}, {0x580a,0x08}, {0x580b,0x0d},
	{0x580c,0x08}, {0x580d,0x03}, {0x580e,0x00},
	{0x580f,0x00}, {0x5810,0x03}, {0x5811,0x09},
	{0x5812,0x07}, {0x5813,0x03}, {0x5814,0x00},
	{0x5815,0x01}, {0x5816,0x03}, {0x5817,0x08},
	{0x5818,0x0d}, {0x5819,0x08}, {0x581a,0x05},
	{0x581b,0x06}, {0x581c,0x08}, {0x581d,0x0e},
	{0x581e,0x29}, {0x581f,0x17}, {0x5820,0x11},
	{0x5821,0x11}, {0x5822,0x15}, {0x5823,0x28},
	{0x5824,0x46}, {0x5825,0x26}, {0x5826,0x08},
	{0x5827,0x26}, {0x5828,0x64}, {0x5829,0x26},
	{0x582a,0x24}, {0x582b,0x22}, {0x582c,0x24},
	{0x582d,0x24}, {0x582e,0x06}, {0x582f,0x22},
	{0x5830,0x40}, {0x5831,0x42}, {0x5832,0x24},
	{0x5833,0x26}, {0x5834,0x24}, {0x5835,0x22},
	{0x5836,0x22}, {0x5837,0x26}, {0x5838,0x44},
	{0x5839,0x24}, {0x583a,0x26}, {0x583b,0x28},
	{0x583c,0x42}, {0x583d,0xce}, {0x5025,0x00},
	{0x3a0f,0x30}, {0x3a10,0x28}, {0x3a1b,0x30},
	{0x3a1e,0x26}, {0x3a11,0x60}, {0x3a1f,0x14},
	{0x3008,0x02},
	{0x4730,0x01},
	{0x3034,0x1a}, {0x3035,0x11}, {0x3036,0x46},
	{0x3037,0x13},

	ENDMARKER,
};

//ov5640_setting_30fps_720P_1280x720
static const struct regval_list ov5640_1280_720_regs[] = {
	{0x3008, 0x82},
	{0x4730, 0x01},
	{0x3103, 0x03}, {0x3017, 0xff}, {0x3018, 0xff},
	{0x3034, 0x1a}, {0x3037, 0x13}, {0x3108, 0x01},
	{0x3630, 0x36}, {0x3631, 0x0e}, {0x3632, 0xe2},
	{0x3633, 0x12}, {0x3621, 0xe0}, {0x3704, 0xa0},
	{0x3703, 0x5a}, {0x3715, 0x78}, {0x3717, 0x01},
	{0x370b, 0x60}, {0x3705, 0x1a}, {0x3905, 0x02},
	{0x3906, 0x10}, {0x3901, 0x0a}, {0x3731, 0x12},
	{0x3600, 0x08}, {0x3601, 0x33}, {0x302d, 0x60},
	{0x3620, 0x52}, {0x371b, 0x20}, {0x471c, 0x50},
	{0x3a13, 0x43}, {0x3a18, 0x00}, {0x3a19, 0x7c},
	{0x3635, 0x13}, {0x3636, 0x03}, {0x3634, 0x40},
	{0x3622, 0x01}, {0x3c01, 0x34}, {0x3c04, 0x28},
	{0x3c05, 0x98}, {0x3c06, 0x00}, {0x3c07, 0x08},
	{0x3c08, 0x00}, {0x3c09, 0x1c}, {0x3c0a, 0x9c},
	{0x3c0b, 0x40}, {0x3810, 0x00}, {0x3811, 0x10},
	{0x3812, 0x00}, {0x3708, 0x64}, {0x4001, 0x02},
	{0x4005, 0x1a}, {0x3000, 0x00}, {0x3004, 0xff},
	{0x300e, 0x58}, //Bit[2]:mipi_en 0:DVP enable 1:MIPI enable
	{0x302e, 0x00}, {0x4300, 0x32},
	{0x501f, 0x00}, //ISP format control registers - 000: ISP YUV422
	{0x440e, 0x00}, {0x5000, 0xa7},
	{0x3008, 0x02},
	{0x4730, 0x01},

	{0x3035, 0x21}, //0x11:60fps 0x21:30fps 0x41:15fps 0xa1:7.5fps
	{0x3036, 0x69}, {0x3c07, 0x07}, {0x3820, 0x41},
	{0x3821, 0x07}, //light meter 1 thereshold
	{0x3814, 0x31}, //horizton subsample
	{0x3815, 0x31}, //vertical subsample
	{0x3800, 0x00}, //x address start high byte
	{0x3801, 0x00}, //x address start low byte
	{0x3802, 0x00}, //y address start high byte
	{0x3803, 0xfa}, //y address start low byte
	{0x3804, 0x0a}, //x address end high byte
	{0x3805, 0x3f}, //x address end low byte
	{0x3806, 0x06}, //y address end high byte
	{0x3807, 0xa9}, //y address end low byte
	{0x3808, 0x05}, //H size MSB
	{0x3809, 0x00}, //H size LSB
	{0x380a, 0x02}, //V size MSB
	{0x380b, 0xd0}, //V size LSB
	{0x380c, 0x07}, //HTS MSB
	{0x380d, 0x64}, //HTS LSB
	{0x380e, 0x02}, //VTS MSB
	{0x380f, 0xe4}, //VTS LSB
	{0x3813, 0x04}, {0x3618, 0x00}, {0x3612, 0x29},
	{0x3709, 0x52}, {0x370c, 0x03},
	{0x3a02, 0x02}, //60HZ max exposure limit MSB
	{0x3a03, 0xe0}, //60HZ max exposure limit LSB
	{0x3a14, 0x02}, //50HZ max exposure limit MSB
	{0x3a15, 0xe0}, //50HZ max exposure limit LSB
	{0x4004, 0x02}, //BLC line number
	{0x3002, 0x1c}, //reset JFIFO SFIFO JPG
	{0x3006, 0xc3}, //enable xx clock
	{0x4713, 0x03}, {0x4407, 0x04}, {0x3824, 0x01},
	{0x460b, 0x37}, //debug mode
	{0x460c, 0x20}, //PCLK manual enable
	{0x4837, 0x16}, //PCLK period
	{0x5001, 0xa3}, //ISP effect
	{0x3503, 0x00}, //AEC enable
	{0x3034, 0x1a}, //MIPI bit mode - 0x8: 8-bit mode 0xA: 10-bit mode
	{0x3a08, 0x01}, //AEC B50 STEP - 50Hz Band Width
	{0x3a0a, 0x01}, //AEC B60 STEP - 60Hz Band Width
	{0x3a0d, 0x02}, //60Hz Max Bands in One Frame
	{0x3037, 0x13},
	{0x302c, 0x42},//bit[7:6]: output drive capability
	//00: 1x   01: 2x  10: 3x  11: 4x
	ENDMARKER,
};

//ov5640_setting_30fps_1080P_1920x1080
static const struct regval_list ov5640_1920_1080_regs[] = {
	{0x3008, 0x82},
	{0x4730, 0x01},
	{0x3103, 0x03}, {0x3017, 0xff}, {0x3018, 0xff},
	{0x3034, 0x1a}, {0x3037, 0x13}, {0x3108, 0x01},
	{0x3630, 0x36}, {0x3631, 0x0e}, {0x3632, 0xe2},
	{0x3633, 0x12}, {0x3621, 0xe0}, {0x3704, 0xa0},
	{0x3703, 0x5a}, {0x3715, 0x78}, {0x3717, 0x01},
	{0x370b, 0x60}, {0x3705, 0x1a}, {0x3905, 0x02},
	{0x3906, 0x10}, {0x3901, 0x0a}, {0x3731, 0x12},
	{0x3600, 0x08}, {0x3601, 0x33}, {0x302d, 0x60},
	{0x3620, 0x52}, {0x371b, 0x20}, {0x471c, 0x50},
	{0x3a13, 0x43}, {0x3a18, 0x00}, {0x3a19, 0x7c},
	{0x3635, 0x13}, {0x3636, 0x03}, {0x3634, 0x40},
	{0x3622, 0x01}, {0x3c01, 0x34}, {0x3c04, 0x28},
	{0x3c05, 0x98}, {0x3c06, 0x00}, {0x3c07, 0x08},
	{0x3c08, 0x00}, {0x3c09, 0x1c}, {0x3c0a, 0x9c},
	{0x3c0b, 0x40}, {0x3810, 0x00}, {0x3811, 0x10},
	{0x3812, 0x00}, {0x3708, 0x64}, {0x4001, 0x02},
	{0x4005, 0x1a}, {0x3000, 0x00}, {0x3004, 0xff},
	{0x300e, 0x58}, //Bit[2]:mipi_en 0:DVP enable 1:MIPI enable
	{0x302e, 0x00}, {0x4300, 0x32},
	{0x501f, 0x00}, //ISP format control registers - 000: ISP YUV422
	{0x440e, 0x00}, {0x5000, 0xa7}, {0x3008, 0x02},
	{0x4730, 0x01},

	{0x3c07, 0x07}, {0x3820, 0x40}, {0x3821, 0x06},
	{0x3814, 0x11}, {0x3815, 0x11}, {0x3800, 0x00},
	{0x3801, 0x00}, {0x3802, 0x00}, {0x3803, 0xee},
	{0x3804, 0x0a}, {0x3805, 0x3f}, {0x3806, 0x05},
	{0x3807, 0xc3}, {0x3808, 0x07}, {0x3809, 0x80},
	{0x380a, 0x04}, {0x380b, 0x38}, {0x380c, 0x0b},
	{0x380d, 0x1c}, {0x380e, 0x07}, {0x380f, 0xb0},
	{0x3813, 0x04}, {0x3618, 0x04}, {0x3612, 0x2b},
	{0x3709, 0x12}, {0x370c, 0x00}, {0x3a02, 0x07},
	{0x3a03, 0xae}, {0x3a14, 0x07}, {0x3a15, 0xae},
	{0x4004, 0x06}, {0x3002, 0x1c}, {0x3006, 0xc3},
	{0x4713, 0x02}, {0x4407, 0x0c}, {0x460b, 0x37},
	{0x460c, 0x20}, {0x4837, 0x2c}, {0x3824, 0x01},
	{0x5001, 0x87}, {0x3034, 0x1a}, {0x3035, 0x21},
	{0x3036, 0x69}, {0x3037, 0x13},

	ENDMARKER,
};

/*
 * supported color format list
 */
static const struct ov5640_color_format ov5640_cfmts[] = {
	{
		.code		= MEDIA_BUS_FMT_YUYV8_2X8,
		.colorspace	= V4L2_COLORSPACE_JPEG,
	},
	{
		.code		= MEDIA_BUS_FMT_YVYU8_2X8,
		.colorspace	= V4L2_COLORSPACE_JPEG,
	},
	{
		.code		= MEDIA_BUS_FMT_UYVY8_2X8,
		.colorspace	= V4L2_COLORSPACE_JPEG,
	},
	{
		.code		= MEDIA_BUS_FMT_RGB555_2X8_PADHI_LE,
		.colorspace	= V4L2_COLORSPACE_SRGB,
	},
	{
		.code		= MEDIA_BUS_FMT_RGB555_2X8_PADHI_BE,
		.colorspace	= V4L2_COLORSPACE_SRGB,
	},
	{
		.code		= MEDIA_BUS_FMT_RGB565_2X8_LE,
		.colorspace	= V4L2_COLORSPACE_SRGB,
	},
	{
		.code		= MEDIA_BUS_FMT_RGB565_2X8_BE,
		.colorspace	= V4L2_COLORSPACE_SRGB,
	},
};


static const struct ov5640_win_size ov5640_win_qvga_size_regs = {
	.name     = "QVGA",
	.width    = QVGA_WIDTH,
	.height   = QVGA_HEIGHT,
	.regs     = ov5640_qvga_regs,
};
static const struct ov5640_win_size ov5640_win_vga_size_regs = {
	.name     = "VGA",
	.width    = VGA_WIDTH,
	.height   = VGA_HEIGHT,
	.regs     = ov5640_vga_regs,
};
static const struct ov5640_win_size ov5640_win_hd_size_regs = {
	.name     = "HD",
	.width    = HD_WIDTH,
	.height   = HD_HEIGHT,
	.regs     = ov5640_1280_720_regs,
};
static const struct ov5640_win_size ov5640_win_fhd_size_regs = {
	.name     = "FHD",
	.width    = FHD_WIDTH,
	.height   = FHD_HEIGHT,
	.regs     = ov5640_1920_1080_regs,
};

/*
 * general function
 */

static struct ov5640 *to_ov5640(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov5640, sd);
}

static int ov5640_read(struct v4l2_subdev *sd, u16 reg, u8 *val)
{
	int ret;
	/* We have 16-bit i2c addresses - care for endianess */
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	unsigned char data[2] = { reg >> 8, reg & 0xff };

	ret = i2c_master_send(c, data, 2);
	if (ret < 2) {
		dev_err(&c->dev, "%s: i2c read error, reg: %x\n",
				__func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	ret = i2c_master_recv(c, val, 1);
	if (ret < 1) {
		dev_err(&c->dev, "%s: i2c read error, reg: %x\n",
				__func__, reg);
		return ret < 0 ? ret : -EIO;
	}
	return 0;
}

static inline int ov5640_write(struct v4l2_subdev *sd, u16 reg,
		unsigned char val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	unsigned char data[3] = { reg >> 8, reg & 0xff, val };

	ret = i2c_master_send(client, data, 3);
	if (ret < 3) {
		dev_err(&client->dev, "%s: i2c write error, reg: %x\n",
				__func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static inline int ov5640_mask_set(struct v4l2_subdev *sd, unsigned char addr,
		u8  mask, u8  set)
{
	return 0;
}

static int ov5640_write_array(struct v4l2_subdev *sd,
		const struct regval_list *vals)
{
	while (vals->reg_num != 0xff) {
		int ret = ov5640_write(sd, vals->reg_num,
				vals->value);
		if (ret < 0)
			return ret;
		vals++;
	}
	return 0;
}

static int ov5640_reset(struct v4l2_subdev *sd, u32 val)
{
	if (ov5640_write (sd, 0x3008, 0x82))
		return -1;

	return 0;
}

/*
 * soc_camera_ops function
 */

static int ov5640_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int ov5640_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	return 0;
}

static int ov5640_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return 0;
}

static int ov5640_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov5640_g_register(struct v4l2_subdev *sd,
		struct v4l2_dbg_register *reg)
{
	int ret;

	reg->size = 1;
	if (reg->reg > 0xffff)
		return -EINVAL;

	ov5640_read(sd, reg->reg, &ret);
	if (ret < 0)
		return ret;

	reg->val = (__u64)ret;

	return 0;
}

static int ov5640_s_register(struct v4l2_subdev *sd,
		struct v4l2_dbg_register *reg)
{

	if (reg->reg > 0xffff ||
			reg->val > 0xff)
		return -EINVAL;

	ov5640_write (sd, reg->reg, reg->val);

	return 0;
}
#endif

/* Find a frame size in an array */
static int ov5640_find_framesize(u32 width, u32 height)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ov5640_frmsizes); i++) {
		if ((ov5640_frmsizes[i].width >= width) &&
				(ov5640_frmsizes[i].height >= height))
			break;
	}
	/* If not found, select biggest */
	if (i >= ARRAY_SIZE(ov5640_frmsizes))
		i = ARRAY_SIZE(ov5640_frmsizes) - 1;

	return i;
}

static const struct ov5640_win_size *ov5640_select_win(u32 width, u32 height)
{
	const struct ov5640_win_size *win;
	int f_size = ov5640_find_framesize(width, height);

	switch (ov5640_frmsizes[f_size].width) {
		case QVGA_WIDTH:
			win = &ov5640_win_qvga_size_regs;
			break;
		case VGA_WIDTH:
			win = &ov5640_win_vga_size_regs;
			break;
		case HD_WIDTH:
			win = &ov5640_win_hd_size_regs;
			break;
		case FHD_WIDTH:
			win = &ov5640_win_fhd_size_regs;
			break;
	}

	return win;
}

static int ov5640_set_params(struct v4l2_subdev *sd, u32 *width, u32 *height,
	u32 code)
{
	struct ov5640 *core = to_ov5640(sd);
	int i;

	/*
	 * select format
	 */
	core->cfmt = NULL;
	for (i = 0; i < ARRAY_SIZE(ov5640_cfmts); i++) {
		if (code == ov5640_cfmts[i].code) {
			core->cfmt = ov5640_cfmts + i;
			break;
		}
	}
	if (!core->cfmt)
		goto ov5640_set_fmt_error;

	/*
	 * select win
	 */
	core->win = ov5640_select_win(*width, *height);


	//reset hardware
	ov5640_reset(sd, 0);

	ov5640_write_array(sd, core->win->regs);

	/*
	 * set size format
	 */
	*width = core->win->width;
	*height = core->win->height;

	return 0;

ov5640_set_fmt_error:

	ov5640_reset(sd, 0);
	core->win = NULL;
	core->cfmt = NULL;

	return 0;
}

static int ov5640_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct ov5640 *core = to_ov5640(sd);

	a->c.left	= 0;
	a->c.top	= 0;
	a->c.width	= core->win->width;
	a->c.height	= core->win->height;
	a->type		= V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}

static int ov5640_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	struct ov5640 *core = to_ov5640(sd);

	a->bounds.left			= 0;
	a->bounds.top			= 0;
	a->bounds.width			= FHD_WIDTH;
	a->bounds.height		= FHD_HEIGHT;
	a->defrect			= a->bounds;
	a->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

static int ov5640_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf;
	struct ov5640 *core = to_ov5640(sd);

	mf = &format->format;
	if (format->pad)
		return -EINVAL;

	if (!core->win || !core->cfmt) {
		int f_size = ov5640_find_framesize(mf->width, mf->height);
		u32 width = ov5640_frmsizes[f_size].width, height = ov5640_frmsizes[f_size].height;
		int ret = ov5640_set_params(sd, &width, &height,
				MEDIA_BUS_FMT_YUYV8_2X8);
		if (ret < 0)
			return ret;
	}

	mf->width	= core->win->width;
	mf->height	= core->win->height;
	mf->code	= core->cfmt->code;
	mf->colorspace	= core->cfmt->colorspace;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int ov5640_s_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *mf)
{
	struct ov5640 *core = to_ov5640(sd);
	int ret = ov5640_set_params(sd, &mf->width, &mf->height,
			mf->code);

	if (!ret)
		mf->colorspace = core->cfmt->colorspace;

	return ret;
}

static int ov5640_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct ov5640 *core = to_ov5640(sd);
	const struct ov5640_win_size *win;
	int i;

	if (format->pad)
		return -EINVAL;

	/*
	 * select suitable win
	 */
	core->win = ov5640_select_win(mf->width, mf->height);

	mf->width	= core->win->width;
	mf->height	= core->win->height;
	mf->field	= V4L2_FIELD_NONE;

	for (i = 0; i < ARRAY_SIZE(ov5640_cfmts); i++)
		if (mf->code == ov5640_cfmts[i].code)
			break;

	if (i == ARRAY_SIZE(ov5640_cfmts)) {
		/* Unsupported format requested. Propose either */
		if (core->cfmt) {
			/* the current one or */
			mf->colorspace = core->cfmt->colorspace;
			mf->code = core->cfmt->code;
		} else {
			/* the default one */
			mf->colorspace = ov5640_cfmts[0].colorspace;
			mf->code = ov5640_cfmts[0].code;
		}
	} else {
		/* Also return the colorspace */
		mf->colorspace	= ov5640_cfmts[i].colorspace;
	}

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		return ov5640_s_fmt(sd, mf);
	cfg->try_fmt = *mf;

	return 0;
}

static int ov5640_video_probe (struct v4l2_subdev *sd)
{
	u8 pid, ver;
	const char         *devname;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/*
	 * check and show product ID and manufacturer ID
	 */
	ov5640_read(sd, 0x300a, &pid);
	ov5640_read(sd, 0x300b, &ver);
	switch (VERSION(pid, ver)) {
		case OV5640:
			devname     = "ov5640";
			break;
		default:
			dev_err(&client->dev, "Product ID error %x:%x\n", pid, ver);
			return -ENODEV;
	}

	dev_info(&client->dev,
			"%s: Product ID %0x:%0x\n",
			devname,
			pid,
			ver);
	return 0;

}

static struct v4l2_subdev_core_ops ov5640_subdev_core_ops = {
	.reset		= ov5640_reset,
	.queryctrl      = ov5640_queryctrl,
	.g_ctrl		= ov5640_g_ctrl,
	.s_ctrl		= ov5640_s_ctrl,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= ov5640_g_register,
	.s_register	= ov5640_s_register,
#endif
};

static int ov5640_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(ov5640_cfmts))
		return -EINVAL;

	code->code = ov5640_cfmts[code->index].code;

	return 0;
}

static int ov5640_g_mbus_config(struct v4l2_subdev *sd,
		struct v4l2_mbus_config *cfg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);

	cfg->flags = V4L2_MBUS_PCLK_SAMPLE_RISING | V4L2_MBUS_MASTER |
		V4L2_MBUS_VSYNC_ACTIVE_HIGH | V4L2_MBUS_HSYNC_ACTIVE_HIGH |
		V4L2_MBUS_DATA_ACTIVE_HIGH;
	cfg->type = V4L2_MBUS_BT656;
	cfg->flags = soc_camera_apply_board_flags(ssdd, cfg);

	return 0;
}

static struct v4l2_subdev_video_ops ov5640_subdev_video_ops = {
	.s_stream	= ov5640_s_stream,
	.cropcap	= ov5640_cropcap,
	.g_crop		= ov5640_g_crop,
	.g_mbus_config  = ov5640_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops ov5640_pad_ops = {
	.enum_mbus_code = ov5640_enum_mbus_code,
	.get_fmt = ov5640_get_fmt,
	.set_fmt = ov5640_set_fmt,
};

static struct v4l2_subdev_ops ov5640_subdev_ops = {
	.core	= &ov5640_subdev_core_ops,
	.video	= &ov5640_subdev_video_ops,
	.pad    = &ov5640_pad_ops,
};

static int ov5640_probe_dt(struct i2c_client *client,
		struct ov5640 *priv)
{
	client->dev.platform_data = &priv->ssdd_dt;
	return 0;
}

/*
 * i2c_driver function
 */
static int ov5640_probe(struct i2c_client *client,
		const struct i2c_device_id *did)
{
	struct ov5640 *priv;
	struct v4l2_subdev *sd;
	struct device_node *np;
	int ret, cam_pwn;

	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
	struct i2c_adapter	*adapter = to_i2c_adapter(client->dev.parent);

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&adapter->dev,
				"OV5640: I2C-Adapter doesn't support SMBUS\n");
		return -EIO;
	}

	priv = kzalloc(sizeof(struct ov5640), GFP_KERNEL);
	if (!priv) {
		dev_err(&adapter->dev,
			"Failed to allocate memory for private data!\n");
		return -ENOMEM;
	}

	if (!ssdd && !client->dev.of_node) {
		dev_err(&client->dev, "Missing platform_data for driver\n");
		goto err_probe;
	}

	if (!ssdd) {
		ret = ov5640_probe_dt(client, priv);
		if (ret)
			goto err_probe;
	}

	/* init subdev */
	sd = &priv->sd;
	v4l2_i2c_subdev_init(sd, client, &ov5640_subdev_ops);

	/* detect camera sensor */
	ret = ov5640_video_probe(sd);
	if (ret) {
		v4l2_info(sd, "camera ov5640 is not found.\n");
		goto err_probe;
	}
	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr , client->adapter->name);

	/* init control handler */
	v4l2_ctrl_handler_init(&priv->hdl, 2);
	sd->ctrl_handler = &priv->hdl;
	if (priv->hdl.error) {
		ret = priv->hdl.error;
		goto err_hdl;
	}
	v4l2_ctrl_handler_setup(&priv->hdl);

	/* register subdev */
	ret = v4l2_async_register_subdev(sd);
	if (ret) {
		v4l2_info(sd, "Failed to register subdev.\n");
		ret = -EINVAL;
		goto err_hdl;
	}

	return 0;

err_hdl:
	v4l2_ctrl_handler_free(&priv->hdl);

err_probe:
	kfree(priv);
	return ret;
}

static int ov5640_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov5640 *ov5640 = to_ov5640(sd);

	v4l2_dbg(1, debug, sd,
			"ov5640.c: removing ov5640 adapter on address 0x%x\n",
			client->addr << 1);

	v4l2_device_unregister_subdev(sd);
	v4l2_async_unregister_subdev(&ov5640->sd);
	media_entity_cleanup(&ov5640->sd.entity);
	v4l2_ctrl_handler_free(&ov5640->hdl);
	kfree(to_ov5640(sd));
	return 0;
}

static const struct i2c_device_id ov5640_id[] = {
	{ "ov5640", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov5640_id);

static const struct of_device_id ov5640_of_match[] = {
	{.compatible = "ovti,ov5640", },
	{},
};
MODULE_DEVICE_TABLE(of, ov5640_of_match);

static struct i2c_driver ov5640_i2c_driver = {
	.driver = {
		.name = "ov5640",
		.of_match_table = of_match_ptr(ov5640_of_match),
	},
	.probe    = ov5640_probe,
	.remove   = ov5640_remove,
	.id_table = ov5640_id,
};

module_i2c_driver(ov5640_i2c_driver);

MODULE_DESCRIPTION("Camera driver for ov5640");
MODULE_LICENSE("GPL v2");
