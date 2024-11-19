// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony IMX477 cameras.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd
 *
 * Based on Sony imx219 camera driver
 * Copyright (C) 2019-2020 Raspberry Pi (Trading) Ltd
 */
#include <asm/unaligned.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <linux/dev_printk.h>


static int dpc_enable = 1;
module_param(dpc_enable, int, 0644);
MODULE_PARM_DESC(dpc_enable, "Enable on-sensor DPC");

static int trigger_mode;
module_param(trigger_mode, int, 0644);
MODULE_PARM_DESC(trigger_mode, "Set vsync trigger mode: 1=source, 2=sink");

#define IMX477_REG_VALUE_08BIT		1
#define IMX477_REG_VALUE_16BIT		2

/* Chip ID */
#define IMX477_REG_CHIP_ID		0x0016
#define IMX477_CHIP_ID			0x0477
#define IMX378_CHIP_ID			0x0378

#define IMX477_REG_MODE_SELECT		0x0100
#define IMX477_MODE_STANDBY		0x00
#define IMX477_MODE_STREAMING		0x01

#define IMX477_REG_ORIENTATION		0x101

#define IMX477_XCLK_FREQ		24000000

#define IMX477_DEFAULT_LINK_FREQ	450000000

/* Pixel rate is fixed at 840MHz for all the modes */
#define IMX477_PIXEL_RATE		840000000

/* V_TIMING internal */
#define IMX477_REG_FRAME_LENGTH		0x0340
#define IMX477_FRAME_LENGTH_MAX		0xffdc

/* H_TIMING internal */
#define IMX477_REG_LINE_LENGTH		0x0342
#define IMX477_LINE_LENGTH_MAX		0xfff0

/* Long exposure multiplier */
#define IMX477_LONG_EXP_SHIFT_MAX	7
#define IMX477_LONG_EXP_SHIFT_REG	0x3100

/* Exposure control */
#define IMX477_REG_EXPOSURE		0x0202
#define IMX477_EXPOSURE_OFFSET		22
#define IMX477_EXPOSURE_MIN		4
#define IMX477_EXPOSURE_STEP		1
#define IMX477_EXPOSURE_DEFAULT		0x640
#define IMX477_EXPOSURE_MAX		(IMX477_FRAME_LENGTH_MAX - \
					 IMX477_EXPOSURE_OFFSET)

/* Analog gain control */
#define IMX477_REG_ANALOG_GAIN		0x0204
#define IMX477_ANA_GAIN_MIN		0
#define IMX477_ANA_GAIN_MAX		978
#define IMX477_ANA_GAIN_STEP		1
#define IMX477_ANA_GAIN_DEFAULT		0x0

/* Digital gain control */
#define IMX477_REG_DIGITAL_GAIN		0x020e
#define IMX477_DGTL_GAIN_MIN		0x0100
#define IMX477_DGTL_GAIN_MAX		0xffff
#define IMX477_DGTL_GAIN_DEFAULT	0x0100
#define IMX477_DGTL_GAIN_STEP		1

/* Test Pattern Control */
#define IMX477_REG_TEST_PATTERN		0x0600
#define IMX477_TEST_PATTERN_DISABLE	0
#define IMX477_TEST_PATTERN_SOLID_COLOR	1
#define IMX477_TEST_PATTERN_COLOR_BARS	2
#define IMX477_TEST_PATTERN_GREY_COLOR	3
#define IMX477_TEST_PATTERN_PN9		4

/* Test pattern colour components */
#define IMX477_REG_TEST_PATTERN_R	0x0602
#define IMX477_REG_TEST_PATTERN_GR	0x0604
#define IMX477_REG_TEST_PATTERN_B	0x0606
#define IMX477_REG_TEST_PATTERN_GB	0x0608
#define IMX477_TEST_PATTERN_COLOUR_MIN	0
#define IMX477_TEST_PATTERN_COLOUR_MAX	0x0fff
#define IMX477_TEST_PATTERN_COLOUR_STEP	1
#define IMX477_TEST_PATTERN_R_DEFAULT	IMX477_TEST_PATTERN_COLOUR_MAX
#define IMX477_TEST_PATTERN_GR_DEFAULT	0
#define IMX477_TEST_PATTERN_B_DEFAULT	0
#define IMX477_TEST_PATTERN_GB_DEFAULT	0

/* Trigger mode */
#define IMX477_REG_MC_MODE		0x3f0b
#define IMX477_REG_MS_SEL		0x3041
#define IMX477_REG_XVS_IO_CTRL		0x3040
#define IMX477_REG_EXTOUT_EN		0x4b81

#define IMX477_X_START_REG      0x0344
#define IMX477_Y_START_REG      0x0346
#define IMX477_X_END_REG        0x0348
#define IMX477_Y_END_REG        0x034a
#define IMX477_X_SIZE_REG       0x034c
#define IMX477_Y_SIZE_REG       0x034e
#define IMX477_X_DIG_CROP_SIZE_REG       0x040c
#define IMX477_Y_DIG_CROP_SIZE_REG       0x040e



/* Embedded metadata stream structure */
#define IMX477_EMBEDDED_LINE_WIDTH 16384
#define IMX477_NUM_EMBEDDED_LINES 1

enum pad_types {
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};

/* IMX477 native and active pixel array size. */
#define IMX477_NATIVE_WIDTH		4072U
#define IMX477_NATIVE_HEIGHT		3176U
#define IMX477_PIXEL_ARRAY_LEFT		8U
#define IMX477_PIXEL_ARRAY_TOP		16U
#define IMX477_PIXEL_ARRAY_WIDTH	4056U
#define IMX477_PIXEL_ARRAY_HEIGHT	3040U


#define V4L2_CID_I2C_8B				(V4L2_CID_USER_S2255_BASE + 1)
#define V4L2_CID_I2C_16B			(V4L2_CID_USER_S2255_BASE + 2)
#define V4L2_CID_I2C_8B_GENERIC		(V4L2_CID_USER_S2255_BASE + 3)
#define V4L2_CID_POWER_ON			(V4L2_CID_USER_S2255_BASE + 4)		// This should be disabled as it forces to redo power_on during init_control?
#define V4L2_CID_I2C_8B_READ		(V4L2_CID_USER_S2255_BASE + 5)
#define V4L2_CID_I2C_16B_READ		(V4L2_CID_USER_S2255_BASE + 6)
#define V4L2_CID_I2C_SET_GENERIC_ID	(V4L2_CID_USER_S2255_BASE + 7)
#define V4L2_CID_I2C_SET_READ_ADDR	(V4L2_CID_USER_S2255_BASE + 8)
#define V4L2_CID_I2C_8B_GENERIC_READ (V4L2_CID_USER_S2255_BASE + 9)
#define V4L2_CID_ROI_START_X		(V4L2_CID_USER_S2255_BASE + 10)
#define V4L2_CID_ROI_START_Y		(V4L2_CID_USER_S2255_BASE + 11)
#define V4L2_CID_FORCE_TRIGGER		(V4L2_CID_USER_S2255_BASE + 12)
#define V4L2_CID_BINNING			(V4L2_CID_USER_S2255_BASE + 13)


struct imx477_reg {
	u16 address;
	u8 val;
};

struct imx477_reg_list {
	unsigned int num_of_regs;
	const struct imx477_reg *regs;
};

/* Mode : resolution and related config&values */
struct imx477_mode {
	/* Frame width */
	unsigned int orig_width;

	/* Frame height */
	unsigned int orig_height;

	/* H-timing in pixels */			// Changed imx477_set_framing_limits to ignore this and use min_hblank
	unsigned int line_length_pix;

	unsigned int min_hblank;
	unsigned int min_vblank;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Highest possible framerate. */		// Changed imx477_set_framing_limits to ignore this and use min_vblank
	struct v4l2_fract timeperframe_min;

	/* Default framerate. */
	struct v4l2_fract timeperframe_default;	// Changed imx477_set_framing_limits to ignore this and use min_vblank

	/* Default register values */
	struct imx477_reg_list reg_list;
};

static const s64 imx477_link_freq_menu[] = {
	IMX477_DEFAULT_LINK_FREQ,
};

static const struct imx477_reg mode_common_regs[] = {
	{0x0136, 0x18},
	{0x0137, 0x00},
	{0x0138, 0x01},
	{0xe000, 0x00},
	{0xe07a, 0x01},
	{0x0808, 0x02},
	{0x4ae9, 0x18},
	{0x4aea, 0x08},
	{0xf61c, 0x04},
	{0xf61e, 0x04},
	{0x4ae9, 0x21},
	{0x4aea, 0x80},
	{0x38a8, 0x1f},
	{0x38a9, 0xff},
	{0x38aa, 0x1f},
	{0x38ab, 0xff},
	{0x55d4, 0x00},
	{0x55d5, 0x00},
	{0x55d6, 0x07},
	{0x55d7, 0xff},
	{0x55e8, 0x07},
	{0x55e9, 0xff},
	{0x55ea, 0x00},
	{0x55eb, 0x00},
	{0x574c, 0x07},
	{0x574d, 0xff},
	{0x574e, 0x00},
	{0x574f, 0x00},
	{0x5754, 0x00},
	{0x5755, 0x00},
	{0x5756, 0x07},
	{0x5757, 0xff},
	{0x5973, 0x04},
	{0x5974, 0x01},
	{0x5d13, 0xc3},
	{0x5d14, 0x58},
	{0x5d15, 0xa3},
	{0x5d16, 0x1d},
	{0x5d17, 0x65},
	{0x5d18, 0x8c},
	{0x5d1a, 0x06},
	{0x5d1b, 0xa9},
	{0x5d1c, 0x45},
	{0x5d1d, 0x3a},
	{0x5d1e, 0xab},
	{0x5d1f, 0x15},
	{0x5d21, 0x0e},
	{0x5d22, 0x52},
	{0x5d23, 0xaa},
	{0x5d24, 0x7d},
	{0x5d25, 0x57},
	{0x5d26, 0xa8},
	{0x5d37, 0x5a},
	{0x5d38, 0x5a},
	{0x5d77, 0x7f},
	{0x7b75, 0x0e},
	{0x7b76, 0x0b},
	{0x7b77, 0x08},
	{0x7b78, 0x0a},
	{0x7b79, 0x47},
	{0x7b7c, 0x00},
	{0x7b7d, 0x00},
	{0x8d1f, 0x00},
	{0x8d27, 0x00},
	{0x9004, 0x03},
	{0x9200, 0x50},
	{0x9201, 0x6c},
	{0x9202, 0x71},
	{0x9203, 0x00},
	{0x9204, 0x71},
	{0x9205, 0x01},
	{0x9371, 0x6a},
	{0x9373, 0x6a},
	{0x9375, 0x64},
	{0x991a, 0x00},
	{0x996b, 0x8c},
	{0x996c, 0x64},
	{0x996d, 0x50},
	{0x9a4c, 0x0d},
	{0x9a4d, 0x0d},
	{0xa001, 0x0a},
	{0xa003, 0x0a},
	{0xa005, 0x0a},
	{0xa006, 0x01},
	{0xa007, 0xc0},
	{0xa009, 0xc0},
	{0x3d8a, 0x01},
	{0x4421, 0x04},
	{0x7b3b, 0x01},
	{0x7b4c, 0x00},
	{0x9905, 0x00},
	{0x9907, 0x00},
	{0x9909, 0x00},
	{0x990b, 0x00},
	{0x9944, 0x3c},
	{0x9947, 0x3c},
	{0x994a, 0x8c},
	{0x994b, 0x50},
	{0x994c, 0x1b},
	{0x994d, 0x8c},
	{0x994e, 0x50},
	{0x994f, 0x1b},
	{0x9950, 0x8c},
	{0x9951, 0x1b},
	{0x9952, 0x0a},
	{0x9953, 0x8c},
	{0x9954, 0x1b},
	{0x9955, 0x0a},
	{0x9a13, 0x04},
	{0x9a14, 0x04},
	{0x9a19, 0x00},
	{0x9a1c, 0x04},
	{0x9a1d, 0x04},
	{0x9a26, 0x05},
	{0x9a27, 0x05},
	{0x9a2c, 0x01},
	{0x9a2d, 0x03},
	{0x9a2f, 0x05},
	{0x9a30, 0x05},
	{0x9a41, 0x00},
	{0x9a46, 0x00},
	{0x9a47, 0x00},
	{0x9c17, 0x35},
	{0x9c1d, 0x31},
	{0x9c29, 0x50},
	{0x9c3b, 0x2f},
	{0x9c41, 0x6b},
	{0x9c47, 0x2d},
	{0x9c4d, 0x40},
	{0x9c6b, 0x00},
	{0x9c71, 0xc8},
	{0x9c73, 0x32},
	{0x9c75, 0x04},
	{0x9c7d, 0x2d},
	{0x9c83, 0x40},
	{0x9c94, 0x3f},
	{0x9c95, 0x3f},
	{0x9c96, 0x3f},
	{0x9c97, 0x00},
	{0x9c98, 0x00},
	{0x9c99, 0x00},
	{0x9c9a, 0x3f},
	{0x9c9b, 0x3f},
	{0x9c9c, 0x3f},
	{0x9ca0, 0x0f},
	{0x9ca1, 0x0f},
	{0x9ca2, 0x0f},
	{0x9ca3, 0x00},
	{0x9ca4, 0x00},
	{0x9ca5, 0x00},
	{0x9ca6, 0x1e},
	{0x9ca7, 0x1e},
	{0x9ca8, 0x1e},
	{0x9ca9, 0x00},
	{0x9caa, 0x00},
	{0x9cab, 0x00},
	{0x9cac, 0x09},
	{0x9cad, 0x09},
	{0x9cae, 0x09},
	{0x9cbd, 0x50},
	{0x9cbf, 0x50},
	{0x9cc1, 0x50},
	{0x9cc3, 0x40},
	{0x9cc5, 0x40},
	{0x9cc7, 0x40},
	{0x9cc9, 0x0a},
	{0x9ccb, 0x0a},
	{0x9ccd, 0x0a},
	{0x9d17, 0x35},
	{0x9d1d, 0x31},
	{0x9d29, 0x50},
	{0x9d3b, 0x2f},
	{0x9d41, 0x6b},
	{0x9d47, 0x42},
	{0x9d4d, 0x5a},
	{0x9d6b, 0x00},
	{0x9d71, 0xc8},
	{0x9d73, 0x32},
	{0x9d75, 0x04},
	{0x9d7d, 0x42},
	{0x9d83, 0x5a},
	{0x9d94, 0x3f},
	{0x9d95, 0x3f},
	{0x9d96, 0x3f},
	{0x9d97, 0x00},
	{0x9d98, 0x00},
	{0x9d99, 0x00},
	{0x9d9a, 0x3f},
	{0x9d9b, 0x3f},
	{0x9d9c, 0x3f},
	{0x9d9d, 0x1f},
	{0x9d9e, 0x1f},
	{0x9d9f, 0x1f},
	{0x9da0, 0x0f},
	{0x9da1, 0x0f},
	{0x9da2, 0x0f},
	{0x9da3, 0x00},
	{0x9da4, 0x00},
	{0x9da5, 0x00},
	{0x9da6, 0x1e},
	{0x9da7, 0x1e},
	{0x9da8, 0x1e},
	{0x9da9, 0x00},
	{0x9daa, 0x00},
	{0x9dab, 0x00},
	{0x9dac, 0x09},
	{0x9dad, 0x09},
	{0x9dae, 0x09},
	{0x9dc9, 0x0a},
	{0x9dcb, 0x0a},
	{0x9dcd, 0x0a},
	{0x9e17, 0x35},
	{0x9e1d, 0x31},
	{0x9e29, 0x50},
	{0x9e3b, 0x2f},
	{0x9e41, 0x6b},
	{0x9e47, 0x2d},
	{0x9e4d, 0x40},
	{0x9e6b, 0x00},
	{0x9e71, 0xc8},
	{0x9e73, 0x32},
	{0x9e75, 0x04},
	{0x9e94, 0x0f},
	{0x9e95, 0x0f},
	{0x9e96, 0x0f},
	{0x9e97, 0x00},
	{0x9e98, 0x00},
	{0x9e99, 0x00},
	{0x9ea0, 0x0f},
	{0x9ea1, 0x0f},
	{0x9ea2, 0x0f},
	{0x9ea3, 0x00},
	{0x9ea4, 0x00},
	{0x9ea5, 0x00},
	{0x9ea6, 0x3f},
	{0x9ea7, 0x3f},
	{0x9ea8, 0x3f},
	{0x9ea9, 0x00},
	{0x9eaa, 0x00},
	{0x9eab, 0x00},
	{0x9eac, 0x09},
	{0x9ead, 0x09},
	{0x9eae, 0x09},
	{0x9ec9, 0x0a},
	{0x9ecb, 0x0a},
	{0x9ecd, 0x0a},
	{0x9f17, 0x35},
	{0x9f1d, 0x31},
	{0x9f29, 0x50},
	{0x9f3b, 0x2f},
	{0x9f41, 0x6b},
	{0x9f47, 0x42},
	{0x9f4d, 0x5a},
	{0x9f6b, 0x00},
	{0x9f71, 0xc8},
	{0x9f73, 0x32},
	{0x9f75, 0x04},
	{0x9f94, 0x0f},
	{0x9f95, 0x0f},
	{0x9f96, 0x0f},
	{0x9f97, 0x00},
	{0x9f98, 0x00},
	{0x9f99, 0x00},
	{0x9f9a, 0x2f},
	{0x9f9b, 0x2f},
	{0x9f9c, 0x2f},
	{0x9f9d, 0x00},
	{0x9f9e, 0x00},
	{0x9f9f, 0x00},
	{0x9fa0, 0x0f},
	{0x9fa1, 0x0f},
	{0x9fa2, 0x0f},
	{0x9fa3, 0x00},
	{0x9fa4, 0x00},
	{0x9fa5, 0x00},
	{0x9fa6, 0x1e},
	{0x9fa7, 0x1e},
	{0x9fa8, 0x1e},
	{0x9fa9, 0x00},
	{0x9faa, 0x00},
	{0x9fab, 0x00},
	{0x9fac, 0x09},
	{0x9fad, 0x09},
	{0x9fae, 0x09},
	{0x9fc9, 0x0a},
	{0x9fcb, 0x0a},
	{0x9fcd, 0x0a},
	{0xa14b, 0xff},
	{0xa151, 0x0c},
	{0xa153, 0x50},
	{0xa155, 0x02},
	{0xa157, 0x00},
	{0xa1ad, 0xff},
	{0xa1b3, 0x0c},
	{0xa1b5, 0x50},
	{0xa1b9, 0x00},
	{0xa24b, 0xff},
	{0xa257, 0x00},
	{0xa2ad, 0xff},
	{0xa2b9, 0x00},
	{0xb21f, 0x04},
	{0xb35c, 0x00},
	{0xb35e, 0x08},
	{0x0112, 0x0c},
	{0x0113, 0x0c},
	{0x0114, 0x01},
	{0x0350, 0x01},		// Automatically use exposure len as frame_len in case the first one is larger
						// Check https://patchwork.kernel.org/project/linux-media/patch/20230530173000.3060865-11-dave.stevenson@raspberrypi.com/
	//{0xbcf1, 0x02},
	{0xbcf1, 0x00},		// No embedded data!
	{0x3ff9, 0x01},		// Related to digital gain?
};

/* 12 mpix 10fps */
static const struct imx477_reg mode_4056x3040_regs[] = {
	{0x0342, 0x5d},		// Modified later after ROI - IMX477_REG_LINE_LENGTH, based on hblank
	{0x0343, 0xc0},		// Modified later after ROI
	{0x0344, 0x00},		// Modified later after ROI - IMX477_X_START_REG
	{0x0345, 0x00},		// Modified later after ROI
	{0x0346, 0x00},		// Modified later after ROI - IMX477_Y_START_REG
	{0x0347, 0x00},		// Modified later after ROI
	{0x0348, 0x0f},		// Modified later after ROI - IMX477_X_END_REG
	{0x0349, 0xd7},		// Modified later after ROI
	{0x034a, 0x0b},		// Modified later after ROI - IMX477_Y_END_REG
	{0x034b, 0xdf},		// Modified later after ROI
	{0x00e3, 0x00},
	{0x00e4, 0x00},
	{0x00fc, 0x0a},
	{0x00fd, 0x0a},
	{0x00fe, 0x0a},
	{0x00ff, 0x0a},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x03},
	{0x3c02, 0xa2},
	{0x3f0d, 0x01},		// Switch between 12b and 10b??
	{0x5748, 0x07},
	{0x5749, 0xff},
	{0x574a, 0x00},
	{0x574b, 0x00},
	{0x7b75, 0x0a},
	{0x7b76, 0x0c},
	{0x7b77, 0x07},
	{0x7b78, 0x06},
	{0x7b79, 0x3c},
	{0x7b53, 0x01},
	{0x9369, 0x5a},	// ??
	{0x936b, 0x55},
	{0x936d, 0x28},
	{0x9304, 0x00}, // ??
	{0x9305, 0x00}, // ??
	{0x9e9a, 0x2f}, // ??
	{0x9e9b, 0x2f}, // ??
	{0x9e9c, 0x2f}, // ??
	{0x9e9d, 0x00}, // ??
	{0x9e9e, 0x00}, // ??
	{0x9e9f, 0x00}, // ??
	{0xa2a9, 0x60},
	{0xa2b7, 0x00},
    {0x0401, 0x00},     // Digital , no scaling
    {0x0404, 0x00},     // Down Scaling Factor M[8]
    {0x0405, 0x10},     // Scale factor [7:0]
    {0x0408, 0x00},     // Digital crop X _offset
    {0x0409, 0x00},     //              X_offset
    {0x040a, 0x00},     //              y_offset
    {0x040b, 0x00},     //              y_offset
	{0x040c, 0x0f},	// Modifiyed later for ROI - IMX477_X_DIG_CROP_SIZE_REG
	{0x040d, 0xd8},	// Modifiyed later for ROI
	{0x040e, 0x0b},	// Modifiyed later for ROI - IMX477_Y_DIG_CROP_SIZE_REG
	{0x040f, 0xe0},	// Modifiyed later for ROI
	{0x034c, 0x0f},	// Modifiyed later for ROI - IMX477_X_SIZE_REG
	{0x034d, 0xd8},	// Modifiyed later for ROI
	{0x034e, 0x0b},	// Modifiyed later for ROI - IMX477_Y_SIZE_REG
	{0x034f, 0xe0},	// Modifiyed later for ROI
    {0x0301, 0x05},     // The Pixel Clock Divider for IVTS  ??
    {0x0303, 0x02},     // The System Clock Divider for IVTS ??
	{0x0305, 0x04},		// PLL sutff ??
	{0x0306, 0x01},
	{0x0307, 0x5e},
	{0x0309, 0x0c},		// Some clock divider ?
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x030e, 0x00},
	{0x030f, 0x96},
	{0x0310, 0x01},
	{0x0820, 0x07},		// 32-bit output data rate??
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x7f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},		// Power save?
	{0x3f56, 0x02},		// No effect? Maybe power save related
	{0x3f57, 0xae},		// No effect?
};

/* 2x2 binned. 40fps */
static const struct imx477_reg mode_2028x1520_regs[] = {
	{0x0342, 0x31},
	{0x0343, 0xc4},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x0b},
	{0x034b, 0xdf},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x03},
	{0x3c02, 0xa2},
	{0x3f0d, 0x01},
	{0x5748, 0x07},
	{0x5749, 0xff},
	{0x574a, 0x00},
	{0x574b, 0x00},
	{0x7b53, 0x01},
	{0x9369, 0x73},
	{0x936b, 0x64},
	{0x936d, 0x5f},
	{0x9304, 0x00},
	{0x9305, 0x00},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0xa2a9, 0x60},
	{0xa2b7, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x20},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f},
	{0x040d, 0xd8},
	{0x040e, 0x0b},
	{0x040f, 0xe0},
	{0x034c, 0x07},
	{0x034d, 0xec},
	{0x034e, 0x05},
	{0x034f, 0xf0},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5e},
	{0x0309, 0x0c},
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x030e, 0x00},
	{0x030f, 0x96},
	{0x0310, 0x01},
	{0x0820, 0x07},
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x7f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x01},
	{0x3f57, 0x6c},
};

/* 1080p cropped mode */
static const struct imx477_reg mode_2028x1080_regs[] = {
	{0x0342, 0x31},
	{0x0343, 0xc4},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x01},
	{0x0347, 0xb8},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x0a},
	{0x034b, 0x27},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x03},
	{0x3c02, 0xa2},
	{0x3f0d, 0x01},
	{0x5748, 0x07},
	{0x5749, 0xff},
	{0x574a, 0x00},
	{0x574b, 0x00},
	{0x7b53, 0x01},
	{0x9369, 0x73},
	{0x936b, 0x64},
	{0x936d, 0x5f},
	{0x9304, 0x00},
	{0x9305, 0x00},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0xa2a9, 0x60},
	{0xa2b7, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x20},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f},
	{0x040d, 0xd8},
	{0x040e, 0x04},
	{0x040f, 0x38},
	{0x034c, 0x07},
	{0x034d, 0xec},
	{0x034e, 0x04},
	{0x034f, 0x38},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5e},
	{0x0309, 0x0c},
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x030e, 0x00},
	{0x030f, 0x96},
	{0x0310, 0x01},
	{0x0820, 0x07},
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x7f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x01},
	{0x3f57, 0x6c},
};

/* 4x4 binned. 120fps */
static const struct imx477_reg mode_1332x990_regs[] = {
	{0x420b, 0x01},
	{0x990c, 0x00},
	{0x990d, 0x08},
	{0x9956, 0x8c},
	{0x9957, 0x64},
	{0x9958, 0x50},
	{0x9a48, 0x06},
	{0x9a49, 0x06},
	{0x9a4a, 0x06},
	{0x9a4b, 0x06},
	{0x9a4c, 0x06},
	{0x9a4d, 0x06},
	{0x0112, 0x0a},
	{0x0113, 0x0a},
	{0x0114, 0x01},
	{0x0342, 0x1a},
	{0x0343, 0x08},
	{0x0340, 0x04},
	{0x0341, 0x1a},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x02},
	{0x0347, 0x10},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x09},
	{0x034b, 0xcf},
	{0x00e3, 0x00},
	{0x00e4, 0x00},
	{0x00fc, 0x0a},
	{0x00fd, 0x0a},
	{0x00fe, 0x0a},
	{0x00ff, 0x0a},
	{0xe013, 0x00},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x01},
	{0x3c02, 0x9c},
	{0x3f0d, 0x00},
	{0x5748, 0x00},
	{0x5749, 0x00},
	{0x574a, 0x00},
	{0x574b, 0xa4},
	{0x7b75, 0x0e},
	{0x7b76, 0x09},
	{0x7b77, 0x08},
	{0x7b78, 0x06},
	{0x7b79, 0x34},
	{0x7b53, 0x00},
	{0x9369, 0x73},
	{0x936b, 0x64},
	{0x936d, 0x5f},
	{0x9304, 0x03},
	{0x9305, 0x80},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0xa2a9, 0x27},
	{0xa2b7, 0x03},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x01},
	{0x0409, 0x5c},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x05},
	{0x040d, 0x34},
	{0x040e, 0x03},
	{0x040f, 0xde},
	{0x034c, 0x05},
	{0x034d, 0x34},
	{0x034e, 0x03},
	{0x034f, 0xde},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0xaf},
	{0x0309, 0x0a},
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x030e, 0x00},
	{0x030f, 0x96},
	{0x0310, 0x01},
	{0x0820, 0x07},
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x5f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x00},
	{0x3f57, 0xbf},
};



static const struct imx477_reg mode_1332x990_regs_8bits[] = {
	{0x420b, 0x01},
	{0x990c, 0x00},
	{0x990d, 0x08},
	{0x9956, 0x8c},
	{0x9957, 0x64},
	{0x9958, 0x50},
	{0x9a48, 0x06},
	{0x9a49, 0x06},
	{0x9a4a, 0x06},
	{0x9a4b, 0x06},
	{0x9a4c, 0x06},
	{0x9a4d, 0x06},
	{0x0112, 0x08},		// Changed to 08 from 0A
	{0x0113, 0x08},		// Changed to 08 from 0A
	{0x0114, 0x01},
	{0x0342, 0x1a},		// Line length
	{0x0343, 0x08},
	{0x0340, 0x04},		// IMX477_REG_FRAME_LENGTH
	{0x0341, 0x1a},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x02},
	{0x0347, 0x10},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x09},
	{0x034b, 0xcf},
	{0x00e3, 0x00},
	{0x00e4, 0x00},
	{0x00fc, 0x08},		// Try both 08 and 0A. Both work? Sticking to 08
	{0x00fd, 0x08},		// Try both 08 and 0A
	{0x00fe, 0x08},		// Try both 08 and 0A
	{0x00ff, 0x08},		// Try both 08 and 0A
	{0xe013, 0x00},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x01},
	{0x3c02, 0x9c},
	{0x3f0d, 0x00},
	{0x5748, 0x00},
	{0x5749, 0x00},
	{0x574a, 0x00},
	{0x574b, 0xa4},
	{0x7b75, 0x0e},
	{0x7b76, 0x09},
	{0x7b77, 0x08},
	{0x7b78, 0x06},
	{0x7b79, 0x34},
	{0x7b53, 0x00},
	{0x9369, 0x73},
	{0x936b, 0x64},
	{0x936d, 0x5f},
	{0x9304, 0x03},
	{0x9305, 0x80},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0xa2a9, 0x27},
	{0xa2b7, 0x03},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x01},
	{0x0409, 0x5c},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x05},
	{0x040d, 0x34},
	{0x040e, 0x03},
	{0x040f, 0xde},
	{0x034c, 0x05},			// x_output_size
	{0x034d, 0x34},
	{0x034e, 0x03},
	{0x034f, 0xde},			// y_output size
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0xaf},
	{0x0309, 0x08},			// Changed to 08 from 0A
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x030e, 0x00},
	{0x030f, 0x96},
	{0x0310, 0x01},
	{0x0820, 0x07},
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x5f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x00},
	{0x3f57, 0xbf},
};


/* 1080p cropped mode */
static const struct imx477_reg mode_2028x1080_regs_8b[] = {
	{0x0112, 0x08},		// Changed to 08 from 0A
	{0x0113, 0x08},		// Changed to 08 from 0A
	{0x0342, 0x31},		// Line length
	{0x0343, 0xc4},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x01},
	{0x0347, 0xb8},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x0a},
	{0x034b, 0x27},
	{0x00e3, 0x00},		// Added
	{0x00e4, 0x00},
	{0x00fc, 0x08},		// Try both 08 and 0A. Both work? Sticking to 08
	{0x00fd, 0x08},		// Try both 08 and 0A
	{0x00fe, 0x08},		// Try both 08 and 0A
	{0x00ff, 0x08},		// Try both 08 and 0A
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x03},
	{0x3c02, 0xa2},
	{0x3f0d, 0x01},
	{0x5748, 0x07},
	{0x5749, 0xff},
	{0x574a, 0x00},
	{0x574b, 0x00},
	{0x7b53, 0x01},
	{0x9369, 0x73},
	{0x936b, 0x64},
	{0x936d, 0x5f},
	{0x9304, 0x00},
	{0x9305, 0x00},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0xa2a9, 0x60},
	{0xa2b7, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x20},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f},
	{0x040d, 0xd8},
	{0x040e, 0x04},
	{0x040f, 0x38},
	{0x034c, 0x07},
	{0x034d, 0xec},
	{0x034e, 0x04},
	{0x034f, 0x38},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5e},
	{0x0309, 0x08},		// Changed to 08
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x030e, 0x00},
	{0x030f, 0x96},
	{0x0310, 0x01},
	{0x0820, 0x07},
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x7f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x01},
	{0x3f57, 0x6c},
};



/* 640x480 cropped mode */
static const struct imx477_reg mode_640x480_regs_8b[] = {
	{0x0112, 0x08},		// Changed to 08 from 0A
	{0x0113, 0x08},		// Changed to 08 from 0A
	{0x0342, 0x3E},		// Line length, was 0x31c4.  Use  (width + h_blanking)*5 xxx
	{0x0343, 0x74},		//  ---> gets overwritten anyway using line_length_pix
	{0x0344, 0x01},		// x_addr_start
	{0x0345, 0x00},
	{0x0346, 0x01},		// y_addr_start
	{0x0347, 0xb8},
	{0x0348, 0x03},		// x_addr_end
	{0x0349, 0x7F},
	{0x034a, 0x03},		// y_addr_end
	{0x034b, 0x97},
	{0x00e3, 0x00},		// Added
	{0x00e4, 0x00},
	{0x00fc, 0x0A},		// Try both 08 and 0A. Both work? Sticking to 08
	{0x00fd, 0x0A},		// Try both 08 and 0A
	{0x00fe, 0x0A},		// Try both 08 and 0A
	{0x00ff, 0x0A},		// Try both 08 and 0A
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x03},
	{0x3c02, 0xa2},
	{0x3f0d, 0x01},
	{0x5748, 0x07},
	{0x5749, 0xff},
	{0x574a, 0x00},
	{0x574b, 0x00},
	{0x7b53, 0x01},
	{0x9369, 0x73},
	{0x936b, 0x64},
	{0x936d, 0x5f},
	{0x9304, 0x00},
	{0x9305, 0x00},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0xa2a9, 0x60},
	{0xa2b7, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x20},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x02},		// Width after digital cropping
	{0x040d, 0x80},
	{0x040e, 0x01},		// Height after digital cropping
	{0x040f, 0xE0},
	{0x034c, 0x02},		// x output size
	{0x034d, 0x80},
	{0x034e, 0x01},		// y_output_size
	{0x034f, 0xe0},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5e},
	{0x0309, 0x08},		// Changed to 08
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x030e, 0x00},
	{0x030f, 0x96},
	{0x0310, 0x01},
	{0x0820, 0x07},
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x7f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x01},
	{0x3f57, 0x6c},
};

static const struct imx477_reg imx477_generic_nobinning_raw10_no_start_regs[] = {
    {0x0342, 0x5d},     // line_length_pck, Horizontal clock count, Line Length [15:8]
    {0x0343, 0xc0},     // Line Length [7:0]
    {0x0344, 0x00},     // x_addr_start
    {0x0345, 0x00},
    {0x0346, 0x00},     // y_addr_start
    {0x0347, 0x00},
    {0x0348, 0x0f},     // x_addr_end 
    {0x0349, 0xd7},
    {0x034a, 0x0b},     // y_addr_end 
    {0x034b, 0xdf},
    {0x00e3, 0x00},     // DOL-HDR disabled
    {0x00e4, 0x00},
    {0x00fc, 0x0a},     //  The output data fmt for CSI: RAW10      // FIXME: Remove and leave default for RAW12?
    {0x00fd, 0x0a},     //  The output data fmt for CSI: RAW10 
    {0x00fe, 0x0a},
    {0x00ff, 0x0a},
    {0x0220, 0x00},     // SME-HDR Mode
    {0x0221, 0x11},     // SME-HDR-Resolution reduction, h[3:0] and v[3:0]
    {0x0381, 0x01},     // Num of pixels skipped, even -> odd
    {0x0383, 0x01},     // Num of pixels skipped, odd -> even
    {0x0385, 0x01},     // Num of lines skipped, even -> odd
    {0x0387, 0x01},     // Num of lines skipped, odd -> even
    {0x0900, 0x00},     // binning_mod disabled
    {0x0901, 0x11},     // binning_type for H 
    {0x0902, 0x02},     // binning_weighting for V
    {0x3140, 0x02},
    {0x3c00, 0x00},
    {0x3c01, 0x03},
    {0x3c02, 0xa2},
    {0x3f0d, 0x01},     //  AD converter for RAW12?
    {0x5748, 0x07},
    {0x5749, 0xff},
    {0x574a, 0x00},
    {0x574b, 0x00},
    {0x7b75, 0x0a},
    {0x7b76, 0x0c},
    {0x7b77, 0x07},
    {0x7b78, 0x06},
    {0x7b79, 0x3c},
    {0x7b53, 0x01},
    {0x9369, 0x5a},
    {0x936b, 0x55},
    {0x936d, 0x28},
    {0x9304, 0x00},     // Not sure what this is -- can try 0x03 ?
    {0x9305, 0x00},
    {0x9e9a, 0x2f},
    {0x9e9b, 0x2f},
    {0x9e9c, 0x2f},
    {0x9e9d, 0x00},
    {0x9e9e, 0x00},
    {0x9e9f, 0x00},
    {0xa2a9, 0x60},
    {0xa2b7, 0x00},
    {0x0401, 0x00},     // Digital , no scaling
    {0x0404, 0x00},     // Down Scaling Factor M[8]
    {0x0405, 0x10},     // Scale factor [7:0]
    {0x0408, 0x00},     // Digital crop X _offset
    {0x0409, 0x00},     // X_offset
    {0x040a, 0x00},     // y_offset
    {0x040b, 0x00},     // y_offset
    {0x040c, 0x0f},     // Width after cropping 
    {0x040d, 0xd8},
    {0x040e, 0x0b},     // Height after cropping
    {0x040f, 0xe0},
    {0x034c, 0x0f},     //  X output size [12:8]
    {0x034d, 0xd8},     //  X output size [7:0]
    {0x034e, 0x0b},     //  Y output size [12:8]
    {0x034f, 0xe0},     //  Y output size [7:0]
    {0x0301, 0x05},     // The Pixel Clock Divider for IVTS 
    {0x0303, 0x02},     // The System Clock Divider for IVTS
    {0x0305, 0x04},     // The pre-PLL Clock Divider for IVTS
    {0x0306, 0x01},     // The PLL multiplier for IVTS [10:8]
    {0x0307, 0x5e},     // The PLL multiplier for IVTS [7:0]
    {0x0309, 0x0c},     //  The Pixel Clock Divider for IOP
    {0x030b, 0x02},     // The System Clock Divider for IOPS
    {0x030d, 0x02},     //  The pre-PLL Clock Divider for IOPS 
    {0x030e, 0x00},     // The PLL multiplier for IOPS [10:8]
    {0x030f, 0x96},     // The PLL multiplier for IOPS [7:0]
    {0x0310, 0x01},     // PLL mode select: Dual Mode       
    {0x0820, 0x07},     // Output Data Rate, Mbps  [31:24]
    {0x0821, 0x08},     // Output Data Rate, Mbps [23:16]  
    {0x0822, 0x00},     // Output Data Rate, Mbps [15:8]  
    {0x0823, 0x00},     // Output Data Rate, Mbps [7:0] 
    {0x080a, 0x00},   /* MIPI Global Timing (Tclk) [9:8]    */
    {0x080b, 0x7f},   /* MIPI Global Timing (Tclk) [7:0]    */
    {0x080c, 0x00},     // THS_PREPARE
    {0x080d, 0x4f},
    {0x080e, 0x00},
    {0x080f, 0x77},
    {0x0810, 0x00},
    {0x0811, 0x5f},
    {0x0812, 0x00},
    {0x0813, 0x57},
    {0x0814, 0x00},
    {0x0815, 0x4f},
    {0x0816, 0x01},
    {0x0817, 0x27},
    {0x0818, 0x00},
    {0x0819, 0x3f},
    {0xe04c, 0x00},
    {0xe04d, 0x7f},
    {0xe04e, 0x00},
    {0xe04f, 0x1f},
    {0x3e20, 0x01},
    {0x3e37, 0x00},
    {0x3f50, 0x00},     /* Power save?: Disable */
    {0x3f56, 0x02},     // Power save setting?
    {0x3f57, 0xae},     // Power save setting?
    {0x0101, 0x00},
    {0x0202, 0x0b},     // Exposure
    {0x0203, 0x80},
    {0x0204, 0x00},     // Gain
    {0x0205, 0x00},
    {0x0340, 0x0c},     // Frame Vertical Clock Count, Frame length  Frame length [15:8]
    {0x0341, 0x00},     //  Frame length [7:0]
    {0x0600, 0x00},
    {0x0601, 0x00},	 // No test pattern
};


/* Mode configs */
// This is not a const anymore, as we can change the width/height using ROI...
static struct imx477_mode supported_modes_12bit[] = {
	{
		/* 12MPix 10fps mode */
		.orig_width = 4056,
		.orig_height = 3040,
		
		.min_hblank  = 100,			// New to support highest fps possible, ignoring line_length_pix, timeperframe_min and timeperframe_default
		.min_vblank  = 42, //20,
		
		.line_length_pix = 0x5dc0,
		.crop = {
			.left = IMX477_PIXEL_ARRAY_LEFT,
			.top = IMX477_PIXEL_ARRAY_TOP,
			.width = 4056,
			.height = 3040,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 1000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 1000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4056x3040_regs),
			.regs = mode_4056x3040_regs,
		},
	},
	{
		/* 2x2 binned 40fps mode */
		.orig_width = 2028,
		.orig_height = 1520,
		
		.min_hblank  = 800,			// New to support highest fps possible, ignoring line_length_pix, timeperframe_min and timeperframe_default
		.min_vblank  = 32, //20,
		
		.line_length_pix = 0x31c4,
		.crop = {
			.left = IMX477_PIXEL_ARRAY_LEFT,
			.top = IMX477_PIXEL_ARRAY_TOP,
			.width = 4056,
			.height = 3040,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 4000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_2028x1520_regs),
			.regs = mode_2028x1520_regs,
		},
	},	
	{
		/* 1080p 50fps cropped mode */
		.orig_width = 2028,
		.orig_height = 1080,
		.min_hblank  = 800,			// New to support highest fps possible, ignoring line_length_pix, timeperframe_min and timeperframe_default
		.min_vblank  = 32, //20,
		.line_length_pix = 0x31c4,
		.crop = {
			.left = IMX477_PIXEL_ARRAY_LEFT,
			.top = IMX477_PIXEL_ARRAY_TOP + 440,
			.width = 4056,
			.height = 2160,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 5000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_2028x1080_regs),
			.regs = mode_2028x1080_regs,
		},
	}
};

static const struct imx477_mode supported_modes_10bit[] = {
	{
		/* 120fps. 2x2 binned and cropped */
		.orig_width = 1332,
		.orig_height = 990,
		.min_hblank  = 800,			// New to support highest fps possible, ignoring line_length_pix, timeperframe_min and timeperframe_default
		.min_vblank  = 32, //20,
		.line_length_pix = 6664,
		.crop = {
			/*
			 * FIXME: the analog crop rectangle is actually
			 * programmed with a horizontal displacement of 0
			 * pixels, not 4. It gets shrunk after going through
			 * the scaler. Move this information to the compose
			 * rectangle once the driver is expanded to represent
			 * its processing blocks with multiple subdevs.
			 */
			.left = IMX477_PIXEL_ARRAY_LEFT + 696,
			.top = IMX477_PIXEL_ARRAY_TOP + 528,
			.width = 2664,
			.height = 1980,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 12000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 12000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1332x990_regs),
			.regs = mode_1332x990_regs,
		}
	}
};

/**
Crop reg:
* 
* 	.x_start_reg = 0x0344,    // x_addr_start
	.x_start_reg_num_bits = 16,

	.y_start_reg = 0x0346,    // y_addr_start
	.y_start_reg_num_bits = 16,

	.x_end_reg = 0x0348,    // x_addr_end
	.x_end_reg_num_bits = 16,

	.y_end_reg = 0x034A,    // y_addr_end
	.y_end_reg_num_bits = 16,

	.x_size_reg = 0x034C,    // x_output_size
	.x_size_reg_num_bits = 16,

	.y_size_reg = 0x034E,    // y_output_size
	.y_size_reg_num_bits = 16,

	.x_dig_crop_size_reg = 0x040C,    // Width after digital cropping 
	.x_dig_crop_size_reg_num_bits = 16,

	.y_dig_crop_size_reg = 0x040E,    // Height after digital cropping 
	.y_dig_crop_size_reg_num_bits = 16,

	.line_len_reg = 0x0342,    // line_length_pck  to control fps
	.line_len_reg_num_bits = 16,

**/

static const struct imx477_mode supported_modes_8bit[] = {
	{
		.orig_width = 1332,
		.orig_height = 990,
		.min_hblank  = 800,			// New to support highest fps possible, ignoring line_length_pix, timeperframe_min and timeperframe_default
		.min_vblank  = 32, //20,
		.line_length_pix = 6664,
		.crop = {
			/*
			 * FIXME: the analog crop rectangle is actually
			 * programmed with a horizontal displacement of 0
			 * pixels, not 4. It gets shrunk after going through
			 * the scaler. Move this information to the compose
			 * rectangle once the driver is expanded to represent
			 * its processing blocks with multiple subdevs.
			 */
			.left = IMX477_PIXEL_ARRAY_LEFT + 696,
			.top = IMX477_PIXEL_ARRAY_TOP + 528,
			.width = 2664,
			.height = 1980,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 12000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 12000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1332x990_regs_8bits),
			.regs = mode_1332x990_regs_8bits,
		}
	},
	{
		/* 1080p 50fps cropped mode */
		.orig_width = 2028,
		.orig_height = 1080,
		.min_hblank  = 800,			// New to support highest fps possible, ignoring line_length_pix, timeperframe_min and timeperframe_default
		.min_vblank  = 32, //20,
		.line_length_pix = 0x31c4,
		// @SK: This is not really used or is just the analog crop, not the digital one
		// Keeping it full
		.crop = {
			.left = IMX477_PIXEL_ARRAY_LEFT,
			.top = IMX477_PIXEL_ARRAY_TOP + 440,
			.width = 4056,
			.height = 2160,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 5000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_2028x1080_regs_8b),
			.regs = mode_2028x1080_regs_8b,
		},
	},
	{
		/* 640x480 cropped mode */
		.orig_width = 640,
		.orig_height = 480,
		.min_hblank  = 800,			// New to support highest fps possible, ignoring line_length_pix, timeperframe_min and timeperframe_default
		.min_vblank  = 32, //20,
		.line_length_pix = 5200,		// Sets hblank and fps subsequently using a CTL call, reg 0x0342
		// @SK: This is not really used or is just the analog crop, not the digital one?
		// Keeping it full
		.crop = {
			.left = IMX477_PIXEL_ARRAY_LEFT,
			.top = IMX477_PIXEL_ARRAY_TOP + 440,
			//.width = 4056,
			//.height = 2160,
			.width = 640,
			.height = 480,
		},
		.timeperframe_min = {			// Sets 0x0340,
			.numerator = 100,
			.denominator = 5000
		},
		.timeperframe_default = {			// Sets 0x0340
			.numerator = 100,
			.denominator = 8000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_640x480_regs_8b),
			.regs = mode_640x480_regs_8b,
		},
	}	
};
/*
 * 
 * 
 * */



/*
 * The supported formats.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */
static const u32 codes[] = {
	/* 12-bit modes. */
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SBGGR12_1X12,
	/* 10-bit modes. */
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	/* 8-bit modes. */
	MEDIA_BUS_FMT_SRGGB8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SBGGR8_1X8,
	MEDIA_BUS_FMT_Y8_1X8,
	MEDIA_BUS_FMT_Y8_1X8,
	MEDIA_BUS_FMT_Y8_1X8,
	MEDIA_BUS_FMT_Y8_1X8,
	
};

static const char * const imx477_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
	"Solid Color",
	"Grey Color Bars",
	"PN9"
};

static const int imx477_test_pattern_val[] = {
	IMX477_TEST_PATTERN_DISABLE,
	IMX477_TEST_PATTERN_COLOR_BARS,
	IMX477_TEST_PATTERN_SOLID_COLOR,
	IMX477_TEST_PATTERN_GREY_COLOR,
	IMX477_TEST_PATTERN_PN9,
};

/* regulator supplies */
static const char * const imx477_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.05V) supply */
	"VDDL",  /* IF (1.8V) supply */
};

#define IMX477_NUM_SUPPLIES ARRAY_SIZE(imx477_supply_name)

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software standby), given by T7 in the
 * datasheet is 8ms.  This does include I2C setup time as well.
 *
 * Note, that delay between XCLR low->high and reading the CCI ID register (T6
 * in the datasheet) is much smaller - 600us.
 */
#define IMX477_XCLR_MIN_DELAY_US	8000
#define IMX477_XCLR_DELAY_RANGE_US	1000

struct imx477_compatible_data {
	unsigned int chip_id;
	struct imx477_reg_list extra_regs;
};

struct imx477 {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	unsigned int fmt_code;

	struct clk *xclk;
	u32 xclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[IMX477_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *binning_ctrl;
	struct v4l2_ctrl *roi_start_x;
	struct v4l2_ctrl *roi_start_y;
	struct v4l2_ctrl *i2c_8b_ctrl;
	struct v4l2_ctrl *i2c_16b_ctrl;
	struct v4l2_ctrl *i2c_8b_read_ctrl;
	struct v4l2_ctrl *i2c_16b_read_ctrl;
	struct v4l2_ctrl *i2c_8b_generic_ctrl;
	struct v4l2_ctrl *i2c_8b_generic_read_ctrl;
	struct v4l2_ctrl *i2c_generic_id_ctrl;
	struct v4l2_ctrl *i2c_read_addr_ctrl;


	/* Current mode */
	// Not read-only anymore since now we can change the size of the image using ROI
	const struct imx477_mode *mode;

	// New: ROI settings
	//uint16_t  roi_start_x;
	//uint16_t  roi_start_y;
	uint16_t  roi_width;
	uint16_t  roi_height;


	/* Trigger mode */
	int trigger_mode_of;
	struct v4l2_ctrl *force_trigger_ctrl;	// New alternative way to change the trigger

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

	/* Rewrite common registers on stream on? */
	bool common_regs_written;

	/* Current long exposure factor in use. Set through V4L2_CID_VBLANK */
	unsigned int long_exp_shift;
	
	uint16_t i2c_generic_id;
	uint16_t i2c_read_addr;

	/* Any extra information related to different compatible sensors */
	const struct imx477_compatible_data *compatible_data;
};

static inline struct imx477 *to_imx477(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx477, sd);
}

static inline void get_mode_table(unsigned int code,
				  const struct imx477_mode **mode_list,
				  unsigned int *num_modes)
{
	
	printk("imx477: %s() called\n", __func__);

	switch (code) {
	/* 12-bit */
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		*mode_list = supported_modes_12bit;
		*num_modes = ARRAY_SIZE(supported_modes_12bit);
		break;
	/* 10-bit */
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		*mode_list = supported_modes_10bit;
		*num_modes = ARRAY_SIZE(supported_modes_10bit);
		break;
	/* 8-bit */
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_Y8_1X8:
		*mode_list = supported_modes_8bit;
		*num_modes = ARRAY_SIZE(supported_modes_8bit);
		break;
	default:
		*mode_list = NULL;
		*num_modes = 0;
	}
}

/* Read registers up to 2 at a time */
static int imx477_read_reg(struct imx477 *imx477, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[4] = { 0, };
	int ret;

	if (len > 4)
		return -EINVAL;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}



// Generic I2C with different client id
static int i2c_client_read_reg(struct imx477 *imx477, u16 other_client_id, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[4] = { 0, };
	int ret;

	if (len > 4)
		return -EINVAL;

	// Write register address
	msgs[0].addr = other_client_id;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	// Read data from register 
	msgs[1].addr = other_client_id;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

    printk("imx477 read I2C addr 0x%x, reg 0x%X, val:0x%x\n",
		 client->addr, reg, *val);


	return 0;
}


/* Write registers up to 2 at a time */
static int imx477_write_reg(struct imx477 *imx477, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

/* Write a list of registers */
static int imx477_write_regs(struct imx477 *imx477,
			     const struct imx477_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = imx477_write_reg(imx477, regs[i].address, 1, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);

			return ret;
		}
	}

	return 0;
}

/* Get bayer order based on flip setting. */
static u32 imx477_get_format_code(struct imx477 *imx477, u32 code)
{
	unsigned int i;

	lockdep_assert_held(&imx477->mutex);

	for (i = 0; i < ARRAY_SIZE(codes); i++)
		if (codes[i] == code)
			break;

	if (i >= ARRAY_SIZE(codes))
		i = 0;

	i = (i & ~3) | (imx477->vflip->val ? 2 : 0) |
	    (imx477->hflip->val ? 1 : 0);

	return codes[i];
}

static void imx477_set_default_format(struct imx477 *imx477)
{
	/* Set default mode to max resolution */
	imx477->mode = &supported_modes_12bit[0];
	imx477->fmt_code = MEDIA_BUS_FMT_SRGGB12_1X12;
}


static int imx477_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx477 *imx477 = to_imx477(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_get_try_format(sd, fh->state, IMAGE_PAD);
	struct v4l2_mbus_framefmt *try_fmt_meta =
		v4l2_subdev_get_try_format(sd, fh->state, METADATA_PAD);
	struct v4l2_rect *try_crop;

	printk("imx477: %s() called\n", __func__);

	mutex_lock(&imx477->mutex);

	/* Initialize try_fmt for the image pad */
	imx477->roi_width   = supported_modes_12bit[0].orig_width;		// Starting values
	imx477->roi_height  = supported_modes_12bit[0].orig_height;
	//imx477->roi_start_x->val = 0;
	//imx477->roi_start_y->val = 0;
	try_fmt_img->width = imx477->roi_width;
	try_fmt_img->height = imx477->roi_height;
	try_fmt_img->code = imx477_get_format_code(imx477,
						   MEDIA_BUS_FMT_SRGGB12_1X12);
	try_fmt_img->field = V4L2_FIELD_NONE;

	/* Initialize try_fmt for the embedded metadata pad */
	try_fmt_meta->width = IMX477_EMBEDDED_LINE_WIDTH;
	try_fmt_meta->height = IMX477_NUM_EMBEDDED_LINES;		// Keep this sicne now we disabled metadata?
	try_fmt_meta->code = MEDIA_BUS_FMT_SENSOR_DATA;
	try_fmt_meta->field = V4L2_FIELD_NONE;

	/* Initialize try_crop */
	try_crop = v4l2_subdev_get_try_crop(sd, fh->state, IMAGE_PAD);		// Does this apply when analog cropping???
	/*
	try_crop->left = IMX477_PIXEL_ARRAY_LEFT;
	try_crop->top = IMX477_PIXEL_ARRAY_TOP;
	try_crop->width = IMX477_PIXEL_ARRAY_WIDTH;
	try_crop->height = IMX477_PIXEL_ARRAY_HEIGHT;
	*/
	// No cropping for now:
	try_crop->left = 0;
	try_crop->top = 0;
	try_crop->width = imx477->roi_width;
	try_crop->height = imx477->roi_height;

	mutex_unlock(&imx477->mutex);

	return 0;
}

static void imx477_adjust_exposure_range(struct imx477 *imx477)		// Not use anymore
{
	int exposure_max, exposure_def;

	/* Honour the VBLANK limits when setting exposure. */
	exposure_max = imx477->roi_height + imx477->vblank->val -
		       IMX477_EXPOSURE_OFFSET;
	exposure_def = min(exposure_max, imx477->exposure->val);
	__v4l2_ctrl_modify_range(imx477->exposure, imx477->exposure->minimum,
				 exposure_max, imx477->exposure->step,
				 exposure_def);
}

static void imx477_adjust_vblank_range_to_fit_exposure(struct imx477 *imx477)
{
	unsigned int vblank_min = umax(imx477->exposure->val + IMX477_EXPOSURE_OFFSET, imx477->mode->min_vblank);
	
	printk("imx477 %s() setting minimum vblank to exposure val: %d\n", __func__, vblank_min);
	
	__v4l2_ctrl_modify_range(imx477->vblank, vblank_min,
				 ((1 << IMX477_LONG_EXP_SHIFT_MAX) *
					IMX477_FRAME_LENGTH_MAX) - imx477->roi_height,
				 1, vblank_min);
	if (imx477->vblank->val < vblank_min) {
		printk("imx477 %s() increasing current vblank from %d to %d\n", __func__, imx477->vblank->val, vblank_min);
		__v4l2_ctrl_s_ctrl(imx477->vblank, vblank_min);
	}

}

static void imx477_set_ROI_size(struct imx477 *imx477)
{
	// Width has to be a multiple of 4 or 16 when binning
	// And the start has to be a multiple of 4...
	
	// By default the offset is in the middle of the previous size? Use zero for now...  
	if ((imx477->roi_width & 15)  != 0x00)
		printk("%s: width (=%d) should be a multiple of 16!\n", __func__, imx477->roi_width);
	if ((imx477->roi_height & 15)  != 0x00)
		printk("%s: width (=%d) should be a multiple of 16!\n", __func__, imx477->roi_height);
	if (imx477->roi_start_x != NULL)
		if ((imx477->roi_start_x->val & 3)  != 0x00)
			printk("%s: roi_start_x (=%d) should be a multiple of 4!\n", __func__, imx477->roi_start_x->val);
	if (imx477->roi_start_y != NULL)
		if ((imx477->roi_start_y->val & 3)  != 0x00)
			printk("%s: roi_start_y (=%d) should be a multiple of 4!\n", __func__, imx477->roi_start_y->val);

    uint16_t x_size  = imx477->roi_width;
    uint16_t y_size  = imx477->roi_height;
    uint16_t x_start = (imx477->roi_start_x == NULL) ? 0 : imx477->roi_start_x->val  & 0xFFFC;		// Needs to be a multiple of 4!
    uint16_t y_start = (imx477->roi_start_y == NULL) ? 0 : imx477->roi_start_y->val  & 0xFFFC;
    uint16_t x_end   = x_start + x_size*imx477->binning_ctrl->val - 1;
    uint16_t y_end   = y_start + y_size*imx477->binning_ctrl->val - 1;
    /*
    if (imx477->hflip->val)		// When flipping end and start are swapped
    {
		printk("%s():   Hflip, swapping x_start and x_end\n", __func__);
		uint16_t tmp = x_start;
		x_start = x_end;
		x_end = tmp;
	}
    if (imx477->vflip->val)		// When flipping end and start are swapped
    {
		printk("%s():   Vflip, swapping y_start and y_end\n", __func__);
		uint16_t tmp = y_start;
		y_start = y_end;
		y_end = tmp;
	}
	*/

    printk("%s(): Setting regs for width=%d height=%d ROI: {%d,%d}-->{%d,%d}\n", 
            __func__, x_size, y_size, x_start, y_start, x_end, y_end);
    imx477_write_reg(imx477, IMX477_X_START_REG,          IMX477_REG_VALUE_16BIT, x_start);
    imx477_write_reg(imx477, IMX477_Y_START_REG,          IMX477_REG_VALUE_16BIT, y_start);
    imx477_write_reg(imx477, IMX477_X_END_REG,            IMX477_REG_VALUE_16BIT, x_end);
    imx477_write_reg(imx477, IMX477_Y_END_REG,            IMX477_REG_VALUE_16BIT, y_end);
    imx477_write_reg(imx477, IMX477_X_SIZE_REG,           IMX477_REG_VALUE_16BIT, x_size);
    imx477_write_reg(imx477, IMX477_Y_SIZE_REG,           IMX477_REG_VALUE_16BIT, y_size);
    imx477_write_reg(imx477, IMX477_X_DIG_CROP_SIZE_REG,  IMX477_REG_VALUE_16BIT, x_size*imx477->binning_ctrl->val);
    imx477_write_reg(imx477, IMX477_Y_DIG_CROP_SIZE_REG,  IMX477_REG_VALUE_16BIT, y_size*imx477->binning_ctrl->val);
}

/*
static void imx477_update_ROI_offset(struct imx477 *imx477, unsigned int offset_x, unsigned int offset_y)
{
    uint16_t x_start = (uint16_t) offset_x;
    uint16_t y_start = (uint16_t) offset_y;
   
    uint16_t x_end   = x_start + imx477->mode->width - 1;
    uint16_t y_end   = y_start + imx477->mode->height - 1;
    uint16_t x_size  = imx477->mode->width;
    uint16_t y_size  = imx477->mode->height;
    printk("%s(): width=%d height=%d ROI: {%d,%d}-->{%d,%d}\n", 
            __func__, x_size, y_size, x_start, y_start, x_end, y_end);
    imx477_write_reg(imx477, IMX477_X_START_REG,          IMX477_REG_VALUE_16BIT, x_start);
    imx477_write_reg(imx477, IMX477_Y_START_REG,          IMX477_REG_VALUE_16BIT, y_start);
    imx477_write_reg(imx477, IMX477_X_END_REG,            IMX477_REG_VALUE_16BIT, x_end);
    imx477_write_reg(imx477, IMX477_Y_END_REG,            IMX477_REG_VALUE_16BIT, y_end);
    imx477_write_reg(imx477, IMX477_X_SIZE_REG,           IMX477_REG_VALUE_16BIT, x_size);
    imx477_write_reg(imx477, IMX477_Y_SIZE_REG,           IMX477_REG_VALUE_16BIT, y_size);
    imx477_write_reg(imx477, IMX477_X_DIG_CROP_SIZE_REG,  IMX477_REG_VALUE_16BIT, x_size);
    imx477_write_reg(imx477, IMX477_Y_DIG_CROP_SIZE_REG,  IMX477_REG_VALUE_16BIT, y_size);
   	
}
*/

static int imx477_set_frame_length(struct imx477 *imx477, unsigned int val)
{
	int ret = 0;

	imx477->long_exp_shift = 0;

	while (val > IMX477_FRAME_LENGTH_MAX) {
		imx477->long_exp_shift++;
		val >>= 1;
	}

	ret = imx477_write_reg(imx477, IMX477_REG_FRAME_LENGTH,
			       IMX477_REG_VALUE_16BIT, val);
			       
    printk("%s(): Set Frame length = %d\n", 
            __func__, val);
			       
	if (ret)
		return ret;

    printk("%s(): Set Long exposure shift = %d\n", 
            __func__, imx477->long_exp_shift);

	return imx477_write_reg(imx477, IMX477_LONG_EXP_SHIFT_REG,
				IMX477_REG_VALUE_08BIT, imx477->long_exp_shift);
}


// These are declared below but used here
static int imx477_power_on(struct device *dev);
static int imx477_power_off(struct device *dev);

static int imx477_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx477 *imx477 =
		container_of(ctrl->handler, struct imx477, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	int ret = 0;

	/*
	 * The VBLANK control may change the limits of usable exposure, so check
	 * and adjust if necessary.
	 */
	//if (ctrl->id == V4L2_CID_VBLANK)
	//	imx477_adjust_exposure_range(imx477);
	// But instead change the vblank according to the exposure
	
	//if (ctrl->id == V4L2_CID_EXPOSURE)
	//	imx477_adjust_vblank_range_to_fit_exposure(imx477);
	
	// --> Instead enabled the auto setting to make this inside the sensor
	
	
	

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	 bool power_off = pm_runtime_get_if_in_use(&client->dev) == 0;
	if ( power_off &&
	    ctrl->id != V4L2_CID_I2C_8B &&
	    ctrl->id != V4L2_CID_I2C_16B && 
	    ctrl->id != V4L2_CID_I2C_8B_GENERIC &&
	    ctrl->id != V4L2_CID_I2C_SET_GENERIC_ID &&
	    ctrl->id != V4L2_CID_I2C_SET_READ_ADDR ) {
		printk("imx477: set_ctrl id=0x%X val=%d not applied since power is off. But values like user hblank, etc should be saved anyway\n", ctrl->id, ctrl->val);
		//printk(" for dbg imx477->vblank->val=%d imx477->hblank->val=%d\n", imx477->vblank->val, imx477->hblank->val);
		return 0;
	}

	/*
	 * If doing and I2C write, only do it if val is not 0
	 * */
	if ((ctrl->id == V4L2_CID_I2C_8B  ||
	     ctrl->id == V4L2_CID_I2C_16B ||
	     ctrl->id == V4L2_CID_I2C_8B_GENERIC) &&
	     ctrl->val == 0)
	     {
			printk("imx477: imx477_set_ctrl() V4L2_CID_I2C_X: Ignoring op since val == 0 == 0x%X. power_off=%d\n", ctrl->val, power_off);
			return 0;
		 }




	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx477_write_reg(imx477, IMX477_REG_ANALOG_GAIN,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		printk("imx477: %s() V4L2_CID_ANALOGUE_GAIN: Set gain to  to %d\n", __func__, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx477_write_reg(imx477, IMX477_REG_EXPOSURE,
				       IMX477_REG_VALUE_16BIT, ctrl->val >>
							imx477->long_exp_shift);
		printk("imx477: %s() V4L2_CID_EXPOSURE: Set exposure to %d >> %d = %d\n", __func__, ctrl->val, imx477->long_exp_shift, ctrl->val >>
							imx477->long_exp_shift);
		printk("imx477: %s() V4L2_CID_EXPOSURE:    -->current frame length %d (vblank + roi_height)\n", __func__, imx477->vblank->val+ imx477->roi_height);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = imx477_write_reg(imx477, IMX477_REG_DIGITAL_GAIN,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx477_write_reg(imx477, IMX477_REG_TEST_PATTERN,
				       IMX477_REG_VALUE_16BIT,
				       imx477_test_pattern_val[ctrl->val]);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		ret = imx477_write_reg(imx477, IMX477_REG_TEST_PATTERN_R,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		ret = imx477_write_reg(imx477, IMX477_REG_TEST_PATTERN_GR,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		ret = imx477_write_reg(imx477, IMX477_REG_TEST_PATTERN_B,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		ret = imx477_write_reg(imx477, IMX477_REG_TEST_PATTERN_GB,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = imx477_write_reg(imx477, IMX477_REG_ORIENTATION, 1,
				       imx477->hflip->val |
				       imx477->vflip->val << 1);
		imx477_set_ROI_size(imx477);
		break;
	case V4L2_CID_VBLANK:
		printk("imx477: %s() V4L2_CID_VBLANK: Set fame length to total %d\n", __func__, imx477->roi_height + ctrl->val);
		ret = imx477_set_frame_length(imx477,
					      imx477->roi_height + ctrl->val);
		break;
	case V4L2_CID_HBLANK:
		ret = imx477_write_reg(imx477, IMX477_REG_LINE_LENGTH, 2,
				       imx477->roi_width + ctrl->val);
		printk("imx477: %s() V4L2_CID_HBLANK: Set line length to total %d\n", __func__, imx477->roi_width + ctrl->val);
		break;
	case V4L2_CID_BINNING:
		printk("imx477: %s() V4L2_CID_BINNING: Value set to %d, not setting done at beggining of start_streaming!\n", __func__, ctrl->val);
		break;

	case V4L2_CID_ROI_START_X:
		//ctrl->val = ctrl->val & 0xFFFC;
		// This depends on flipping, better to use imx477_set_ROI_size()
		//ret = imx477_write_reg(imx477, IMX477_X_START_REG, 2,
		//       (ctrl->val & 0xFFFC));
		printk("imx477: %s() V4L2_CID_ROI_START_X: Set ROI start X to %d, calling imx477_set_ROI_size()\n", __func__, (ctrl->val & 0xFFFC));
		//imx477->roi_start_x->val  = (ctrl->val & 0xFFFC);
		imx477_set_ROI_size(imx477);
		break;
	case V4L2_CID_ROI_START_Y:
		//ctrl->val = ctrl->val & 0xFFFC;
		// This depends on flipping, better to use imx477_set_ROI_size()
		//ret = imx477_write_reg(imx477, IMX477_Y_START_REG, 2,
		//		       (ctrl->val & 0xFFFC));
		printk("imx477: %s() V4L2_CID_ROI_START_Y: Set ROI start Y to %d, calling imx477_set_ROI_size()\n", __func__, (ctrl->val & 0xFFFC));
		//imx477->roi_start_y->val = (ctrl->val & 0xFFFC);
		imx477_set_ROI_size(imx477);
		break;

	case V4L2_CID_FORCE_TRIGGER:
		imx477->trigger_mode_of = ctrl->val;
		printk("imx477: %s() saved trigger_mode_of=%d. I2C ops done later part of imx477_start_streaming()\n", __func__, ctrl->val);
		break;

	case V4L2_CID_I2C_8B:   // Write 8b
		printk("imx477: imx477_set_ctrl() V4L2_CID_I2C_8B I2C write 8b reg 0x%X <== 0x%X\n",
			((ctrl->val>>16) & 0xFFFF), (ctrl->val & 0xFF) );
		ret = imx477_write_reg(imx477, ((ctrl->val>>16) & 0xFFFF), 1,
				       (ctrl->val & 0xFF));
		break;
	case V4L2_CID_I2C_16B:   // Write 16b
		printk("imx477: imx477_set_ctrl() V4L2_CID_I2C_16B I2C write 16b reg 0x%X <== 0x%X\n",
			((ctrl->val>>16) & 0xFFFF), (ctrl->val & 0xFFFF) );
		ret = imx477_write_reg(imx477, ((ctrl->val>>16) & 0xFFFF), 2,
				       (ctrl->val & 0xFFFF));
		break;
	case V4L2_CID_I2C_8B_GENERIC:
		printk("imx477: imx477_set_ctrl() V4L2_CID_I2C_8B_GENERIC I2C id=0x%X write 16b reg 0x%X <== 0x%X. NOT YET IMPLEMENTED\n",
			imx477->i2c_generic_id, ((ctrl->val>>16) & 0xFFFF), (ctrl->val & 0xFFFF) );
		ret = 0; //i2c_client_write_reg(imx477, imx477->i2c_generic_id, ((ctrl->val>>16) & 0xFFFF), 2,
				 //      (ctrl->val & 0xFFFF));
		break;
	case V4L2_CID_I2C_SET_GENERIC_ID:		// Set the ID for the generic I2C ops
		printk("imx477: imx477_set_ctrl() V4L2_CID_I2C_GENERIC_ID set ID to 0x%X\n",(ctrl->val & 0xFFFF));
		imx477->i2c_generic_id = (ctrl->val & 0xFFFF);
		ret = 0;
		break;
	case V4L2_CID_I2C_SET_READ_ADDR:
		printk("imx477: imx477_set_ctrl() V4L2_CID_I2C_READ_ADDR set addr to 0x%X\n",(ctrl->val & 0xFFFF));
		imx477->i2c_read_addr = (ctrl->val & 0xFFFF);
		ret = 0;
		break;
	case V4L2_CID_POWER_ON:   // Power on
		/*
		printk("imx477: imx477_set_ctrl() V4L2_CID_POWER ON with val=%d\n", ctrl->val);
		if (ctrl->val)
			ret = imx477_power_on(&client->dev);
		else
			ret = imx477_power_off(&client->dev);
		*/
		printk("imx477: imx477_set_ctrl() V4L2_CID_POWER ON disabled for now to avoid automatic call during __v4l2_ctrl_handler_setup. Called with val=%d\n", ctrl->val);
		break;
	case V4L2_CID_I2C_8B_READ:
		printk("imx477: imx477_set_ctrl() V4L2_CID_I2C_8B_READ. Read-only i2c op, not doing anyting\n");
		ret = 0;
		break;
	case V4L2_CID_I2C_16B_READ:
		printk("imx477: imx477_set_ctrl() V4L2_CID_I2C_16B_READ. Read-only i2c op, not doing anyting\n");
		ret = 0;
		break;
	case V4L2_CID_I2C_8B_GENERIC_READ:
		printk("imx477: imx477_set_ctrl() V4L2_CID_I2C_8B_GENERIC_READ. Read-only i2c op, not doing anyting\n");
		ret = 0;
		break;

	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}


static int imx477_get_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx477 *imx477 =
		container_of(ctrl->handler, struct imx477, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	int ret = 0;
	u32 val = 0;
	u16 reg_addr;

	printk("imx477: imx477_get_ctrl() called. ID=%d, name=%s val=%d cur=%d\n", ctrl->id, ctrl->name, ctrl->val, ctrl->cur.val );

	reg_addr = ((ctrl->val>>16) & 0xFFFF) + 0x16 /* FIXME */;

	switch (ctrl->id) {
		case V4L2_CID_I2C_8B:   // Read 8b		// Doesn't seem to work because not volatile?
			ret = imx477_read_reg(imx477, imx477->i2c_read_addr, 1, &val);
			printk("imx477: imx477_get_ctrl() V4L2_CID_I2C_8B I2C read reg 0x%X ==> 0x%X\n",
				imx477->i2c_read_addr, val);
			ctrl->val = (val & 0xFFFF);
			break;
		case V4L2_CID_I2C_16B:   // Read 16b 		// Doesn't seem to work because not volatile?
			ret = imx477_read_reg(imx477, imx477->i2c_read_addr, 2, &val);
			printk("imx477: imx477_get_ctrl() V4L2_CID_I2C_16B I2C read reg 0x%X ==> 0x%X\n",
				imx477->i2c_read_addr, val);
			ctrl->val =  (val & 0xFFFF);
			break;
		case V4L2_CID_I2C_8B_GENERIC_READ:
			ret = i2c_client_read_reg(imx477, imx477->i2c_generic_id, imx477->i2c_read_addr, 1, &val);
			printk("imx477: imx477_get_ctrl() V4L2_CID_I2C_8B_GENERIC_READ I2C ID=0x%X read reg 0x%X ==> 0x%X\n",
				imx477->i2c_generic_id, imx477->i2c_read_addr, val);
			ctrl->val =  (val & 0xFFFF);
			break;
		case V4L2_CID_I2C_8B_READ:   // Read 8b
			ret = imx477_read_reg(imx477, imx477->i2c_read_addr, 1, &val);
			printk("imx477: imx477_get_ctrl() V4L2_CID_I2C_8B_READ I2C read reg 0x%X ==> 0x%X\n",
				imx477->i2c_read_addr, val);
			ctrl->val =  (val);
			break;
		case V4L2_CID_I2C_16B_READ:   // Read-only 16b
			ret = imx477_read_reg(imx477, imx477->i2c_read_addr, 2, &val);
			printk("imx477: imx477_get_ctrl() V4L2_CID_I2C_16B_READ I2C read reg 0x%X ==> 0x%X\n",
				imx477->i2c_read_addr, val);
			ctrl->val =  (val);
			break;

		default:
			printk("imx477: imx477_get_ctrl() unkown CID 0x%X\n", ctrl->id);
			dev_info(&client->dev,
				 "ctrl(id:0x%x,val:0x%x) is not handled\n",
				 ctrl->id, ctrl->val);
			ret = -EINVAL;
			break;
	}

	//pm_runtime_put(&client->dev);

	return ret;

}
static const struct v4l2_ctrl_ops imx477_ctrl_ops = {
	.s_ctrl = imx477_set_ctrl,
	.g_volatile_ctrl = imx477_get_ctrl,
};

static int imx477_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx477 *imx477 = to_imx477(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD) {
		if (code->index >= (ARRAY_SIZE(codes) / 4))
			return -EINVAL;

		code->code = imx477_get_format_code(imx477,
						    codes[code->index * 4]);
	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int imx477_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx477 *imx477 = to_imx477(sd);
	printk("imx477 %s() called\n", __func__);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD) {
		const struct imx477_mode *mode_list;
		unsigned int num_modes;

		get_mode_table(fse->code, &mode_list, &num_modes);

		if (fse->index >= num_modes)
			return -EINVAL;

		if (fse->code != imx477_get_format_code(imx477, fse->code))
			return -EINVAL;

		fse->min_width = mode_list[fse->index].orig_width;
		fse->max_width = fse->min_width;
		fse->min_height = mode_list[fse->index].orig_height;
		fse->max_height = fse->min_height;
	} else {
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;

		fse->min_width = IMX477_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = IMX477_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static void imx477_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	printk("imx477 %s() called\n", __func__);
	printk("imx477 %s() New settings: colorspace=V4L2_COLORSPACE_RAW\n", __func__);
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void imx477_update_image_pad_format(struct imx477 *imx477,
					   const struct imx477_mode *mode,
					   struct v4l2_subdev_format *fmt)
{
	printk("imx477 %s() called\n", __func__);
	printk("imx477 %s() New settings: orig_width=%d, orig_height=%d Using roi_width=%d roi_height=%d field=V4L2_FIELD_NONE\n", __func__,
		mode->orig_width, mode->orig_height, imx477->roi_width, imx477->roi_height);
	fmt->format.width = imx477->roi_width;
	fmt->format.height = imx477->roi_height;
	fmt->format.field = V4L2_FIELD_NONE;
	imx477_reset_colorspace(&fmt->format);
}

static void imx477_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	printk("imx477 %s() called\n", __func__);
	fmt->format.width = IMX477_EMBEDDED_LINE_WIDTH;
	fmt->format.height = IMX477_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int imx477_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx477 *imx477 = to_imx477(sd);
	printk("imx477 %s() called\n", __func__);

	if (fmt->pad >= NUM_PADS) {
		printk("imx477 %s() pad number mismatch error: %d > %d \n", __func__, fmt->pad, NUM_PADS);
		return -EINVAL;
	}

	mutex_lock(&imx477->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(&imx477->sd, sd_state,
						   fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		try_fmt->code = fmt->pad == IMAGE_PAD ?
				imx477_get_format_code(imx477, try_fmt->code) :
				MEDIA_BUS_FMT_SENSOR_DATA;
		fmt->format = *try_fmt;
		printk("imx477 %s() updating format code after v4l2_subdev_get_try_format() to code=0x%X\n", __func__, try_fmt->code);
	} else {
		if (fmt->pad == IMAGE_PAD) {
			imx477_update_image_pad_format(imx477, imx477->mode,
						       fmt);
			fmt->format.code =
			       imx477_get_format_code(imx477, imx477->fmt_code);
			printk("imx477 %s() updating format code after imx477_update_image_pad_format() to code=0x%X\n", __func__, fmt->format.code);
		} else {
			imx477_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx477->mutex);
	return 0;
}

static
unsigned int imx477_get_frame_length(const struct imx477_mode *mode,
				     const struct v4l2_fract *timeperframe, unsigned int roi_height)
{
	u64 frame_length;
	printk("imx477 %s() called\n", __func__);

	frame_length = (u64)timeperframe->numerator * IMX477_PIXEL_RATE;
	do_div(frame_length,
	       (u64)timeperframe->denominator * mode->line_length_pix);

	if (WARN_ON(frame_length > IMX477_FRAME_LENGTH_MAX))
		frame_length = IMX477_FRAME_LENGTH_MAX;

	return max_t(unsigned int, frame_length, roi_height);
}

static void imx477_set_framing_limits(struct imx477 *imx477)
{
	unsigned int frm_length_min, frm_length_default, hblank_min;
	const struct imx477_mode *mode = imx477->mode;
	printk("imx477 %s() called\n", __func__);

	/*
	frm_length_min = imx477_get_frame_length(mode, &mode->timeperframe_min, imx477->roi_height);
	frm_length_default =
		     imx477_get_frame_length(mode, &mode->timeperframe_default, imx477->roi_height);
	__v4l2_ctrl_modify_range(imx477->vblank, frm_length_min - imx477->roi_height,
				 ((1 << IMX477_LONG_EXP_SHIFT_MAX) *
					IMX477_FRAME_LENGTH_MAX) - imx477->roi_height,
				 1, frm_length_default - imx477->roi_height);
	*/
	// Now just using min_hblank and min_vblank

	/* Default to no long exposure multiplier. */
	imx477->long_exp_shift = 0;

	/* Update limits and set FPS to default */
	// FIXME: Should we make the minimum limit the exposure value?
	__v4l2_ctrl_modify_range(imx477->vblank, mode->min_vblank,
				 ((1 << IMX477_LONG_EXP_SHIFT_MAX) *
					IMX477_FRAME_LENGTH_MAX) - imx477->roi_height,
				 1, mode->min_vblank);

	/* Setting this will adjust the exposure limits as well --> Not anymore!*/
	printk("imx477 %s() setting vblank to mode->min_vblank=%d\n", __func__, mode->min_vblank);
	__v4l2_ctrl_s_ctrl(imx477->vblank, mode->min_vblank);

	hblank_min = mode->min_hblank; //mode->line_length_pix - imx477->roi_width;
	__v4l2_ctrl_modify_range(imx477->hblank, hblank_min,
				 IMX477_LINE_LENGTH_MAX, 1, hblank_min);
	printk("imx477 %s() setting hblank to hblank_min=%d\n", __func__, hblank_min);
	__v4l2_ctrl_s_ctrl(imx477->hblank, hblank_min);
}

static int imx477_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;
	const struct imx477_mode *mode;
	struct imx477 *imx477 = to_imx477(sd);
	printk("imx477 %s() called\n", __func__);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx477->mutex);

	if (fmt->pad == IMAGE_PAD) {
		const struct imx477_mode *mode_list;
		unsigned int num_modes;

		/* Bayer order varies with flips */
		fmt->format.code = imx477_get_format_code(imx477,
							  fmt->format.code);

		get_mode_table(fmt->format.code, &mode_list, &num_modes);


		// To support any ROI resolution, we need to simply adjust it here

		mode = v4l2_find_nearest_size(mode_list,
					      num_modes,
					      orig_width, orig_height,
					      fmt->format.width,
					      fmt->format.height);
		printk("imx477 %s() nearest official mode is %dx%d, not switching to that, but using first mode with %dx%d and applying ROI instead to crop the current mode\n",
			__func__, mode->orig_width, mode->orig_height, mode_list[0].orig_width, mode_list[0].orig_height);
		//imx477_update_image_pad_format(imx477, mode, fmt);
		
		// Switch to the first mode matching the number of bits, and then change the ROI
		// This assumes that the first mode is always the one with the highest non-binning resolution
		printk("imx477 %s() binning is set to %d, chosing mode regs accordingly. FIXME: Using 0 for now\n", __func__, imx477->binning_ctrl->val);
		//imx477->mode = &mode_list[((imx477->binning_ctrl->val == 2) ? 1 : 0)];	// Only binning of 0 or 2
		imx477->mode = &mode_list[0];	// 0 for now
		imx477->fmt_code = fmt->format.code;
		
		imx477->roi_height = fmt->format.height;
		imx477->roi_width  = fmt->format.width;
		// Choose the middle when doing a set, then the client can make a call to change the start:
		if (imx477->roi_start_x != NULL)
			imx477->roi_start_x->val = ((imx477->mode->orig_width - imx477->roi_width)/2)  &  0xFFFC;	// Multiple of 4
		else
			printk("imx477 %s() imx477->roi_start_x is NULL!\n", __func__);
			
		if (imx477->roi_start_y != NULL)
			imx477->roi_start_y->val = ((imx477->mode->orig_height- imx477->roi_height)/2) &  0xFFFC ;	// Multiple of 4
		else
			printk("imx477 %s() imx477->roi_start_y is NULL!\n", __func__);
		
		imx477_set_framing_limits(imx477);
		
		
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else if (imx477->fmt_code != fmt->format.code) //imx477->mode != mode)
		{
			//imx477->mode = mode;		// No need to do this anymore
			imx477->fmt_code = fmt->format.code;
			imx477_set_framing_limits(imx477);
		}
	} else {
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else {
			/* Only one embedded data mode is supported */
			imx477_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx477->mutex);

	return 0;
}

static const struct v4l2_rect *
__imx477_get_pad_crop(struct imx477 *imx477,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	printk("imx477 %s() called\n", __func__);
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&imx477->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx477->mode->crop;
	}

	return NULL;
}

static int imx477_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	printk("imx477 %s() called\n", __func__);
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct imx477 *imx477 = to_imx477(sd);

		printk("imx477 %s() returning current copping selection\n", __func__);
		mutex_lock(&imx477->mutex);
		sel->r = *__imx477_get_pad_crop(imx477, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&imx477->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		printk("imx477 %s() returning native uncropped size\n", __func__);
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX477_NATIVE_WIDTH;
		sel->r.height = IMX477_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		printk("imx477 %s() returning default\n", __func__);
		sel->r.left = IMX477_PIXEL_ARRAY_LEFT;
		sel->r.top = IMX477_PIXEL_ARRAY_TOP;
		sel->r.width = IMX477_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX477_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

/* Start streaming */
static int imx477_start_streaming(struct imx477 *imx477)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	const struct imx477_reg_list *reg_list;
	const struct imx477_reg_list *extra_regs;
	int ret, tm;
	printk("imx477 %s() called\n", __func__);

	if (!imx477->common_regs_written) {
		printk("imx477 %s() writting all the common I2C registers to the sensor (num_of_regs=%ld)\n", __func__, ARRAY_SIZE(mode_common_regs));
		ret = imx477_write_regs(imx477, mode_common_regs,
					ARRAY_SIZE(mode_common_regs));
		if (!ret) {
			extra_regs = &imx477->compatible_data->extra_regs;
			printk("imx477 %s() writting all the extra I2C registers to the sensor (num_of_regs=%d)\n", __func__, extra_regs->num_of_regs);
			ret = imx477_write_regs(imx477,	extra_regs->regs,
						extra_regs->num_of_regs);
		}

		if (ret) {
			dev_err(&client->dev, "%s failed to set common settings\n",
				__func__);
			return ret;
		}
		imx477->common_regs_written = true;
	}



	printk("imx477 %s() Chosen mode regs according to binning = %d\n", __func__, imx477->binning_ctrl->val);
	imx477->mode = &supported_modes_12bit[((imx477->binning_ctrl->val == 2) ? 1 : 0)];

	/* Apply default values of current mode */
	reg_list = &imx477->mode->reg_list;
	printk("imx477 %s() writting all the mode-specific I2C registers to the sensor (num_of_regs=%d)\n", __func__, reg_list->num_of_regs);
	ret = imx477_write_regs(imx477, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	// Adjust ROI function
	printk("imx477 %s() Updating the ROI registers\n", __func__);
	imx477_set_ROI_size(imx477);

	/* Set on-sensor DPC. */
	imx477_write_reg(imx477, 0x0b05, IMX477_REG_VALUE_08BIT, !!dpc_enable);
	imx477_write_reg(imx477, 0x0b06, IMX477_REG_VALUE_08BIT, !!dpc_enable);

	/* Apply customized values from user */
	// NOTE this will call imx477_adjust_exposure_range() subsequently (since the exposure is set by default) and set many other parameters the 
	// The full ctrl_handler sequence of settings all the defaults defined in imx477_init_controls  (or whatever got updated by the user)
	printk("imx477: calling __v4l2_ctrl_handler_setup(imx477->sd.ctrl_handler, all settings defined in imx477_init_controls)\n");
	ret =  __v4l2_ctrl_handler_setup(imx477->sd.ctrl_handler);
	if (ret) {
		printk("imx477:   __v4l2_ctrl_handler_setup()  failed ret=%d\n", ret);
		return ret;
	}
	/* Set vsync trigger mode: 0=standalone, 1=source, 2=sink */
	tm = (imx477->trigger_mode_of >= 0) ? imx477->trigger_mode_of : trigger_mode;
	printk("imx477 %s() Writing sync-trigger regs for tm=%d\n", __func__, tm);
	imx477_write_reg(imx477, 0x0350,
			 IMX477_REG_VALUE_08BIT, (tm > 0) ? 0 : 1);	// Need to disable auto frame length adj for multi=camera
	imx477_write_reg(imx477, IMX477_REG_MC_MODE,
			 IMX477_REG_VALUE_08BIT, (tm > 0) ? 1 : 0);
	imx477_write_reg(imx477, IMX477_REG_MS_SEL,
			 IMX477_REG_VALUE_08BIT, (tm <= 1) ? 1 : 0);
	imx477_write_reg(imx477, IMX477_REG_XVS_IO_CTRL,
			 IMX477_REG_VALUE_08BIT, (tm == 1) ? 1 : 0);
	imx477_write_reg(imx477, IMX477_REG_EXTOUT_EN,
			 IMX477_REG_VALUE_08BIT, (tm == 1) ? 1 : 0);

	/* set stream on register */
	return imx477_write_reg(imx477, IMX477_REG_MODE_SELECT,
				IMX477_REG_VALUE_08BIT, IMX477_MODE_STREAMING);
}

/* Stop streaming */
static void imx477_stop_streaming(struct imx477 *imx477)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	int ret;
	printk("imx477 %s() called\n", __func__);

	/* set stream off register */
	ret = imx477_write_reg(imx477, IMX477_REG_MODE_SELECT,
			       IMX477_REG_VALUE_08BIT, IMX477_MODE_STANDBY);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);

	/* Stop driving XVS out (there is still a weak pull-up) */
	imx477_write_reg(imx477, IMX477_REG_EXTOUT_EN,
			 IMX477_REG_VALUE_08BIT, 0);
}

static int imx477_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx477 *imx477 = to_imx477(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	printk("imx477 %s() called with enable=%d\n", __func__, enable);

	mutex_lock(&imx477->mutex);
	if (imx477->streaming == enable) {
		mutex_unlock(&imx477->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto err_unlock;
		}

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = imx477_start_streaming(imx477);
		if (ret)
			goto err_rpm_put;
	} else {
		imx477_stop_streaming(imx477);
		pm_runtime_put(&client->dev);
	}

	imx477->streaming = enable;

	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(imx477->vflip, enable);
	__v4l2_ctrl_grab(imx477->hflip, enable);

	mutex_unlock(&imx477->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&imx477->mutex);

	return ret;
}



/* Power/clock management functions */
static int imx477_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx477 *imx477 = to_imx477(sd);
	int ret;
	printk("imx477 %s() called\n", __func__);

	ret = regulator_bulk_enable(IMX477_NUM_SUPPLIES,
				    imx477->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(imx477->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	gpiod_set_value_cansleep(imx477->reset_gpio, 1);
	usleep_range(IMX477_XCLR_MIN_DELAY_US,
		     IMX477_XCLR_MIN_DELAY_US + IMX477_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(IMX477_NUM_SUPPLIES, imx477->supplies);
	return ret;
}

static int imx477_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx477 *imx477 = to_imx477(sd);
	printk("imx477 %s() called\n", __func__);

	gpiod_set_value_cansleep(imx477->reset_gpio, 0);
	regulator_bulk_disable(IMX477_NUM_SUPPLIES, imx477->supplies);
	clk_disable_unprepare(imx477->xclk);

	/* Force reprogramming of the common registers when powered up again. */
	imx477->common_regs_written = false;

	return 0;
}

static int __maybe_unused imx477_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx477 *imx477 = to_imx477(sd);

	if (imx477->streaming)
		imx477_stop_streaming(imx477);

	return 0;
}

static int __maybe_unused imx477_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx477 *imx477 = to_imx477(sd);
	int ret;

	if (imx477->streaming) {
		ret = imx477_start_streaming(imx477);
		if (ret)
			goto error;
	}

	return 0;

error:
	imx477_stop_streaming(imx477);
	imx477->streaming = 0;
	return ret;
}

static int imx477_get_regulators(struct imx477 *imx477)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	unsigned int i;
	printk("imx477 %s() called\n", __func__);

	for (i = 0; i < IMX477_NUM_SUPPLIES; i++)
		imx477->supplies[i].supply = imx477_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       IMX477_NUM_SUPPLIES,
				       imx477->supplies);
}

/* Verify chip ID */
static int imx477_identify_module(struct imx477 *imx477, u32 expected_id)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	int ret;
	u32 val;
	printk("imx477 %s() called\n", __func__);

	ret = imx477_read_reg(imx477, IMX477_REG_CHIP_ID,
			      IMX477_REG_VALUE_16BIT, &val);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id %x, with error %d\n",
			expected_id, ret);
		return ret;
	}

	if (val != expected_id) {
		dev_err(&client->dev, "chip id mismatch: %x!=%x\n",
			expected_id, val);
		return -EIO;
	}

	dev_info(&client->dev, "Device found is imx%x\n", val);

	return 0;
}

static const struct v4l2_subdev_core_ops imx477_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx477_video_ops = {
	.s_stream = imx477_set_stream,
};

static const struct v4l2_subdev_pad_ops imx477_pad_ops = {
	.enum_mbus_code = imx477_enum_mbus_code,
	.get_fmt = imx477_get_pad_format,
	.set_fmt = imx477_set_pad_format,
	.get_selection = imx477_get_selection,
	.enum_frame_size = imx477_enum_frame_size,
};

static const struct v4l2_subdev_ops imx477_subdev_ops = {
	.core = &imx477_core_ops,
	.video = &imx477_video_ops,
	.pad = &imx477_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx477_internal_ops = {
	.open = imx477_open,
};

/* Initialize control handlers */
static const struct v4l2_ctrl_config i2c_8b_ctrl = {
        .ops = &imx477_ctrl_ops,
        .name = "V4L2_I2C_8B_OP",
        .id = V4L2_CID_I2C_8B,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .min=-0x7FFFFFFF,
        .max = 0x7FFFFFFF,
        .step = 1,
        .def = 0,
};

static const struct v4l2_ctrl_config i2c_16b_ctrl = {
        .ops = &imx477_ctrl_ops,
        .name = "V4L2_I2C_16B_OP",
        .id = V4L2_CID_I2C_16B,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .min=-0x7FFFFFFF,
        .max = 0x7FFFFFFF,
        .step = 1,
        .def = 0,
};
static const struct v4l2_ctrl_config i2c_8b_read_ctrl = {
        .ops = &imx477_ctrl_ops,
        .name = "V4L2_I2C_8B_OP_READ",
        .id = V4L2_CID_I2C_8B_READ,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .min=-0x7FFFFFFF,
        .max = 0x7FFFFFFF,
        .step = 1,
        .def = 0,
};
static const struct v4l2_ctrl_config i2c_16b_read_ctrl = {
        .ops = &imx477_ctrl_ops,
        .name = "V4L2_I2C_16B_OP_READ",
        .id = V4L2_CID_I2C_16B_READ,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .min=-0x7FFFFFFF,
        .max = 0x7FFFFFFF,
        .step = 1,
        .def = 0,
};
static const struct v4l2_ctrl_config i2c_8b_generic_ctrl = {
        .ops = &imx477_ctrl_ops,
        .name = "V4L2_I2C_8B_Generic_OP",
        .id = V4L2_CID_I2C_8B_GENERIC,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .min=-0x7FFFFFFF,
        .max = 0x7FFFFFFF,
        .step = 1,
        .def = 0,
};
static const struct v4l2_ctrl_config i2c_8b_generic_read_ctrl = {
        .ops = &imx477_ctrl_ops,
        .name = "V4L2_I2C_8B_Generic_read_OP",
        .id = V4L2_CID_I2C_8B_GENERIC_READ,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .min=-0x7FFFFFFF,
        .max = 0x7FFFFFFF,
        .step = 1,
        .def = 0,
};
static const struct v4l2_ctrl_config power_on_ctrl = {
        .ops = &imx477_ctrl_ops,
        .name = "PowerOn",
        .id = V4L2_CID_POWER_ON,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .min = 0,
        .max = 1,
        .step = 1,
        .def = 0,
};
static const struct v4l2_ctrl_config i2c_generic_id_ctrl = {
        .ops = &imx477_ctrl_ops,
        .name = "V4L2_I2C_set_Generic_ID_OP",
        .id = V4L2_CID_I2C_SET_GENERIC_ID,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .min=0,
        .max = 0xFFF,		// 11 bits max?
        .step = 1,
        .def = 0,
};
static const struct v4l2_ctrl_config i2c_read_addr_ctrl = {
        .ops = &imx477_ctrl_ops,
        .name = "V4L2_I2C_set_read_addr_OP",
        .id = V4L2_CID_I2C_SET_READ_ADDR,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .min=0,
        .max = 0xFFFF,
        .step = 1,
        .def = 0,
};


static const struct v4l2_ctrl_config roi_start_x_ctrl = {
        .ops = &imx477_ctrl_ops,
        .name = "V4L2_ROI_start_x",
        .id = V4L2_CID_ROI_START_X,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .min=0,
        .max = 0xFFFF,
        .step = 4,
        .def = 0,
};
static const struct v4l2_ctrl_config roi_start_y_ctrl = {
        .ops = &imx477_ctrl_ops,
        .name = "V4L2_ROI_start_y",
        .id = V4L2_CID_ROI_START_Y,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .min=0,
        .max = 0xFFFF,
        .step = 4,
        .def = 0,
};

static const struct v4l2_ctrl_config force_trigger_ctrl = {
        .ops = &imx477_ctrl_ops,
        .name = "V4L2_force_trigger",
        .id = V4L2_CID_FORCE_TRIGGER,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .min=0,
        .max = 2,
        .step = 1,
        .def = 0,
};

static const struct v4l2_ctrl_config binning_ctrl = {
        .ops = &imx477_ctrl_ops,
        .name = "V4L2_binning",
        .id = V4L2_CID_BINNING,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .min=1,
        .max = 2,
        .step = 1,
        .def = 1,
};



static int imx477_init_controls(struct imx477 *imx477)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	struct v4l2_fwnode_device_properties props;
	unsigned int i;
	int ret;
	printk("imx477 %s() called\n", __func__);

	ctrl_hdlr = &imx477->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16+9);  // 9 extra custom commands
	if (ret)
		return ret;

	mutex_init(&imx477->mutex);
	ctrl_hdlr->lock = &imx477->mutex;

	/* By default, PIXEL_RATE is read only */
	imx477->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       IMX477_PIXEL_RATE,
					       IMX477_PIXEL_RATE, 1,
					       IMX477_PIXEL_RATE);
	if (imx477->pixel_rate)
		imx477->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* LINK_FREQ is also read only */
	imx477->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, &imx477_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(imx477_link_freq_menu) - 1, 0,
				       imx477_link_freq_menu);
	if (imx477->link_freq)
		imx477->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * Create the controls here, but mode specific limits are setup
	 * in the imx477_set_framing_limits() call below.
	 */
	imx477->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					   V4L2_CID_VBLANK, 0, 0xffff, 1, 0);
	imx477->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0xffff, 1, 0);
					   
	imx477->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX477_EXPOSURE_MIN,
					     IMX477_EXPOSURE_MAX,
					     IMX477_EXPOSURE_STEP,
					     IMX477_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX477_ANA_GAIN_MIN, IMX477_ANA_GAIN_MAX,
			  IMX477_ANA_GAIN_STEP, IMX477_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX477_DGTL_GAIN_MIN, IMX477_DGTL_GAIN_MAX,
			  IMX477_DGTL_GAIN_STEP, IMX477_DGTL_GAIN_DEFAULT);

	imx477->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (imx477->hflip)
		imx477->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	imx477->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (imx477->vflip)
		imx477->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx477_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx477_test_pattern_menu) - 1,
				     0, 0, imx477_test_pattern_menu);
	for (i = 0; i < 4; i++) {
		/*
		 * The assumption is that
		 * V4L2_CID_TEST_PATTERN_GREENR == V4L2_CID_TEST_PATTERN_RED + 1
		 * V4L2_CID_TEST_PATTERN_BLUE   == V4L2_CID_TEST_PATTERN_RED + 2
		 * V4L2_CID_TEST_PATTERN_GREENB == V4L2_CID_TEST_PATTERN_RED + 3
		 */
		v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  IMX477_TEST_PATTERN_COLOUR_MIN,
				  IMX477_TEST_PATTERN_COLOUR_MAX,
				  IMX477_TEST_PATTERN_COLOUR_STEP,
				  IMX477_TEST_PATTERN_COLOUR_MAX);
		/* The "Solid color" pattern is white by default */
	}

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	// New custom i2c controls, after the error check:
	// Mark them volatile to allow reading back
	imx477->i2c_8b_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr, &i2c_8b_ctrl, NULL);
	if (imx477->i2c_8b_ctrl)
	    imx477->i2c_8b_ctrl->flags |= 0; //V4L2_CTRL_FLAG_VOLATILE;   // Volatile doesn't work for writing...
	else
		printk("imx477: imx477_init_controls() v4l2_ctrl_new_std(V4L2_CID_I2C_8B) FAILED err=%d\n", ctrl_hdlr->error);
		
	imx477->i2c_16b_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr, &i2c_16b_ctrl, NULL);
	if(imx477->i2c_16b_ctrl)
	   imx477->i2c_16b_ctrl->flags |= 0; //V4L2_CTRL_FLAG_VOLATILE;
	else
		printk("imx477: imx477_init_controls() v4l2_ctrl_new_std(V4L2_CID_I2C_16B) FAILED err=%d\n", ctrl_hdlr->error);

	imx477->i2c_8b_generic_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr, &i2c_8b_generic_ctrl, NULL);
	if(imx477->i2c_8b_generic_ctrl)
	   imx477->i2c_8b_generic_ctrl->flags |= 0; //V4L2_CTRL_FLAG_VOLATILE;		// Volatile doesn't work for writing...
	else
	   printk("imx477: imx477_init_controls() v4l2_ctrl_new_std(V4L2_CID_I2C_8B_GENERIC_READ) FAILED err=%d\n", ctrl_hdlr->error);

	imx477->i2c_8b_generic_read_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr, &i2c_8b_generic_read_ctrl, NULL);
	if(imx477->i2c_8b_generic_read_ctrl)
	   imx477->i2c_8b_generic_read_ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY;
	else
	   printk("imx477: imx477_init_controls() v4l2_ctrl_new_std(V4L2_CID_I2C_8B_GENERIC) FAILED err=%d\n", ctrl_hdlr->error);

		
	imx477->i2c_generic_id_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr, &i2c_generic_id_ctrl, NULL);
	if (imx477->i2c_generic_id_ctrl == NULL)
	   printk("imx477: imx477_init_controls() v4l2_ctrl_new_std(V4L2_CID_I2C_SET_GENERIC_ID) FAILED err=%d\n", ctrl_hdlr->error);

	
	imx477->i2c_read_addr_ctrl  = v4l2_ctrl_new_custom(ctrl_hdlr, &i2c_read_addr_ctrl, NULL);
	if (imx477->i2c_read_addr_ctrl == NULL)
	   printk("imx477: imx477_init_controls() v4l2_ctrl_new_std(V4L2_CID_I2C_SET_READ_ADDR) FAILED err=%d\n", ctrl_hdlr->error);

		
	if (v4l2_ctrl_new_custom(ctrl_hdlr, &power_on_ctrl, NULL) == NULL)
		printk("imx477: imx477_init_controls() v4l2_ctrl_new_std(V4L2_CID_POWER_ON) FAILED err=%d\n", ctrl_hdlr->error);


	imx477->i2c_8b_read_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr, &i2c_8b_read_ctrl, NULL);
	if(imx477->i2c_8b_read_ctrl) {
	   imx477->i2c_8b_read_ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY;
        }
	else
		printk("imx477: imx477_init_controls() v4l2_ctrl_new_std(V4L2_CID_I2C_8B_READ) FAILED err=%d\n", ctrl_hdlr->error);

	imx477->i2c_16b_read_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr, &i2c_16b_read_ctrl, NULL);
	if(imx477->i2c_16b_read_ctrl) {
	   imx477->i2c_16b_read_ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY;
        }
	else
		printk("imx477: imx477_init_controls() v4l2_ctrl_new_std(V4L2_CID_I2C_16B_READ) FAILED err=%d\n", ctrl_hdlr->error);


	imx477->roi_start_x = v4l2_ctrl_new_custom(ctrl_hdlr, &roi_start_x_ctrl, NULL);
	if (imx477->roi_start_x == NULL)
	   printk("imx477: imx477_init_controls() v4l2_ctrl_new_custom(roi_start_x_ctrl) FAILED err=%d\n", ctrl_hdlr->error);


	imx477->roi_start_y = v4l2_ctrl_new_custom(ctrl_hdlr, &roi_start_y_ctrl, NULL);
	if (imx477->roi_start_y == NULL)
	   printk("imx477: imx477_init_controls() v4l2_ctrl_new_custom(roi_start_y_ctrl) FAILED err=%d\n", ctrl_hdlr->error);


	imx477->force_trigger_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr, &force_trigger_ctrl, NULL);
	if (imx477->force_trigger_ctrl == NULL)
	   printk("imx477: imx477_init_controls() v4l2_ctrl_new_custom(force_trigger_ctrl) FAILED err=%d\n", ctrl_hdlr->error);


	imx477->binning_ctrl = v4l2_ctrl_new_custom(ctrl_hdlr, &binning_ctrl, NULL);
	if (imx477->binning_ctrl == NULL)
	   printk("imx477: imx477_init_controls() v4l2_ctrl_new_custom(binning_ctrl) FAILED err=%d\n", ctrl_hdlr->error);

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx477_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	imx477->sd.ctrl_handler = ctrl_hdlr;

	mutex_lock(&imx477->mutex);

	/* Setup exposure and frame/line length limits. */
	imx477_set_framing_limits(imx477);

	mutex_unlock(&imx477->mutex);

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx477->mutex);

	return ret;
}

static void imx477_free_controls(struct imx477 *imx477)
{
	v4l2_ctrl_handler_free(imx477->sd.ctrl_handler);
	mutex_destroy(&imx477->mutex);
}

static int imx477_check_hwcfg(struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret = -EINVAL;
	printk("imx477 %s() called\n", __func__);

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* Check the number of MIPI CSI2 data lanes */
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2) {
		dev_err(dev, "only 2 data lanes are currently supported\n");
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	if (ep_cfg.nr_of_link_frequencies != 1 ||
	    ep_cfg.link_frequencies[0] != IMX477_DEFAULT_LINK_FREQ) {
		dev_err(dev, "Link frequency not supported: %lld\n",
			ep_cfg.link_frequencies[0]);
		goto error_out;
	}

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static const struct imx477_compatible_data imx477_compatible = {
	.chip_id = IMX477_CHIP_ID,
	.extra_regs = {
		.num_of_regs = 0,
		.regs = NULL
	}
};

static const struct imx477_reg imx378_regs[] = {
	{0x3e35, 0x01},
	{0x4421, 0x08},
	{0x3ff9, 0x00},
};

static const struct imx477_compatible_data imx378_compatible = {
	.chip_id = IMX378_CHIP_ID,
	.extra_regs = {
		.num_of_regs = ARRAY_SIZE(imx378_regs),
		.regs = imx378_regs
	}
};

static const struct of_device_id imx477_dt_ids[] = {
	{ .compatible = "sony,imx477", .data = &imx477_compatible },
	{ .compatible = "sony,imx378", .data = &imx378_compatible },
	{ /* sentinel */ }
};

static int imx477_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx477 *imx477;
	const struct of_device_id *match;
	int ret;
	u32 tm_of;
	printk("imx477 %s() called\n", __func__);

	imx477 = devm_kzalloc(&client->dev, sizeof(*imx477), GFP_KERNEL);
	if (!imx477)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx477->sd, client, &imx477_subdev_ops);

	match = of_match_device(imx477_dt_ids, dev);
	if (!match)
		return -ENODEV;
	imx477->compatible_data =
		(const struct imx477_compatible_data *)match->data;

	/* Check the hardware configuration in device tree */
	if (imx477_check_hwcfg(dev))
		return -EINVAL;

	/* Default the trigger mode from OF to -1, which means invalid */
	ret = of_property_read_u32(dev->of_node, "trigger-mode", &tm_of);
	imx477->trigger_mode_of = (ret == 0) ? tm_of : -1;

	/* Get system clock (xclk) */
	imx477->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx477->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(imx477->xclk);
	}

	imx477->xclk_freq = clk_get_rate(imx477->xclk);
	if (imx477->xclk_freq != IMX477_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			imx477->xclk_freq);
		return -EINVAL;
	}

	ret = imx477_get_regulators(imx477);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	imx477->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	/*
	 * The sensor must be powered for imx477_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = imx477_power_on(dev);
	if (ret)
		return ret;

	ret = imx477_identify_module(imx477, imx477->compatible_data->chip_id);
	if (ret)
		goto error_power_off;

	/* Initialize default format */
	imx477_set_default_format(imx477);

	/* Enable runtime PM and turn off the device , see https://www.kernel.org/doc/Documentation/power/runtime_pm.txt */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	/* This needs the pm runtime to be registered. */
	ret = imx477_init_controls(imx477);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	imx477->sd.internal_ops = &imx477_internal_ops;
	imx477->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	imx477->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	imx477->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	imx477->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx477->sd.entity, NUM_PADS, imx477->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&imx477->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	return 0;

error_media_entity:
	media_entity_cleanup(&imx477->sd.entity);

error_handler_free:
	imx477_free_controls(imx477);

error_power_off:
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	imx477_power_off(&client->dev);

	return ret;
}

static void imx477_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx477 *imx477 = to_imx477(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx477_free_controls(imx477);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx477_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

MODULE_DEVICE_TABLE(of, imx477_dt_ids);

static const struct dev_pm_ops imx477_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx477_suspend, imx477_resume)
	SET_RUNTIME_PM_OPS(imx477_power_off, imx477_power_on, NULL)
};

static struct i2c_driver imx477_i2c_driver = {
	.driver = {
		.name = "imx477",
		.of_match_table	= imx477_dt_ids,
		.pm = &imx477_pm_ops,
	},
	.probe = imx477_probe,
	.remove = imx477_remove,
};

module_i2c_driver(imx477_i2c_driver);

MODULE_AUTHOR("Naushir Patuck <naush@raspberrypi.com>");
MODULE_DESCRIPTION("Sony IMX477 sensor driver");
MODULE_LICENSE("GPL v2");
