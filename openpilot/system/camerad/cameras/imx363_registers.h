#pragma once

#include "media/cam_sensor.h"

// IMX363 start streaming register
const struct i2c_random_wr_payload start_reg_array_imx363[] = {
  {0x0100, 0x01},  // MODE_SELECT = streaming
};

// IMX363 init register array
// Mode: 1920x1080 @ ~20fps, 10-bit RAW RGGB, 4 MIPI lanes
// MCLK: 24 MHz
// PLL VCO: 1920 MHz, VT_PIX_CLK: 96 MHz
// MIPI data rate: 960 Mbps total (240 Mbps/lane)
const struct i2c_random_wr_payload init_array_imx363[] = {
  // Software standby
  {0x0100, 0x00},

  // External clock frequency (24.0 MHz)
  {0x0136, 0x18},  // EXCK_FREQ [15:8] = 24
  {0x0137, 0x00},  // EXCK_FREQ [7:0] = .0

  // Data format - 10-bit RAW, 4 MIPI lanes
  {0x0112, 0x0A},  // CSI_DATA_FORMAT [15:8]
  {0x0113, 0x0A},  // CSI_DATA_FORMAT [7:0]
  {0x0114, 0x03},  // CSI_LANE_MODE = 4 lanes

  // PLL settings for 24 MHz MCLK
  // VCO = MCLK / PRE_PLL_DIV * PLL_MULT = 24/4 * 320 = 1920 MHz
  // VT_PIX_CLK = VCO / (VT_SYS_DIV * VT_PIX_DIV) = 1920/(4*5) = 96 MHz
  {0x0301, 0x05},  // VT_PIX_CLK_DIV = 5
  {0x0303, 0x04},  // VT_SYS_CLK_DIV = 4
  {0x0305, 0x04},  // PRE_PLL_CLK_DIV = 4
  {0x0306, 0x01},  // PLL_MULTIPLIER [12:8]
  {0x0307, 0x40},  // PLL_MULTIPLIER [7:0] = 320

  // OP clock: VCO / OP_SYS_DIV = 1920/2 = 960 Mbps total
  // Per lane: 960/4 = 240 MHz DDR = 480 Mbps/lane
  {0x0309, 0x0A},  // OP_PIX_CLK_DIV = 10
  {0x030B, 0x02},  // OP_SYS_CLK_DIV = 2

  // 2nd PLL: 24/2 * 200 = 2400 MHz
  {0x030D, 0x02},  // PRE_PLL_CLK_DIV_2nd = 2
  {0x030E, 0x00},  // PLL_MULT_2nd [10:8]
  {0x030F, 0xC8},  // PLL_MULT_2nd [7:0] = 200
  {0x0310, 0x00},

  // CSI data rate
  {0x0820, 0x03},  // 960 Mbps total = 0x03C0
  {0x0821, 0xC0},
  {0x0822, 0x00},
  {0x0823, 0x00},

  // Frame timing
  {0x0342, 0x0F},  // LINE_LENGTH_PCK [15:8] = 3968
  {0x0343, 0x80},  // LINE_LENGTH_PCK [7:0]
  {0x0340, 0x04},  // FRAME_LENGTH_LINES [15:8] = 1200
  {0x0341, 0xB0},  // FRAME_LENGTH_LINES [7:0]
  // FPS = 96 MHz / (3968 * 1200) = 20.16 fps

  // Analog crop window: center 1920x1080 from 4032x3024 sensor
  {0x0344, 0x04},  // X_ADD_STA [12:8] = 1056 (0x0420)
  {0x0345, 0x20},  // X_ADD_STA [7:0]
  {0x0346, 0x03},  // Y_ADD_STA [12:8] = 972 (0x03CC)
  {0x0347, 0xCC},  // Y_ADD_STA [7:0]
  {0x0348, 0x0B},  // X_ADD_END [12:8] = 2975 (0x0B9F)
  {0x0349, 0x9F},  // X_ADD_END [7:0]
  {0x034A, 0x08},  // Y_ADD_END [12:8] = 2051 (0x0803)
  {0x034B, 0x03},  // Y_ADD_END [7:0]

  // Output size
  {0x034C, 0x07},  // X_OUTPUT_SIZE [12:8] = 1920 (0x0780)
  {0x034D, 0x80},  // X_OUTPUT_SIZE [7:0]
  {0x034E, 0x04},  // Y_OUTPUT_SIZE [12:8] = 1080 (0x0438)
  {0x034F, 0x38},  // Y_OUTPUT_SIZE [7:0]

  // Subsampling / binning
  {0x0220, 0x00},
  {0x0221, 0x11},
  {0x0381, 0x01},  // X_EVN_INC
  {0x0383, 0x01},  // X_ODD_INC
  {0x0385, 0x01},  // Y_EVN_INC
  {0x0387, 0x01},  // Y_ODD_INC
  {0x0900, 0x00},  // BINNING_MODE = OFF
  {0x0901, 0x11},  // BINNING_TYPE H=1, V=1

  // Digital crop (passthrough - analog window already 1920x1080)
  {0x0401, 0x00},  // SCALING_MODE = no scaling
  {0x0404, 0x00},
  {0x0405, 0x10},  // SCALE_M = 16 (1:1)
  {0x0408, 0x00},  // DIG_CROP_X_OFFSET = 0
  {0x0409, 0x00},
  {0x040A, 0x00},  // DIG_CROP_Y_OFFSET = 0
  {0x040B, 0x00},
  {0x040C, 0x07},  // DIG_CROP_IMAGE_WIDTH = 1920
  {0x040D, 0x80},
  {0x040E, 0x04},  // DIG_CROP_IMAGE_HEIGHT = 1080
  {0x040F, 0x38},

  // Exposure defaults
  {0x0202, 0x04},  // COARSE_INTEG_TIME [15:8] = 1100
  {0x0203, 0x4C},  // COARSE_INTEG_TIME [7:0]
  {0x0204, 0x00},  // ANA_GAIN_GLOBAL [15:8] = 0 (1x)
  {0x0205, 0x00},  // ANA_GAIN_GLOBAL [7:0]

  // Digital gain (all channels 1x = 0x0100)
  {0x020E, 0x01},  // DIG_GAIN_GR [15:8]
  {0x020F, 0x00},  // DIG_GAIN_GR [7:0]
  {0x0210, 0x01},  // DIG_GAIN_R [15:8]
  {0x0211, 0x00},  // DIG_GAIN_R [7:0]
  {0x0212, 0x01},  // DIG_GAIN_B [15:8]
  {0x0213, 0x00},  // DIG_GAIN_B [7:0]
  {0x0214, 0x01},  // DIG_GAIN_GB [15:8]
  {0x0215, 0x00},  // DIG_GAIN_GB [7:0]

  // Image orientation (no flip)
  {0x0101, 0x00},

  // Test pattern off
  {0x0600, 0x00},
  {0x0601, 0x00},
};
