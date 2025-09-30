/*
 * Phoenix-RTOS
 *
 * IMX219PQ AXI I2C (axi_iic) BUS driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Krzysztof Szostek
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef IMX219PQ
#define IMX219PQ

#include <stdint.h>
#include <stddef.h>

#define MODEL_ID 0x0000  // 2 bytes

#define MODE_SELECT            0x0100
#define SOFTWARE_RESET         0x0103
#define CORRUPTED_FRAME_STATUS 0x0104

#define CSI_LANE_MODE 0x0114
#define DPHY_CTRL     0x0128

// Output Set-up Registers

// Frame Bank Registers Group "A"
#define FRAME_DURATION_A          0x0154  // [7:0] default: 0x00
#define COMP_ENABLE_A             0x0155  // [0] default: 0 - compression 10 to 8 mode: 0/1 - Disable/Enable
#define ANA_GAIN_GLOBAL_A         0x0157  // [7:0] default: 0x00
#define DIG_GAIN_GLOBAL_A         0x0158  // [3:0][7:0] default: 0x0100
#define COARSE_INTEGRATION_TIME_A 0x015A  // [7:0][7:0] default: 0x03E8
#define SENSOR_MODE_A             0x015D  // [0] read only shutter mode register. 0: ERS, 1: reserved
#define FRM_LENGTH_A              0x0160  // [7:0][7:0] default: 0x0AA8
#define LINE_LENGTH_A             0x0162  // [7:0][7:0] default: 0x0D78
#define X_ADD_STA_A               0x0164  // [3:0][7:0] default: 0x0000
#define X_ADD_END_A               0x0166  // [3:0][7:0] default: 0x0CCF
#define Y_ADD_STA_A               0x0168  // [3:0][7:0] default: 0x0000
#define Y_ADD_END_A               0x016A  // [3:0][7:0] default: 0x099F
#define X_OUTPUT_SIZE_A           0x016C  // [3:0][7:0] default: 0x0CD0
#define Y_OUTPUT_SIZE_A           0x016E  // [3:0][7:0] default: 0x09A0
#define X_ODD_INC_A               0x0170  // [2:0] default: 1
#define Y_ODD_INC_A               0x0171  // [2:0] default: 1
#define IMG_ORIENTATION_A         0x0172  // [1:0] default: 0
#define BINNING_MODE_H_A          0x0174  // [1:0] default: 0, 0/1/2/3 - no/x2/x4/x2-analog binning
#define BINNING_MODE_V_A          0x0175  // [1:0] default: 0, 0/1/2/3 - no/x2/x4/x2-analog binning
#define BINNING_CAL_MODE_H_A      0x0176  // [0] default: 0, 0/1 - average/sum binning mode
#define BINNING_CAL_MODE_V_A      0x0177  // [0] default: 0, 0/1 - average/sum binning mode
#define ANA_GAIN_GLOBAL_SHORT_A   0x0189  // [7:0] default: 0
#define COARSE_INTEG_TIME_SHORT_A 0x018A  // [7:0][7:0] default: 0x01F4
#define CSI_DATA_FORMAT_A         0x018C  // [7:0][7:0] default: 0x0A0A

// Frame Bank Registers Group "B"
#define FRAME_DURATION_B          0x0254  // [7:0] default: 0x00
#define COMP_ENABLE_B             0x0255  // [0] default: 0 - compression 10 to 8 mode: 0/1 - Disable/Enable
#define ANA_GAIN_GLOBAL_B         0x0257  // [7:0] default: 0x00
#define DIG_GAIN_GLOBAL_B         0x0258  // [3:0][7:0] default: 0x0100
#define COARSE_INTEGRATION_TIME_B 0x025A  // [7:0][7:0] default: 0x03E8
#define SENSOR_MODE_B             0x025D  // [0] read only shutter mode register. 0: ERS, 1: reserved
#define FRM_LENGTH_B              0x0260  // [7:0][7:0] default: 0x0AA8
#define LINE_LENGTH_B             0x0262  // [7:0][7:0] default: 0x0D78
#define X_ADD_STA_B               0x0264  // [3:0][7:0] default: 0x0000
#define X_ADD_END_B               0x0266  // [3:0][7:0] default: 0x0CCF
#define Y_ADD_STA_B               0x0268  // [3:0][7:0] default: 0x0000
#define Y_ADD_END_B               0x026A  // [3:0][7:0] default: 0x099F
#define X_OUTPUT_SIZE_B           0x026C  // [3:0][7:0] default: 0x0CD0
#define Y_OUTPUT_SIZE_B           0x026E  // [3:0][7:0] default: 0x09A0
#define X_ODD_INC_B               0x0270  // [2:0] default: 1
#define Y_ODD_INC_B               0x0271  // [2:0] default: 1
#define IMG_ORIENTATION_B         0x0272  // [1:0] default: 0
#define BINNING_MODE_H_B          0x0274  // [1:0] default: 0, 0/1/2/3 - no/x2/x4/x2-analog binning
#define BINNING_MODE_V_B          0x0275  // [1:0] default: 0, 0/1/2/3 - no/x2/x4/x2-analog binning
#define BINNING_CAL_MODE_H_B      0x0276  // [0] default: 0, 0/1 - average/sum binning mode
#define BINNING_CAL_MODE_V_B      0x0277  // [0] default: 0, 0/1 - average/sum binning mode
#define ANA_GAIN_GLOBAL_SHORT_B   0x0289  // [7:0] default: 0
#define COARSE_INTEG_TIME_SHORT_B 0x028A  // [7:0][7:0] default: 0x01F4
#define CSI_DATA_FORMAT_B         0x028C  // [7:0][7:0] default: 0x0A0A


// Clock Set-up Registers
#define VTPXCK_DIV      0x0301  // [4:0] default: 5
#define VTSYCK_DIV      0x0303  // [1:0] default: 1
#define PREPLLCK_VT_DIV 0x0304  // [7:0] default: 2
#define PREPLLCK_OP_DIV 0x0305  // [7:0] default: 2
#define PLL_VT_MPY      0x0306  // [2:0][7:0] default: 0x75
#define OPPXCK_DIV      0x0309  // [4:0] default: 0x0A
#define OPSYCK_DIV      0x030B  // [1:0] default: 1
#define PLL_OP_MPY      0x030C  // [2:0][7:0] default: 0x75

#define MDSEL_SW_STANDBY 0
#define MDSEL_STREAMING  1

// int i2c


void setStreaming(uint8_t dev_addr);
#endif  // !IMX219PQ
