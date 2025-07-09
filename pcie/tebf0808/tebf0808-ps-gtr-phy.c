/******************************************************************************
 * Copyright (C) 2010-2020 Xilinx, Inc.  All rights reserved.
 * SPDX-License-Identifier: MIT
 ******************************************************************************/

/*
 * Phoenix-RTOS
 *
 * PCI Express PS PHY initialisation code for TEBF0808 baseboard + TE0807 SOM
 * Code is derived from Trenz Electronic example design.
 *
 * Copyright 2025 Phoenix Systems
 * Author: Dariusz Sabala
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/platform.h>

#include <phoenix/arch/aarch64/zynqmp/zynqmp.h>

#include <board_config.h>

#include <pcie.h>

#include <tebf0808-ps-gtr-phy.h>


#define SERDES_SIZE    0x20000
#define SERDES_ADDRESS 0xfd400000


#define PCIREG_SIZE    0x1000
#define PCIREG_ADDRESS 0xfd480000


#define SERDES_L0_TX_ANA_TM_3                   0x000C
#define SERDES_L0_TX_DIG_TM_61                  0x00F4
#define SERDES_L0_TM_ANA_BYP_4                  0x1010
#define SERDES_L0_TM_ANA_BYP_7                  0x1018
#define SERDES_L0_TM_ANA_BYP_12_OFFSET          0x102C
#define SERDES_L0_TM_ANA_BYP_15_OFFSET          0x1038
#define SERDES_L0_TM_DIG_6                      0x106C
#define SERDES_L0_TM_DIG_8_OFFSET               0x1074
#define SERDES_L0_TM_DIG_10_OFFSET              0x107C
#define SERDES_L0_TM_DIG_22                     0x10AC
#define SERDES_L0_TM_AUX_0_OFFSET               0x10CC
#define SERDES_L0_TM_MISC2_OFFSET               0x189C
#define SERDES_L0_TM_IQ_ILL1_OFFSET             0x18F8
#define SERDES_L0_TM_IQ_ILL1                    0x18F8
#define SERDES_L0_TM_IQ_ILL2_OFFSET             0x18FC
#define SERDES_L0_TM_IQ_ILL2                    0x18FC
#define SERDES_L0_TM_IQ_ILL3_OFFSET             0x1900
#define SERDES_L0_TM_IQ_ILL3                    0x1900
#define SERDES_L0_TM_IQ_ILL7                    0x1910
#define SERDES_L0_TM_IQ_ILL8_OFFSET             0x1914
#define SERDES_L0_TM_IQ_ILL8                    0x1914
#define SERDES_L0_TM_IQ_ILL9_OFFSET             0x1918
#define SERDES_L0_TM_IQ_ILL9                    0x1918
#define SERDES_L0_TM_E_ILL1_OFFSET              0x1924
#define SERDES_L0_TM_E_ILL1                     0x1924
#define SERDES_L0_TM_E_ILL2_OFFSET              0x1928
#define SERDES_L0_TM_E_ILL2                     0x1928
#define SERDES_L0_TM_E_ILL3_OFFSET              0x192C
#define SERDES_L0_TM_E_ILL3                     0x192C
#define SERDES_L0_TM_E_ILL7                     0x193C
#define SERDES_L0_TM_E_ILL8_OFFSET              0x1940
#define SERDES_L0_TM_E_ILL8                     0x1940
#define SERDES_L0_TM_E_ILL9_OFFSET              0x1944
#define SERDES_L0_TM_E_ILL9                     0x1944
#define SERDES_L0_TM_EQ11_OFFSET                0x1978
#define SERDES_L0_TM_ILL8_OFFSET                0x1980
#define SERDES_L0_TM_ILL8                       0x1980
#define SERDES_L0_TM_ILL11                      0x198C
#define SERDES_L0_TM_ILL12_OFFSET               0x1990
#define SERDES_L0_TM_ILL12                      0x1990
#define SERDES_L0_TM_ILL13_OFFSET               0x1994
#define SERDES_L0_TM_ILL13                      0x1994
#define SERDES_L0_TM_RST_DLY_OFFSET             0x19A4
#define SERDES_L0_TM_MISC3_OFFSET               0x19AC
#define SERDES_L0_TM_MISC_ST_0                  0x1AC8
#define SERDES_L0_TM_PLL_DIG_33                 0x2084
#define SERDES_L0_PLL_FBDIV_FRAC_3_MSB          0x2360
#define SERDES_L0_PLL_STATUS_READ_1_OFFSET      0x23E4
#define SERDES_L0_PLL_STATUS_READ_1             0x23E4
#define SERDES_L0_L0_REF_CLK_SEL_OFFSET         0x2860
#define SERDES_L0_L1_REF_CLK_SEL_OFFSET         0x2864
#define SERDES_L0_L2_REF_CLK_SEL_OFFSET         0x2868
#define SERDES_L0_L3_REF_CLK_SEL_OFFSET         0x286C
#define SERDES_L0_BIST_CTRL_1                   0x3004
#define SERDES_L0_BIST_CTRL_2                   0x3008
#define SERDES_L0_BIST_RUN_LEN_L                0x300C
#define SERDES_L0_BIST_ERR_INJ_POINT_L          0x3010
#define SERDES_L0_BIST_RUNLEN_ERR_INJ_H         0x3014
#define SERDES_L0_BIST_IDLE_TIME                0x3018
#define SERDES_L0_BIST_MARKER_L                 0x301C
#define SERDES_L0_BIST_IDLE_CHAR_L              0x3020
#define SERDES_L0_BIST_MARKER_IDLE_H            0x3024
#define SERDES_L0_BIST_LOW_PULSE_TIME           0x3028
#define SERDES_L0_BIST_TOTAL_PULSE_TIME         0x302C
#define SERDES_L0_BIST_TEST_PAT_1               0x3030
#define SERDES_L0_BIST_TEST_PAT_2               0x3034
#define SERDES_L0_BIST_TEST_PAT_3               0x3038
#define SERDES_L0_BIST_TEST_PAT_4               0x303C
#define SERDES_L0_BIST_TEST_PAT_MSBS            0x3040
#define SERDES_L0_BIST_PKT_NUM                  0x3044
#define SERDES_L0_BIST_FRM_IDLE_TIME            0x3048
#define SERDES_L0_BIST_PKT_CTR_L                0x304C
#define SERDES_L0_BIST_PKT_CTR_H                0x3050
#define SERDES_L0_BIST_ERR_CTR_L                0x3054
#define SERDES_L0_BIST_ERR_CTR_H                0x3058
#define SERDES_L0_DATA_BUS_WID                  0x3060
#define SERDES_L0_BIST_FILLER_OUT               0x3068
#define SERDES_L0_BIST_FORCE_MK_RST             0x306C
#define SERDES_L1_TX_ANA_TM_3                   0x400C
#define SERDES_L1_TX_DIG_TM_61_OFFSET           0x40F4
#define SERDES_L1_TX_DIG_TM_61                  0x40F4
#define SERDES_L1_TM_ANA_BYP_4                  0x5010
#define SERDES_L1_TM_ANA_BYP_7                  0x5018
#define SERDES_L1_TM_ANA_BYP_12_OFFSET          0x502C
#define SERDES_L1_TM_ANA_BYP_15_OFFSET          0x5038
#define SERDES_L1_TM_DIG_6_OFFSET               0x506C
#define SERDES_L1_TM_DIG_6                      0x506C
#define SERDES_L1_TM_DIG_8_OFFSET               0x5074
#define SERDES_L1_TM_DIG_10_OFFSET              0x507C
#define SERDES_L1_TM_DIG_22                     0x50AC
#define SERDES_L1_TM_AUX_0_OFFSET               0x50CC
#define SERDES_L1_TM_MISC2_OFFSET               0x589C
#define SERDES_L1_TM_MISC2                      0x589C
#define SERDES_L1_TM_IQ_ILL1_OFFSET             0x58F8
#define SERDES_L1_TM_IQ_ILL1                    0x58F8
#define SERDES_L1_TM_IQ_ILL2_OFFSET             0x58FC
#define SERDES_L1_TM_IQ_ILL2                    0x58FC
#define SERDES_L1_TM_IQ_ILL3_OFFSET             0x5900
#define SERDES_L1_TM_IQ_ILL3                    0x5900
#define SERDES_L1_TM_IQ_ILL7                    0x5910
#define SERDES_L1_TM_IQ_ILL8_OFFSET             0x5914
#define SERDES_L1_TM_IQ_ILL8                    0x5914
#define SERDES_L1_TM_IQ_ILL9_OFFSET             0x5918
#define SERDES_L1_TM_IQ_ILL9                    0x5918
#define SERDES_L1_TM_E_ILL1_OFFSET              0x5924
#define SERDES_L1_TM_E_ILL1                     0x5924
#define SERDES_L1_TM_E_ILL2_OFFSET              0x5928
#define SERDES_L1_TM_E_ILL2                     0x5928
#define SERDES_L1_TM_E_ILL3_OFFSET              0x592C
#define SERDES_L1_TM_E_ILL3                     0x592C
#define SERDES_L1_TM_E_ILL7                     0x593C
#define SERDES_L1_TM_E_ILL8_OFFSET              0x5940
#define SERDES_L1_TM_E_ILL8                     0x5940
#define SERDES_L1_TM_E_ILL9_OFFSET              0x5944
#define SERDES_L1_TM_E_ILL9                     0x5944
#define SERDES_L1_TM_EQ11_OFFSET                0x5978
#define SERDES_L1_TM_ILL8_OFFSET                0x5980
#define SERDES_L1_TM_ILL8                       0x5980
#define SERDES_L1_TM_ILL11                      0x598C
#define SERDES_L1_TM_ILL12_OFFSET               0x5990
#define SERDES_L1_TM_ILL12                      0x5990
#define SERDES_L1_TM_ILL13_OFFSET               0x5994
#define SERDES_L1_TM_ILL13                      0x5994
#define SERDES_L1_TM_RST_DLY_OFFSET             0x59A4
#define SERDES_L1_TM_MISC3_OFFSET               0x59AC
#define SERDES_L1_TM_MISC_ST_0                  0x5AC8
#define SERDES_L1_TM_PLL_DIG_33                 0x6084
#define SERDES_L1_TM_PLL_DIG_37_OFFSET          0x6094
#define SERDES_L1_PLL_FBDIV_FRAC_3_MSB          0x6360
#define SERDES_L1_PLL_SS_STEPS_0_LSB_OFFSET     0x6368
#define SERDES_L1_PLL_SS_STEPS_1_MSB_OFFSET     0x636C
#define SERDES_L1_PLL_SS_STEP_SIZE_0_LSB_OFFSET 0x6370
#define SERDES_L1_PLL_SS_STEP_SIZE_1_OFFSET     0x6374
#define SERDES_L1_PLL_SS_STEP_SIZE_2_OFFSET     0x6378
#define SERDES_L1_PLL_SS_STEP_SIZE_3_MSB_OFFSET 0x637C
#define SERDES_L1_PLL_STATUS_READ_1_OFFSET      0x63E4
#define SERDES_L1_PLL_STATUS_READ_1             0x63E4
#define SERDES_L1_BIST_CTRL_1                   0x7004
#define SERDES_L1_BIST_CTRL_2                   0x7008
#define SERDES_L1_BIST_RUN_LEN_L                0x700C
#define SERDES_L1_BIST_ERR_INJ_POINT_L          0x7010
#define SERDES_L1_BIST_RUNLEN_ERR_INJ_H         0x7014
#define SERDES_L1_BIST_IDLE_TIME                0x7018
#define SERDES_L1_BIST_MARKER_L                 0x701C
#define SERDES_L1_BIST_IDLE_CHAR_L              0x7020
#define SERDES_L1_BIST_MARKER_IDLE_H            0x7024
#define SERDES_L1_BIST_LOW_PULSE_TIME           0x7028
#define SERDES_L1_BIST_TOTAL_PULSE_TIME         0x702C
#define SERDES_L1_BIST_TEST_PAT_1               0x7030
#define SERDES_L1_BIST_TEST_PAT_2               0x7034
#define SERDES_L1_BIST_TEST_PAT_3               0x7038
#define SERDES_L1_BIST_TEST_PAT_4               0x703C
#define SERDES_L1_BIST_TEST_PAT_MSBS            0x7040
#define SERDES_L1_BIST_PKT_NUM                  0x7044
#define SERDES_L1_BIST_FRM_IDLE_TIME            0x7048
#define SERDES_L1_BIST_PKT_CTR_L                0x704C
#define SERDES_L1_BIST_PKT_CTR_H                0x7050
#define SERDES_L1_BIST_ERR_CTR_L                0x7054
#define SERDES_L1_BIST_ERR_CTR_H                0x7058
#define SERDES_L1_DATA_BUS_WID                  0x7060
#define SERDES_L1_BIST_FILLER_OUT               0x7068
#define SERDES_L1_BIST_FORCE_MK_RST             0x706C
#define SERDES_L2_TX_ANA_TM_3                   0x800C
#define SERDES_L2_TX_ANA_TM_18_OFFSET           0x8048
#define SERDES_L2_TX_DIG_TM_61_OFFSET           0x80F4
#define SERDES_L2_TX_DIG_TM_61                  0x80F4
#define SERDES_L2_TX_ANA_TM_118_OFFSET          0x81D8
#define SERDES_L2_TM_ANA_BYP_4                  0x9010
#define SERDES_L2_TM_ANA_BYP_7                  0x9018
#define SERDES_L2_TM_ANA_BYP_12_OFFSET          0x902C
#define SERDES_L2_TM_ANA_BYP_15_OFFSET          0x9038
#define SERDES_L2_TM_DIG_6_OFFSET               0x906C
#define SERDES_L2_TM_DIG_6                      0x906C
#define SERDES_L2_TM_DIG_8_OFFSET               0x9074
#define SERDES_L2_TM_DIG_10_OFFSET              0x907C
#define SERDES_L2_TM_DIG_22                     0x90AC
#define SERDES_L2_TM_MISC2_OFFSET               0x989C
#define SERDES_L2_TM_MISC2                      0x989C
#define SERDES_L2_TM_IQ_ILL1_OFFSET             0x98F8
#define SERDES_L2_TM_IQ_ILL1                    0x98F8
#define SERDES_L2_TM_IQ_ILL2_OFFSET             0x98FC
#define SERDES_L2_TM_IQ_ILL2                    0x98FC
#define SERDES_L2_TM_IQ_ILL3_OFFSET             0x9900
#define SERDES_L2_TM_IQ_ILL3                    0x9900
#define SERDES_L2_TM_IQ_ILL7                    0x9910
#define SERDES_L2_TM_IQ_ILL8_OFFSET             0x9914
#define SERDES_L2_TM_IQ_ILL8                    0x9914
#define SERDES_L2_TM_IQ_ILL9_OFFSET             0x9918
#define SERDES_L2_TM_IQ_ILL9                    0x9918
#define SERDES_L2_TM_E_ILL1_OFFSET              0x9924
#define SERDES_L2_TM_E_ILL1                     0x9924
#define SERDES_L2_TM_E_ILL2_OFFSET              0x9928
#define SERDES_L2_TM_E_ILL2                     0x9928
#define SERDES_L2_TM_E_ILL3_OFFSET              0x992C
#define SERDES_L2_TM_E_ILL3                     0x992C
#define SERDES_L2_TM_E_ILL7                     0x993C
#define SERDES_L2_TM_E_ILL8_OFFSET              0x9940
#define SERDES_L2_TM_E_ILL8                     0x9940
#define SERDES_L2_TM_E_ILL9_OFFSET              0x9944
#define SERDES_L2_TM_E_ILL9                     0x9944
#define SERDES_L2_TM_EQ0_OFFSET                 0x994C
#define SERDES_L2_TM_EQ1_OFFSET                 0x9950
#define SERDES_L2_TM_EQ11_OFFSET                0x9978
#define SERDES_L2_TM_ILL8_OFFSET                0x9980
#define SERDES_L2_TM_ILL8                       0x9980
#define SERDES_L2_TM_ILL11_OFFSET               0x998C
#define SERDES_L2_TM_ILL11                      0x998C
#define SERDES_L2_TM_ILL12_OFFSET               0x9990
#define SERDES_L2_TM_ILL12                      0x9990
#define SERDES_L2_TM_ILL13_OFFSET               0x9994
#define SERDES_L2_TM_ILL13                      0x9994
#define SERDES_L2_TM_RST_DLY_OFFSET             0x99A4
#define SERDES_L2_TM_MISC3_OFFSET               0x99AC
#define SERDES_L2_TM_MISC_ST_0                  0x9AC8
#define SERDES_L2_TM_CDR5_OFFSET                0x9C14
#define SERDES_L2_TM_CDR16_OFFSET               0x9C40
#define SERDES_L2_TM_PLL_DIG_33                 0xA084
#define SERDES_L2_PLL_FBDIV_FRAC_3_MSB_OFFSET   0xA360
#define SERDES_L2_PLL_FBDIV_FRAC_3_MSB          0xA360
#define SERDES_L2_PLL_SS_STEPS_0_LSB_OFFSET     0xA368
#define SERDES_L2_PLL_SS_STEPS_1_MSB_OFFSET     0xA36C
#define SERDES_L2_PLL_SS_STEP_SIZE_0_LSB_OFFSET 0xA370
#define SERDES_L2_PLL_SS_STEP_SIZE_1_OFFSET     0xA374
#define SERDES_L2_PLL_SS_STEP_SIZE_2_OFFSET     0xA378
#define SERDES_L2_PLL_SS_STEP_SIZE_3_MSB_OFFSET 0xA37C
#define SERDES_L2_PLL_STATUS_READ_1_OFFSET      0xA3E4
#define SERDES_L2_PLL_STATUS_READ_1             0xA3E4
#define SERDES_L2_BIST_CTRL_1                   0xB004
#define SERDES_L2_BIST_CTRL_2                   0xB008
#define SERDES_L2_BIST_RUN_LEN_L                0xB00C
#define SERDES_L2_BIST_ERR_INJ_POINT_L          0xB010
#define SERDES_L2_BIST_RUNLEN_ERR_INJ_H         0xB014
#define SERDES_L2_BIST_IDLE_TIME                0xB018
#define SERDES_L2_BIST_MARKER_L                 0xB01C
#define SERDES_L2_BIST_IDLE_CHAR_L              0xB020
#define SERDES_L2_BIST_MARKER_IDLE_H            0xB024
#define SERDES_L2_BIST_LOW_PULSE_TIME           0xB028
#define SERDES_L2_BIST_TOTAL_PULSE_TIME         0xB02C
#define SERDES_L2_BIST_TEST_PAT_1               0xB030
#define SERDES_L2_BIST_TEST_PAT_2               0xB034
#define SERDES_L2_BIST_TEST_PAT_3               0xB038
#define SERDES_L2_BIST_TEST_PAT_4               0xB03C
#define SERDES_L2_BIST_TEST_PAT_MSBS            0xB040
#define SERDES_L2_BIST_PKT_NUM                  0xB044
#define SERDES_L2_BIST_FRM_IDLE_TIME            0xB048
#define SERDES_L2_BIST_PKT_CTR_L                0xB04C
#define SERDES_L2_BIST_PKT_CTR_H                0xB050
#define SERDES_L2_BIST_ERR_CTR_L                0xB054
#define SERDES_L2_BIST_ERR_CTR_H                0xB058
#define SERDES_L2_DATA_BUS_WID                  0xB060
#define SERDES_L2_BIST_FILLER_OUT               0xB068
#define SERDES_L2_BIST_FORCE_MK_RST             0xB06C
#define SERDES_L3_TX_ANA_TM_3                   0xC00C
#define SERDES_L3_TX_ANA_TM_18_OFFSET           0xC048
#define SERDES_L3_TX_DIG_TM_61                  0xC0F4
#define SERDES_L3_TX_ANA_TM_118_OFFSET          0xC1D8
#define SERDES_L3_TXPMD_TM_45_OFFSET            0xCCB4
#define SERDES_L3_TXPMD_TM_48_OFFSET            0xCCC0
#define SERDES_L3_TM_ANA_BYP_4                  0xD010
#define SERDES_L3_TM_ANA_BYP_7                  0xD018
#define SERDES_L3_TM_ANA_BYP_12_OFFSET          0xD02C
#define SERDES_L3_TM_ANA_BYP_15_OFFSET          0xD038
#define SERDES_L3_TM_DIG_6                      0xD06C
#define SERDES_L3_TM_DIG_8_OFFSET               0xD074
#define SERDES_L3_TM_DIG_10_OFFSET              0xD07C
#define SERDES_L3_TM_DIG_22                     0xD0AC
#define SERDES_L3_TM_MISC2                      0xD89C
#define SERDES_L3_TM_IQ_ILL1                    0xD8F8
#define SERDES_L3_TM_IQ_ILL2                    0xD8FC
#define SERDES_L3_TM_IQ_ILL3                    0xD900
#define SERDES_L3_TM_IQ_ILL7                    0xD910
#define SERDES_L3_TM_IQ_ILL8                    0xD914
#define SERDES_L3_TM_IQ_ILL9                    0xD918
#define SERDES_L3_TM_E_ILL1                     0xD924
#define SERDES_L3_TM_E_ILL2                     0xD928
#define SERDES_L3_TM_E_ILL3                     0xD92C
#define SERDES_L3_TM_E_ILL7                     0xD93C
#define SERDES_L3_TM_E_ILL8                     0xD940
#define SERDES_L3_TM_E_ILL9                     0xD944
#define SERDES_L3_TM_EQ11_OFFSET                0xD978
#define SERDES_L3_TM_ILL8                       0xD980
#define SERDES_L3_TM_ILL11                      0xD98C
#define SERDES_L3_TM_ILL12                      0xD990
#define SERDES_L3_TM_ILL13_OFFSET               0xD994
#define SERDES_L3_TM_ILL13                      0xD994
#define SERDES_L3_TM_RST_DLY_OFFSET             0xD9A4
#define SERDES_L3_TM_MISC3_OFFSET               0xD9AC
#define SERDES_L3_TM_MISC_ST_0                  0xDAC8
#define SERDES_L3_TM_PLL_DIG_33                 0xE084
#define SERDES_L3_PLL_FBDIV_FRAC_3_MSB          0xE360
#define SERDES_L3_PLL_SS_STEPS_0_LSB_OFFSET     0xE368
#define SERDES_L3_PLL_SS_STEPS_1_MSB_OFFSET     0xE36C
#define SERDES_L3_PLL_SS_STEP_SIZE_0_LSB_OFFSET 0xE370
#define SERDES_L3_PLL_SS_STEP_SIZE_1_OFFSET     0xE374
#define SERDES_L3_PLL_SS_STEP_SIZE_2_OFFSET     0xE378
#define SERDES_L3_PLL_SS_STEP_SIZE_3_MSB_OFFSET 0xE37C
#define SERDES_L3_PLL_STATUS_READ_1_OFFSET      0xE3E4
#define SERDES_L3_PLL_STATUS_READ_1             0xE3E4
#define SERDES_L3_BIST_CTRL_1                   0xF004
#define SERDES_L3_BIST_CTRL_2                   0xF008
#define SERDES_L3_BIST_RUN_LEN_L                0xF00C
#define SERDES_L3_BIST_ERR_INJ_POINT_L          0xF010
#define SERDES_L3_BIST_RUNLEN_ERR_INJ_H         0xF014
#define SERDES_L3_BIST_IDLE_TIME                0xF018
#define SERDES_L3_BIST_MARKER_L                 0xF01C
#define SERDES_L3_BIST_IDLE_CHAR_L              0xF020
#define SERDES_L3_BIST_MARKER_IDLE_H            0xF024
#define SERDES_L3_BIST_LOW_PULSE_TIME           0xF028
#define SERDES_L3_BIST_TOTAL_PULSE_TIME         0xF02C
#define SERDES_L3_BIST_TEST_PAT_1               0xF030
#define SERDES_L3_BIST_TEST_PAT_2               0xF034
#define SERDES_L3_BIST_TEST_PAT_3               0xF038
#define SERDES_L3_BIST_TEST_PAT_4               0xF03C
#define SERDES_L3_BIST_TEST_PAT_MSBS            0xF040
#define SERDES_L3_BIST_PKT_NUM                  0xF044
#define SERDES_L3_BIST_FRM_IDLE_TIME            0xF048
#define SERDES_L3_BIST_PKT_CTR_L                0xF04C
#define SERDES_L3_BIST_PKT_CTR_H                0xF050
#define SERDES_L3_BIST_ERR_CTR_L                0xF054
#define SERDES_L3_BIST_ERR_CTR_H                0xF058
#define SERDES_L3_DATA_BUS_WID                  0xF060
#define SERDES_L3_BIST_FILLER_OUT               0xF068
#define SERDES_L3_BIST_FORCE_MK_RST             0xF06C
#define SERDES_PLL_REF_SEL0_OFFSET              0x10000
#define SERDES_PLL_REF_SEL1_OFFSET              0x10004
#define SERDES_PLL_REF_SEL2_OFFSET              0x10008
#define SERDES_PLL_REF_SEL3_OFFSET              0x1000C
#define SERDES_ICM_CFG0_OFFSET                  0x10010
#define SERDES_ICM_CFG1_OFFSET                  0x10014
#define SERDES_LPBK_CTRL0                       0x10038
#define SERDES_LPBK_CTRL1                       0x1003C
#define SERDES_TX_PROT_BUS_WIDTH                0x10040
#define SERDES_RX_PROT_BUS_WIDTH                0x10044
#define SERDES_UPHY_SPARE0                      0x10098
#define SERDES_UPHY_SPARE1                      0x1009C
#define SERDES_UPHY_SPARE2                      0x100A0
#define SERDES_UPHY_SPARE3                      0x100A4


#define PCIE_ATTRIB_ATTR_4_OFFSET    0X0010
#define PCIE_ATTRIB_ATTR_8_OFFSET    0X0020
#define PCIE_ATTRIB_ATTR_7_OFFSET    0X001C
#define PCIE_ATTRIB_ATTR_9_OFFSET    0X0024
#define PCIE_ATTRIB_ATTR_10_OFFSET   0X0028
#define PCIE_ATTRIB_ATTR_11_OFFSET   0X002C
#define PCIE_ATTRIB_ATTR_12_OFFSET   0X0030
#define PCIE_ATTRIB_ATTR_13_OFFSET   0X0034
#define PCIE_ATTRIB_ATTR_14_OFFSET   0X0038
#define PCIE_ATTRIB_ATTR_15_OFFSET   0X003C
#define PCIE_ATTRIB_ATTR_16_OFFSET   0X0040
#define PCIE_ATTRIB_ATTR_17_OFFSET   0X0044
#define PCIE_ATTRIB_ATTR_18_OFFSET   0X0048
#define PCIE_ATTRIB_ATTR_24_OFFSET   0X0060
#define PCIE_ATTRIB_ATTR_25_OFFSET   0X0064
#define PCIE_ATTRIB_ATTR_25_OFFSET   0X0064
#define PCIE_ATTRIB_ATTR_27_OFFSET   0X006C
#define PCIE_ATTRIB_ATTR_34_OFFSET   0X0088
#define PCIE_ATTRIB_ATTR_35_OFFSET   0X008C
#define PCIE_ATTRIB_ATTR_37_OFFSET   0X0094
#define PCIE_ATTRIB_ATTR_41_OFFSET   0X00A4
#define PCIE_ATTRIB_ATTR_43_OFFSET   0X00AC
#define PCIE_ATTRIB_ATTR_44_OFFSET   0X00B0
#define PCIE_ATTRIB_ATTR_45_OFFSET   0X00B4
#define PCIE_ATTRIB_ATTR_46_OFFSET   0X00B8
#define PCIE_ATTRIB_ATTR_47_OFFSET   0X00BC
#define PCIE_ATTRIB_ATTR_48_OFFSET   0X00C0
#define PCIE_ATTRIB_ATTR_50_OFFSET   0X00C8
#define PCIE_ATTRIB_ATTR_53_OFFSET   0X00D4
#define PCIE_ATTRIB_ATTR_79_OFFSET   0X013C
#define PCIE_ATTRIB_ATTR_89_OFFSET   0X0164
#define PCIE_ATTRIB_ATTR_93_OFFSET   0X0174
#define PCIE_ATTRIB_ATTR_97_OFFSET   0X0184
#define PCIE_ATTRIB_ATTR_100_OFFSET  0X0190
#define PCIE_ATTRIB_ATTR_101_OFFSET  0X0194
#define PCIE_ATTRIB_ATTR_105_OFFSET  0X01A4
#define PCIE_ATTRIB_ATTR_106_OFFSET  0X01A8
#define PCIE_ATTRIB_ATTR_107_OFFSET  0X01AC
#define PCIE_ATTRIB_ATTR_108_OFFSET  0X01B0
#define PCIE_ATTRIB_ATTR_109_OFFSET  0X01B4
#define PCIE_ATTRIB_CB_OFFSET        0X031C
#define PCIE_ATTRIB_ID_OFFSET        0X0200
#define PCIE_ATTRIB_REV_ID_OFFSET    0X0208
#define PCIE_ATTRIB_SUBSYS_ID_OFFSET 0X0204


static int pcie_phyInitPcieClock(void)
{
	platformctl_t ctl = {
		.action = pctl_set,
		.type = pctl_devclock,
		.devclock.dev = pctl_devclock_fpd_pcie,
		.devclock.src = 0,  /* source: IOPLL_TO_FPD (500 MHz) */
		.devclock.div0 = 2, /* divide by 2 = 250 MHz */
		.devclock.div1 = 0,
		.devclock.active = 0x1,
	};
	return platformctl(&ctl);
}


static int pcie_phyDeassertGtReset(void)
{
	int ret = 0;
	platformctl_t ctl1 = {
		.action = pctl_set,
		.type = pctl_devreset,
		.devreset.dev = pctl_devreset_fpd_gt,
		.devreset.state = 0,
	};
	ret = platformctl(&ctl1);
	if (ret != 0) {
		fprintf(stderr, "pcie-phy: fail to deassert GT reset\n");
		return ret;
	}

	return ret;
}


static int pcie_phyAssertResets(void)
{
	int ret = 0;

	platformctl_t ctl1 = {
		.action = pctl_set,
		.type = pctl_devreset,
		.devreset.dev = pctl_devreset_fpd_pcie_ctrl,
		.devreset.state = 1,
	};
	ret = platformctl(&ctl1);
	if (ret != 0) {
		fprintf(stderr, "pcie-phy: fail to assert PCIE CTRL clock\n");
		return ret;
	}

	platformctl_t ctl2 = {
		.action = pctl_set,
		.type = pctl_devreset,
		.devreset.dev = pctl_devreset_fpd_pcie_bridge,
		.devreset.state = 1,
	};
	ret = platformctl(&ctl2);
	if (ret != 0) {
		fprintf(stderr, "pcie-phy: fail to assert PCIE BRIDGE clock\n");
		return ret;
	}

	platformctl_t ctl3 = {
		.action = pctl_set,
		.type = pctl_devreset,
		.devreset.dev = pctl_devreset_fpd_pcie_cfg,
		.devreset.state = 1,
	};
	ret = platformctl(&ctl3);
	if (ret != 0) {
		fprintf(stderr, "pcie-phy: fail to assert PCIE CFG clock\n");
		return ret;
	}

	return ret;
}


static int mask_poll(uint32_t *base, uint32_t offset, uint32_t mask)
{
	int i = 0;

	while (1) {
		if (i == 1000) {
			return 0;
		}
		uint32_t reg_value = readReg(base, offset);
		if (reg_value & mask) {
			break;
		}
		i++;
		usleep(1 * 1000);
	}
	return 1;
}


static uint32_t mask_read(uint32_t *base, uint32_t offset, uint32_t mask)
{
	return (readReg(base, offset) & mask);
}


static int serdes_fixcal_code(uint32_t *base)
{
	int mask_status = 1;

	unsigned int rdata = 0;

	/*The valid codes are from 0x26 to 0x3C.
	 *There are 23 valid codes in total.
	 */
	/*Each element of array stands for count of occurrence of valid code.*/
	unsigned int match_pmos_code[23];
	/*Each element of array stands for count of occurrence of valid code.*/
	/*The valid codes are from 0xC to 0x12.
	 *There are 7 valid codes in total.
	 */
	unsigned int match_nmos_code[23];
	/*Each element of array stands for count of occurrence of valid code.*/
	/*The valid codes are from 0x6 to 0xC.
	 * There are 7 valid codes in total.
	 */
	unsigned int match_ical_code[7];
	/*Each element of array stands for count of occurrence of valid code.*/
	unsigned int match_rcal_code[7];

	unsigned int p_code = 0;
	unsigned int n_code = 0;
	unsigned int i_code = 0;
	unsigned int r_code = 0;
	unsigned int repeat_count = 0;
	unsigned int L3_TM_CALIB_DIG20 = 0;
	unsigned int L3_TM_CALIB_DIG19 = 0;
	unsigned int L3_TM_CALIB_DIG18 = 0;
	unsigned int L3_TM_CALIB_DIG16 = 0;
	unsigned int L3_TM_CALIB_DIG15 = 0;
	unsigned int L3_TM_CALIB_DIG14 = 0;

	int i = 0;

	rdata = readReg(base, 0x289C);
	rdata = rdata & ~0x03;
	rdata = rdata | 0x1;
	writeReg(base, 0x289C, rdata);
	// check supply good status before starting AFE sequencing
	int count = 0;
	do {
		if (count == 1100000) {
			break;
		}
		rdata = readReg(base, 0x2B1C);
		count++;
	} while ((rdata & 0x0000000E) != 0x0000000E);

	for (i = 0; i < 23; i++) {
		match_pmos_code[i] = 0;
		match_nmos_code[i] = 0;
	}
	for (i = 0; i < 7; i++) {
		match_ical_code[i] = 0;
		match_rcal_code[i] = 0;
	}


	do {
		/*Clear ICM_CFG value*/
		writeReg(base, 0x10010, 0x00000000);
		writeReg(base, 0x10014, 0x00000000);

		/*Set ICM_CFG value*/
		/*This will trigger recalibration of all stages*/
		writeReg(base, 0x10010, 0x00000001);
		writeReg(base, 0x10014, 0x00000000);

		/*is calibration done? polling on L3_CALIB_DONE_STATUS*/
		mask_status = mask_poll(base, 0xEF14, 0x2);
		if (mask_status == 0) {
			/*failure here is because of calibration done timeout*/
			fprintf(stderr, "pcie-phy: fail, SERDES initialization timed out\n");
			return mask_status;
		}

		p_code = mask_read(base, 0xEF18, 0xFFFFFFFF);   /*PMOS code*/
		n_code = mask_read(base, 0xEF1C, 0xFFFFFFFF);   /*NMOS code*/
		/*m_code = mask_read(0xFD40EF20, 0xFFFFFFFF)*/; /*MPHY code*/
		i_code = mask_read(base, 0xEF24, 0xFFFFFFFF);   /*ICAL code*/
		r_code = mask_read(base, 0xEF28, 0xFFFFFFFF);   /*RX code*/
		/*u_code = mask_read(0xFD40EF2C, 0xFFFFFFFF)*/; /*USB2 code*/

		/*PMOS code in acceptable range*/
		if ((p_code >= 0x26) && (p_code <= 0x3C)) {
			match_pmos_code[p_code - 0x26] += 1;
		}

		/*NMOS code in acceptable range*/
		if ((n_code >= 0x26) && (n_code <= 0x3C)) {
			match_nmos_code[n_code - 0x26] += 1;
		}

		/*PMOS code in acceptable range*/
		if ((i_code >= 0xC) && (i_code <= 0x12)) {
			match_ical_code[i_code - 0xC] += 1;
		}

		/*NMOS code in acceptable range*/
		if ((r_code >= 0x6) && (r_code <= 0xC)) {
			match_rcal_code[r_code - 0x6] += 1;
		}
	} while (repeat_count++ < 10);

	/*find the valid code which resulted in maximum times in 10 iterations*/
	for (i = 0; i < 23; i++) {
		if (match_pmos_code[i] >= match_pmos_code[0]) {
			match_pmos_code[0] = match_pmos_code[i];
			p_code = 0x26 + i;
		}
		if (match_nmos_code[i] >= match_nmos_code[0]) {
			match_nmos_code[0] = match_nmos_code[i];
			n_code = 0x26 + i;
		}
	}

	for (i = 0; i < 7; i++) {
		if (match_ical_code[i] >= match_ical_code[0]) {
			match_ical_code[0] = match_ical_code[i];
			i_code = 0xC + i;
		}
		if (match_rcal_code[i] >= match_rcal_code[0]) {
			match_rcal_code[0] = match_rcal_code[i];
			r_code = 0x6 + i;
		}
	}
	/*L3_TM_CALIB_DIG20[3] PSW MSB Override*/
	/*L3_TM_CALIB_DIG20[2:0]	PSW Code [4:2]*/
	L3_TM_CALIB_DIG20 = mask_read(base, 0xEC50, 0xFFFFFFF0); /*read DIG20*/
	L3_TM_CALIB_DIG20 = L3_TM_CALIB_DIG20 | 0x8 | ((p_code >> 2) & 0x7);


	/*L3_TM_CALIB_DIG19[7:6]	PSW Code [1:0]*/
	/*L3_TM_CALIB_DIG19[5]	PSW Override*/
	/*L3_TM_CALIB_DIG19[2]	NSW MSB Override*/
	/*L3_TM_CALIB_DIG19[1:0]	NSW Code [4:3]*/
	L3_TM_CALIB_DIG19 = mask_read(base, 0xEC4C, 0xFFFFFF18); /*read DIG19*/
	L3_TM_CALIB_DIG19 = L3_TM_CALIB_DIG19 | ((p_code & 0x3) << 6) | 0x20 | 0x4 | ((n_code >> 3) & 0x3);

	/*L3_TM_CALIB_DIG18[7:5]	NSW Code [2:0]*/
	/*L3_TM_CALIB_DIG18[4]	NSW Override*/
	L3_TM_CALIB_DIG18 = mask_read(base, 0xEC48, 0xFFFFFF0F); /*read DIG18*/
	L3_TM_CALIB_DIG18 = L3_TM_CALIB_DIG18 | ((n_code & 0x7) << 5) | 0x10;


	/*L3_TM_CALIB_DIG16[2:0]	RX Code [3:1]*/
	L3_TM_CALIB_DIG16 = mask_read(base, 0xEC40, 0xFFFFFFF8); /*read DIG16*/
	L3_TM_CALIB_DIG16 = L3_TM_CALIB_DIG16 | ((r_code >> 1) & 0x7);

	/*L3_TM_CALIB_DIG15[7]	RX Code [0]*/
	/*L3_TM_CALIB_DIG15[6]	RX CODE Override*/
	/*L3_TM_CALIB_DIG15[3]	ICAL MSB Override*/
	/*L3_TM_CALIB_DIG15[2:0]	ICAL Code [3:1]*/
	L3_TM_CALIB_DIG15 = mask_read(base, 0xEC3C, 0xFFFFFF30); /*read DIG15*/
	L3_TM_CALIB_DIG15 = L3_TM_CALIB_DIG15 | ((r_code & 0x1) << 7) | 0x40 | 0x8 | ((i_code >> 1) & 0x7);

	/*L3_TM_CALIB_DIG14[7]	ICAL Code [0]*/
	/*L3_TM_CALIB_DIG14[6]	ICAL Override*/
	L3_TM_CALIB_DIG14 = mask_read(base, 0xEC38, 0xFFFFFF3F); /*read DIG14*/
	L3_TM_CALIB_DIG14 = L3_TM_CALIB_DIG14 | ((i_code & 0x1) << 7) | 0x40;

	/*Forces the calibration values*/
	writeReg(base, 0xEC50, L3_TM_CALIB_DIG20);
	writeReg(base, 0xEC4C, L3_TM_CALIB_DIG19);
	writeReg(base, 0xEC48, L3_TM_CALIB_DIG18);
	writeReg(base, 0xEC40, L3_TM_CALIB_DIG16);
	writeReg(base, 0xEC3C, L3_TM_CALIB_DIG15);
	writeReg(base, 0xEC38, L3_TM_CALIB_DIG14);

	/* Enable PLL Coarse Code saturation Logic */
	writeReg(base, 0x2094, 0x00000010);
	writeReg(base, 0x6094, 0x00000010);
	writeReg(base, 0xA094, 0x00000010);
	writeReg(base, 0xE094, 0x00000010);

	return mask_status;
}


static int serdes_rst_seq(uint32_t *base, uint32_t pllsel, uint32_t lane3_protocol, uint32_t lane3_rate, uint32_t lane2_protocol, uint32_t lane2_rate, uint32_t lane1_protocol, uint32_t lane1_rate, uint32_t lane0_protocol, uint32_t lane0_rate)
{
	writeReg(base, SERDES_UPHY_SPARE0, 0x00000000);
	writeReg(base, SERDES_L0_TM_ANA_BYP_4, 0x00000040);
	writeReg(base, SERDES_L1_TM_ANA_BYP_4, 0x00000040);
	writeReg(base, SERDES_L2_TM_ANA_BYP_4, 0x00000040);
	writeReg(base, SERDES_L3_TM_ANA_BYP_4, 0x00000040);
	writeReg(base, SERDES_L0_TM_PLL_DIG_33, 0x00000080);
	writeReg(base, SERDES_L1_TM_PLL_DIG_33, 0x00000080);
	writeReg(base, SERDES_L2_TM_PLL_DIG_33, 0x00000080);
	writeReg(base, SERDES_L3_TM_PLL_DIG_33, 0x00000080);
	writeReg(base, SERDES_UPHY_SPARE0, 0x00000004);
	usleep(50);
	if (lane0_rate == 1)
		writeReg(base, SERDES_UPHY_SPARE0, 0x0000000E);
	writeReg(base, SERDES_UPHY_SPARE0, 0x00000006);
	if (lane0_rate == 1) {
		writeReg(base, SERDES_L0_TX_ANA_TM_3, 0x00000004);
		writeReg(base, SERDES_L1_TX_ANA_TM_3, 0x00000004);
		writeReg(base, SERDES_L2_TX_ANA_TM_3, 0x00000004);
		writeReg(base, SERDES_L3_TX_ANA_TM_3, 0x00000004);
		writeReg(base, SERDES_UPHY_SPARE0, 0x00000007);
		usleep(400);
		writeReg(base, SERDES_L0_TX_ANA_TM_3, 0x0000000C);
		writeReg(base, SERDES_L1_TX_ANA_TM_3, 0x0000000C);
		writeReg(base, SERDES_L2_TX_ANA_TM_3, 0x0000000C);
		writeReg(base, SERDES_L3_TX_ANA_TM_3, 0x0000000C);
		usleep(15);
		writeReg(base, SERDES_UPHY_SPARE0, 0x0000000F);
		usleep(100);
	}
	if (pllsel == 0) {
		int wait_ret = mask_poll(base, SERDES_L0_PLL_STATUS_READ_1, 0x00000010U);
		if (wait_ret == 0) {
			fprintf(stderr, "pcie-phy: timeout on PS GTR 0 ready wait\n");
		}
	}
	if (pllsel == 1) {
		int wait_ret = mask_poll(base, SERDES_L1_PLL_STATUS_READ_1, 0x00000010U);
		if (wait_ret == 0) {
			fprintf(stderr, "pcie-phy: timeout on PS GTR 1 ready wait\n");
		}
	}
	if (pllsel == 2) {
		int wait_ret = mask_poll(base, SERDES_L2_PLL_STATUS_READ_1, 0x00000010U);
		if (wait_ret == 0) {
			fprintf(stderr, "pcie-phy: timeout on PS GTR 2 ready wait\n");
		}
	}
	if (pllsel == 3) {
		int wait_ret = mask_poll(base, SERDES_L3_PLL_STATUS_READ_1, 0x00000010U);
		if (wait_ret == 0) {
			fprintf(stderr, "pcie-phy: timeout on PS GTR 3 ready wait\n");
		}
	}
	usleep(50);
	writeReg(base, SERDES_L0_TM_ANA_BYP_4, 0x000000C0);
	writeReg(base, SERDES_L1_TM_ANA_BYP_4, 0x000000C0);
	writeReg(base, SERDES_L2_TM_ANA_BYP_4, 0x000000C0);
	writeReg(base, SERDES_L3_TM_ANA_BYP_4, 0x000000C0);
	writeReg(base, SERDES_L0_TM_ANA_BYP_4, 0x00000080);
	writeReg(base, SERDES_L1_TM_ANA_BYP_4, 0x00000080);
	writeReg(base, SERDES_L2_TM_ANA_BYP_4, 0x00000080);
	writeReg(base, SERDES_L3_TM_ANA_BYP_4, 0x00000080);

	writeReg(base, SERDES_L0_TM_PLL_DIG_33, 0x000000C0);
	writeReg(base, SERDES_L1_TM_PLL_DIG_33, 0x000000C0);
	writeReg(base, SERDES_L2_TM_PLL_DIG_33, 0x000000C0);
	writeReg(base, SERDES_L3_TM_PLL_DIG_33, 0x000000C0);
	usleep(50);
	writeReg(base, SERDES_L0_TM_PLL_DIG_33, 0x00000080);
	writeReg(base, SERDES_L1_TM_PLL_DIG_33, 0x00000080);
	writeReg(base, SERDES_L2_TM_PLL_DIG_33, 0x00000080);
	writeReg(base, SERDES_L3_TM_PLL_DIG_33, 0x00000080);
	usleep(50);
	writeReg(base, SERDES_L0_TM_ANA_BYP_4, 0x00000000);
	writeReg(base, SERDES_L1_TM_ANA_BYP_4, 0x00000000);
	writeReg(base, SERDES_L2_TM_ANA_BYP_4, 0x00000000);
	writeReg(base, SERDES_L3_TM_ANA_BYP_4, 0x00000000);
	writeReg(base, SERDES_L0_TM_PLL_DIG_33, 0x00000000);
	writeReg(base, SERDES_L1_TM_PLL_DIG_33, 0x00000000);
	writeReg(base, SERDES_L2_TM_PLL_DIG_33, 0x00000000);
	writeReg(base, SERDES_L3_TM_PLL_DIG_33, 0x00000000);
	usleep(500);
	return 1;
}


static int serdes_bist_run(uint32_t *base, uint32_t lane_active)
{
	if (lane_active == 0) {
		writeRegMsk(base, SERDES_RX_PROT_BUS_WIDTH, 0x00000003U, 0x00000000U);
		writeRegMsk(base, SERDES_TX_PROT_BUS_WIDTH, 0x00000003U, 0x00000000U);
		writeRegMsk(base, SERDES_LPBK_CTRL0, 0x00000007U, 0x00000001U);
		writeReg(base, SERDES_L0_TM_DIG_22, 0x0020);
		writeReg(base, SERDES_L0_BIST_CTRL_1, (readReg(base, SERDES_L0_BIST_CTRL_1) | 0x1));
	}
	if (lane_active == 1) {
		writeRegMsk(base, SERDES_RX_PROT_BUS_WIDTH, 0x0000000CU, 0x00000000U);
		writeRegMsk(base, SERDES_TX_PROT_BUS_WIDTH, 0x0000000CU, 0x00000000U);
		writeRegMsk(base, SERDES_LPBK_CTRL0, 0x00000070U, 0x00000010U);
		writeReg(base, SERDES_L1_TM_DIG_22, 0x0020);
		writeReg(base, SERDES_L1_BIST_CTRL_1, (readReg(base, SERDES_L1_BIST_CTRL_1) | 0x1));
	}
	if (lane_active == 2) {
		writeRegMsk(base, SERDES_RX_PROT_BUS_WIDTH, 0x00000030U, 0x00000000U);
		writeRegMsk(base, SERDES_TX_PROT_BUS_WIDTH, 0x00000030U, 0x00000000U);
		writeRegMsk(base, SERDES_LPBK_CTRL1, 0x00000007U, 0x00000001U);
		writeReg(base, SERDES_L2_TM_DIG_22, 0x0020);
		writeReg(base, SERDES_L2_BIST_CTRL_1, (readReg(base, SERDES_L2_BIST_CTRL_1) | 0x1));
	}
	if (lane_active == 3) {
		writeRegMsk(base, SERDES_TX_PROT_BUS_WIDTH, 0x000000C0U, 0x00000000U);
		writeRegMsk(base, SERDES_RX_PROT_BUS_WIDTH, 0x000000C0U, 0x00000000U);
		writeRegMsk(base, SERDES_LPBK_CTRL1, 0x00000070U, 0x00000010U);
		writeReg(base, SERDES_L3_TM_DIG_22, 0x0020);
		writeReg(base, SERDES_L3_BIST_CTRL_1, (readReg(base, SERDES_L3_BIST_CTRL_1) | 0x1));
	}
	usleep(100);
	return (1);
}


static int serdes_bist_result(uint32_t *base, uint32_t lane_active)
{
	uint32_t pkt_cnt_l0, pkt_cnt_h0, err_cnt_l0, err_cnt_h0;
	if (lane_active == 0) {
		pkt_cnt_l0 = readReg(base, SERDES_L0_BIST_PKT_CTR_L);
		pkt_cnt_h0 = readReg(base, SERDES_L0_BIST_PKT_CTR_H);
		err_cnt_l0 = readReg(base, SERDES_L0_BIST_ERR_CTR_L);
		err_cnt_h0 = readReg(base, SERDES_L0_BIST_ERR_CTR_H);
	}
	if (lane_active == 1) {
		pkt_cnt_l0 = readReg(base, SERDES_L1_BIST_PKT_CTR_L);
		pkt_cnt_h0 = readReg(base, SERDES_L1_BIST_PKT_CTR_H);
		err_cnt_l0 = readReg(base, SERDES_L1_BIST_ERR_CTR_L);
		err_cnt_h0 = readReg(base, SERDES_L1_BIST_ERR_CTR_H);
	}
	if (lane_active == 2) {
		pkt_cnt_l0 = readReg(base, SERDES_L2_BIST_PKT_CTR_L);
		pkt_cnt_h0 = readReg(base, SERDES_L2_BIST_PKT_CTR_H);
		err_cnt_l0 = readReg(base, SERDES_L2_BIST_ERR_CTR_L);
		err_cnt_h0 = readReg(base, SERDES_L2_BIST_ERR_CTR_H);
	}
	if (lane_active == 3) {
		pkt_cnt_l0 = readReg(base, SERDES_L3_BIST_PKT_CTR_L);
		pkt_cnt_h0 = readReg(base, SERDES_L3_BIST_PKT_CTR_H);
		err_cnt_l0 = readReg(base, SERDES_L3_BIST_ERR_CTR_L);
		err_cnt_h0 = readReg(base, SERDES_L3_BIST_ERR_CTR_H);
	}
	if (lane_active == 0)
		writeReg(base, SERDES_L0_BIST_CTRL_1, 0x0);
	if (lane_active == 1)
		writeReg(base, SERDES_L1_BIST_CTRL_1, 0x0);
	if (lane_active == 2)
		writeReg(base, SERDES_L2_BIST_CTRL_1, 0x0);
	if (lane_active == 3)
		writeReg(base, SERDES_L3_BIST_CTRL_1, 0x0);
	if ((err_cnt_l0 > 0) || (err_cnt_h0 > 0) || ((pkt_cnt_l0 == 0) && (pkt_cnt_h0 == 0)))
		return (0);
	return (1);
}


static int serdes_bist_static_settings(uint32_t *base, uint32_t lane_active)
{
	if (lane_active == 0) {
		writeReg(base, SERDES_L0_BIST_CTRL_1, (readReg(base, SERDES_L0_BIST_CTRL_1) & 0xFFFFFF1F));
		writeReg(base, SERDES_L0_BIST_FILLER_OUT, 0x1);
		writeReg(base, SERDES_L0_BIST_FORCE_MK_RST, 0x1);
		writeReg(base, SERDES_L0_TM_DIG_22, 0x0020);
		writeReg(base, SERDES_L0_BIST_CTRL_2, 0x0);
		writeReg(base, SERDES_L0_BIST_RUN_LEN_L, 0xF4);
		writeReg(base, SERDES_L0_BIST_ERR_INJ_POINT_L, 0x0);
		writeReg(base, SERDES_L0_BIST_RUNLEN_ERR_INJ_H, 0x0);
		writeReg(base, SERDES_L0_BIST_IDLE_TIME, 0x00);
		writeReg(base, SERDES_L0_BIST_MARKER_L, 0xFB);
		writeReg(base, SERDES_L0_BIST_IDLE_CHAR_L, 0xFF);
		writeReg(base, SERDES_L0_BIST_MARKER_IDLE_H, 0x0);
		writeReg(base, SERDES_L0_BIST_LOW_PULSE_TIME, 0x00);
		writeReg(base, SERDES_L0_BIST_TOTAL_PULSE_TIME, 0x00);
		writeReg(base, SERDES_L0_BIST_TEST_PAT_1, 0x4A);
		writeReg(base, SERDES_L0_BIST_TEST_PAT_2, 0x4A);
		writeReg(base, SERDES_L0_BIST_TEST_PAT_3, 0x4A);
		writeReg(base, SERDES_L0_BIST_TEST_PAT_4, 0x4A);
		writeReg(base, SERDES_L0_BIST_TEST_PAT_MSBS, 0x0);
		writeReg(base, SERDES_L0_BIST_PKT_NUM, 0x14);
		writeReg(base, SERDES_L0_BIST_FRM_IDLE_TIME, 0x02);
		writeReg(base, SERDES_L0_BIST_CTRL_1, (readReg(base, SERDES_L0_BIST_CTRL_1) & 0xFFFFFF1F));
	}
	if (lane_active == 1) {
		writeReg(base, SERDES_L1_BIST_CTRL_1, (readReg(base, SERDES_L1_BIST_CTRL_1) & 0xFFFFFF1F));
		writeReg(base, SERDES_L1_BIST_FILLER_OUT, 0x1);
		writeReg(base, SERDES_L1_BIST_FORCE_MK_RST, 0x1);
		writeReg(base, SERDES_L1_TM_DIG_22, 0x0020);
		writeReg(base, SERDES_L1_BIST_CTRL_2, 0x0);
		writeReg(base, SERDES_L1_BIST_RUN_LEN_L, 0xF4);
		writeReg(base, SERDES_L1_BIST_ERR_INJ_POINT_L, 0x0);
		writeReg(base, SERDES_L1_BIST_RUNLEN_ERR_INJ_H, 0x0);
		writeReg(base, SERDES_L1_BIST_IDLE_TIME, 0x00);
		writeReg(base, SERDES_L1_BIST_MARKER_L, 0xFB);
		writeReg(base, SERDES_L1_BIST_IDLE_CHAR_L, 0xFF);
		writeReg(base, SERDES_L1_BIST_MARKER_IDLE_H, 0x0);
		writeReg(base, SERDES_L1_BIST_LOW_PULSE_TIME, 0x00);
		writeReg(base, SERDES_L1_BIST_TOTAL_PULSE_TIME, 0x00);
		writeReg(base, SERDES_L1_BIST_TEST_PAT_1, 0x4A);
		writeReg(base, SERDES_L1_BIST_TEST_PAT_2, 0x4A);
		writeReg(base, SERDES_L1_BIST_TEST_PAT_3, 0x4A);
		writeReg(base, SERDES_L1_BIST_TEST_PAT_4, 0x4A);
		writeReg(base, SERDES_L1_BIST_TEST_PAT_MSBS, 0x0);
		writeReg(base, SERDES_L1_BIST_PKT_NUM, 0x14);
		writeReg(base, SERDES_L1_BIST_FRM_IDLE_TIME, 0x02);
		writeReg(base, SERDES_L1_BIST_CTRL_1, (readReg(base, SERDES_L1_BIST_CTRL_1) & 0xFFFFFF1F));
	}

	if (lane_active == 2) {
		writeReg(base, SERDES_L2_BIST_CTRL_1, (readReg(base, SERDES_L2_BIST_CTRL_1) & 0xFFFFFF1F));
		writeReg(base, SERDES_L2_BIST_FILLER_OUT, 0x1);
		writeReg(base, SERDES_L2_BIST_FORCE_MK_RST, 0x1);
		writeReg(base, SERDES_L2_TM_DIG_22, 0x0020);
		writeReg(base, SERDES_L2_BIST_CTRL_2, 0x0);
		writeReg(base, SERDES_L2_BIST_RUN_LEN_L, 0xF4);
		writeReg(base, SERDES_L2_BIST_ERR_INJ_POINT_L, 0x0);
		writeReg(base, SERDES_L2_BIST_RUNLEN_ERR_INJ_H, 0x0);
		writeReg(base, SERDES_L2_BIST_IDLE_TIME, 0x00);
		writeReg(base, SERDES_L2_BIST_MARKER_L, 0xFB);
		writeReg(base, SERDES_L2_BIST_IDLE_CHAR_L, 0xFF);
		writeReg(base, SERDES_L2_BIST_MARKER_IDLE_H, 0x0);
		writeReg(base, SERDES_L2_BIST_LOW_PULSE_TIME, 0x00);
		writeReg(base, SERDES_L2_BIST_TOTAL_PULSE_TIME, 0x00);
		writeReg(base, SERDES_L2_BIST_TEST_PAT_1, 0x4A);
		writeReg(base, SERDES_L2_BIST_TEST_PAT_2, 0x4A);
		writeReg(base, SERDES_L2_BIST_TEST_PAT_3, 0x4A);
		writeReg(base, SERDES_L2_BIST_TEST_PAT_4, 0x4A);
		writeReg(base, SERDES_L2_BIST_TEST_PAT_MSBS, 0x0);
		writeReg(base, SERDES_L2_BIST_PKT_NUM, 0x14);
		writeReg(base, SERDES_L2_BIST_FRM_IDLE_TIME, 0x02);
		writeReg(base, SERDES_L2_BIST_CTRL_1, (readReg(base, SERDES_L2_BIST_CTRL_1) & 0xFFFFFF1F));
	}

	if (lane_active == 3) {
		writeReg(base, SERDES_L3_BIST_CTRL_1, (readReg(base, SERDES_L3_BIST_CTRL_1) & 0xFFFFFF1F));
		writeReg(base, SERDES_L3_BIST_FILLER_OUT, 0x1);
		writeReg(base, SERDES_L3_BIST_FORCE_MK_RST, 0x1);
		writeReg(base, SERDES_L3_TM_DIG_22, 0x0020);
		writeReg(base, SERDES_L3_BIST_CTRL_2, 0x0);
		writeReg(base, SERDES_L3_BIST_RUN_LEN_L, 0xF4);
		writeReg(base, SERDES_L3_BIST_ERR_INJ_POINT_L, 0x0);
		writeReg(base, SERDES_L3_BIST_RUNLEN_ERR_INJ_H, 0x0);
		writeReg(base, SERDES_L3_BIST_IDLE_TIME, 0x00);
		writeReg(base, SERDES_L3_BIST_MARKER_L, 0xFB);
		writeReg(base, SERDES_L3_BIST_IDLE_CHAR_L, 0xFF);
		writeReg(base, SERDES_L3_BIST_MARKER_IDLE_H, 0x0);
		writeReg(base, SERDES_L3_BIST_LOW_PULSE_TIME, 0x00);
		writeReg(base, SERDES_L3_BIST_TOTAL_PULSE_TIME, 0x00);
		writeReg(base, SERDES_L3_BIST_TEST_PAT_1, 0x4A);
		writeReg(base, SERDES_L3_BIST_TEST_PAT_2, 0x4A);
		writeReg(base, SERDES_L3_BIST_TEST_PAT_3, 0x4A);
		writeReg(base, SERDES_L3_BIST_TEST_PAT_4, 0x4A);
		writeReg(base, SERDES_L3_BIST_TEST_PAT_MSBS, 0x0);
		writeReg(base, SERDES_L3_BIST_PKT_NUM, 0x14);
		writeReg(base, SERDES_L3_BIST_FRM_IDLE_TIME, 0x02);
		writeReg(base, SERDES_L3_BIST_CTRL_1, (readReg(base, SERDES_L3_BIST_CTRL_1) & 0xFFFFFF1F));
	}
	return (1);
}


static int serdes_illcalib_pcie_gen1(
		uint32_t *base,
		uint32_t pllsel,
		uint32_t lane3_protocol, uint32_t lane3_rate,
		uint32_t lane2_protocol, uint32_t lane2_rate,
		uint32_t lane1_protocol, uint32_t lane1_rate,
		uint32_t lane0_protocol, uint32_t lane0_rate,
		uint32_t gen2_calib)
{
	uint64_t tempbistresult;
	uint32_t currbistresult[4];
	uint32_t prevbistresult[4];
	uint32_t itercount = 0;
	uint32_t ill12_val[4], ill1_val[4];
	uint32_t loop = 0;
	uint32_t iterresult[8];
	uint32_t meancount[4];
	uint32_t bistpasscount[4];
	uint32_t meancountalt[4];
	uint32_t meancountalt_bistpasscount[4];
	uint32_t lane0_active;
	uint32_t lane1_active;
	uint32_t lane2_active;
	uint32_t lane3_active;

	lane0_active = (lane0_protocol == 1);
	lane1_active = (lane1_protocol == 1);
	lane2_active = (lane2_protocol == 1);
	lane3_active = (lane3_protocol == 1);
	for (loop = 0; loop <= 3; loop++) {
		iterresult[loop] = 0;
		iterresult[loop + 4] = 0;
		meancountalt[loop] = 0;
		meancountalt_bistpasscount[loop] = 0;
		meancount[loop] = 0;
		prevbistresult[loop] = 0;
		bistpasscount[loop] = 0;
	}
	itercount = 0;
	if (lane0_active)
		serdes_bist_static_settings(base, 0);
	if (lane1_active)
		serdes_bist_static_settings(base, 1);
	if (lane2_active)
		serdes_bist_static_settings(base, 2);
	if (lane3_active)
		serdes_bist_static_settings(base, 3);
	do {
		if (gen2_calib != 1) {
			if (lane0_active == 1)
				ill1_val[0] = ((0x04 + itercount * 8) % 0x100);
			if (lane0_active == 1)
				ill12_val[0] = ((0x04 + itercount * 8) >= 0x100) ? 0x10 : 0x00;
			if (lane1_active == 1)
				ill1_val[1] = ((0x04 + itercount * 8) % 0x100);
			if (lane1_active == 1)
				ill12_val[1] = ((0x04 + itercount * 8) >= 0x100) ? 0x10 : 0x00;
			if (lane2_active == 1)
				ill1_val[2] = ((0x04 + itercount * 8) % 0x100);
			if (lane2_active == 1)
				ill12_val[2] = ((0x04 + itercount * 8) >= 0x100) ? 0x10 : 0x00;
			if (lane3_active == 1)
				ill1_val[3] = ((0x04 + itercount * 8) % 0x100);
			if (lane3_active == 1)
				ill12_val[3] = ((0x04 + itercount * 8) >= 0x100) ? 0x10 : 0x00;

			if (lane0_active == 1)
				writeReg(base, SERDES_L0_TM_E_ILL1, ill1_val[0]);
			if (lane0_active == 1)
				writeRegMsk(base, SERDES_L0_TM_ILL12, 0x000000F0U, ill12_val[0]);
			if (lane1_active == 1)
				writeReg(base, SERDES_L1_TM_E_ILL1, ill1_val[1]);
			if (lane1_active == 1)
				writeRegMsk(base, SERDES_L1_TM_ILL12, 0x000000F0U, ill12_val[1]);
			if (lane2_active == 1)
				writeReg(base, SERDES_L2_TM_E_ILL1, ill1_val[2]);
			if (lane2_active == 1)
				writeRegMsk(base, SERDES_L2_TM_ILL12, 0x000000F0U, ill12_val[2]);
			if (lane3_active == 1)
				writeReg(base, SERDES_L3_TM_E_ILL1, ill1_val[3]);
			if (lane3_active == 1)
				writeRegMsk(base, SERDES_L3_TM_ILL12, 0x000000F0U, ill12_val[3]);
		}
		if (gen2_calib == 1) {
			if (lane0_active == 1)
				ill1_val[0] = ((0x104 + itercount * 8) % 0x100);
			if (lane0_active == 1)
				ill12_val[0] = ((0x104 + itercount * 8) >= 0x200) ? 0x02 : 0x01;
			if (lane1_active == 1)
				ill1_val[1] = ((0x104 + itercount * 8) % 0x100);
			if (lane1_active == 1)
				ill12_val[1] = ((0x104 + itercount * 8) >= 0x200) ? 0x02 : 0x01;
			if (lane2_active == 1)
				ill1_val[2] = ((0x104 + itercount * 8) % 0x100);
			if (lane2_active == 1)
				ill12_val[2] = ((0x104 + itercount * 8) >= 0x200) ? 0x02 : 0x01;
			if (lane3_active == 1)
				ill1_val[3] = ((0x104 + itercount * 8) % 0x100);
			if (lane3_active == 1)
				ill12_val[3] = ((0x104 + itercount * 8) >= 0x200) ? 0x02 : 0x01;

			if (lane0_active == 1)
				writeReg(base, SERDES_L0_TM_E_ILL2, ill1_val[0]);
			if (lane0_active == 1)
				writeRegMsk(base, SERDES_L0_TM_ILL12, 0x0000000FU, ill12_val[0]);
			if (lane1_active == 1)
				writeReg(base, SERDES_L1_TM_E_ILL2, ill1_val[1]);
			if (lane1_active == 1)
				writeRegMsk(base, SERDES_L1_TM_ILL12, 0x0000000FU, ill12_val[1]);
			if (lane2_active == 1)
				writeReg(base, SERDES_L2_TM_E_ILL2, ill1_val[2]);
			if (lane2_active == 1)
				writeRegMsk(base, SERDES_L2_TM_ILL12, 0x0000000FU, ill12_val[2]);
			if (lane3_active == 1)
				writeReg(base, SERDES_L3_TM_E_ILL2, ill1_val[3]);
			if (lane3_active == 1)
				writeRegMsk(base, SERDES_L3_TM_ILL12, 0x0000000FU, ill12_val[3]);
		}

		if (lane0_active == 1)
			writeRegMsk(base, SERDES_L0_TM_ANA_BYP_7, 0x00000030U, 0x00000010U);
		if (lane1_active == 1)
			writeRegMsk(base, SERDES_L1_TM_ANA_BYP_7, 0x00000030U, 0x00000010U);
		if (lane2_active == 1)
			writeRegMsk(base, SERDES_L2_TM_ANA_BYP_7, 0x00000030U, 0x00000010U);
		if (lane3_active == 1)
			writeRegMsk(base, SERDES_L3_TM_ANA_BYP_7, 0x00000030U, 0x00000010U);
		if (lane0_active == 1)
			currbistresult[0] = 0;
		if (lane1_active == 1)
			currbistresult[1] = 0;
		if (lane2_active == 1)
			currbistresult[2] = 0;
		if (lane3_active == 1)
			currbistresult[3] = 0;
		serdes_rst_seq(base, pllsel, lane3_protocol, lane3_rate, lane2_protocol, lane2_rate, lane1_protocol, lane1_rate, lane0_protocol, lane0_rate);
		if (lane3_active == 1)
			serdes_bist_run(base, 3);
		if (lane2_active == 1)
			serdes_bist_run(base, 2);
		if (lane1_active == 1)
			serdes_bist_run(base, 1);
		if (lane0_active == 1)
			serdes_bist_run(base, 0);
		tempbistresult = 0;
		if (lane3_active == 1)
			tempbistresult = tempbistresult | serdes_bist_result(base, 3);
		tempbistresult = tempbistresult << 1;
		if (lane2_active == 1)
			tempbistresult = tempbistresult | serdes_bist_result(base, 2);
		tempbistresult = tempbistresult << 1;
		if (lane1_active == 1)
			tempbistresult = tempbistresult | serdes_bist_result(base, 1);
		tempbistresult = tempbistresult << 1;
		if (lane0_active == 1)
			tempbistresult = tempbistresult | serdes_bist_result(base, 0);
		writeReg(base, SERDES_UPHY_SPARE0, 0x0);
		writeReg(base, SERDES_UPHY_SPARE0, 0x2);

		if (itercount < 32) {
			iterresult[0] = ((iterresult[0] << 1) | ((tempbistresult & 0x1) == 0x1));
			iterresult[1] = ((iterresult[1] << 1) | ((tempbistresult & 0x2) == 0x2));
			iterresult[2] = ((iterresult[2] << 1) | ((tempbistresult & 0x4) == 0x4));
			iterresult[3] = ((iterresult[3] << 1) | ((tempbistresult & 0x8) == 0x8));
		}
		else {
			iterresult[4] = ((iterresult[4] << 1) | ((tempbistresult & 0x1) == 0x1));
			iterresult[5] = ((iterresult[5] << 1) | ((tempbistresult & 0x2) == 0x2));
			iterresult[6] = ((iterresult[6] << 1) | ((tempbistresult & 0x4) == 0x4));
			iterresult[7] = ((iterresult[7] << 1) | ((tempbistresult & 0x8) == 0x8));
		}
		currbistresult[0] = currbistresult[0] | ((tempbistresult & 0x1) == 1);
		currbistresult[1] = currbistresult[1] | ((tempbistresult & 0x2) == 0x2);
		currbistresult[2] = currbistresult[2] | ((tempbistresult & 0x4) == 0x4);
		currbistresult[3] = currbistresult[3] | ((tempbistresult & 0x8) == 0x8);

		for (loop = 0; loop <= 3; loop++) {
			if ((currbistresult[loop] == 1) && (prevbistresult[loop] == 1))
				bistpasscount[loop] = bistpasscount[loop] + 1;
			if ((bistpasscount[loop] < 4) && (currbistresult[loop] == 0) && (itercount > 2)) {
				if (meancountalt_bistpasscount[loop] < bistpasscount[loop]) {
					meancountalt_bistpasscount[loop] = bistpasscount[loop];
					meancountalt[loop] = ((itercount - 1) - ((bistpasscount[loop] + 1) / 2));
				}
				bistpasscount[loop] = 0;
			}
			if ((meancount[loop] == 0) && (bistpasscount[loop] >= 4) && ((currbistresult[loop] == 0) || (itercount == 63)) && (prevbistresult[loop] == 1))
				meancount[loop] = (itercount - 1) - ((bistpasscount[loop] + 1) / 2);
			prevbistresult[loop] = currbistresult[loop];
		}
	} while (++itercount < 64);

	for (loop = 0; loop <= 3; loop++) {
		if ((lane0_active == 0) && (loop == 0))
			continue;
		if ((lane1_active == 0) && (loop == 1))
			continue;
		if ((lane2_active == 0) && (loop == 2))
			continue;
		if ((lane3_active == 0) && (loop == 3))
			continue;

		if (meancount[loop] == 0)
			meancount[loop] = meancountalt[loop];


		if (gen2_calib != 1) {
			ill1_val[loop] = ((0x04 + meancount[loop] * 8) % 0x100);
			ill12_val[loop] = ((0x04 + meancount[loop] * 8) >= 0x100) ? 0x10 : 0x00;
#ifdef XFSBL_DEBUG
			writeReg(base, 0xFFFE0000 + loop * 4, iterresult[loop]);
			writeReg(base, 0xFFFE0010 + loop * 4, iterresult[loop + 4]);
			writeReg(base, 0xFFFE0020 + loop * 4, bistpasscount[loop]);
			writeReg(base, 0xFFFE0030 + loop * 4, meancount[loop]);
#endif
		}
		if (gen2_calib == 1) {
			ill1_val[loop] = ((0x104 + meancount[loop] * 8) % 0x100);
			ill12_val[loop] = ((0x104 + meancount[loop] * 8) >= 0x200) ? 0x02 : 0x01;
#ifdef XFSBL_DEBUG
			writeReg(base, 0xFFFE0040 + loop * 4, iterresult[loop]);
			writeReg(base, 0xFFFE0050 + loop * 4, iterresult[loop + 4]);
			writeReg(base, 0xFFFE0060 + loop * 4, bistpasscount[loop]);
			writeReg(base, 0xFFFE0070 + loop * 4, meancount[loop]);
#endif
		}
	}
	if (gen2_calib != 1) {
		if (lane0_active == 1)
			writeReg(base, SERDES_L0_TM_E_ILL1, ill1_val[0]);
		if (lane0_active == 1)
			writeRegMsk(base, SERDES_L0_TM_ILL12, 0x000000F0U, ill12_val[0]);
		if (lane1_active == 1)
			writeReg(base, SERDES_L1_TM_E_ILL1, ill1_val[1]);
		if (lane1_active == 1)
			writeRegMsk(base, SERDES_L1_TM_ILL12, 0x000000F0U, ill12_val[1]);
		if (lane2_active == 1)
			writeReg(base, SERDES_L2_TM_E_ILL1, ill1_val[2]);
		if (lane2_active == 1)
			writeRegMsk(base, SERDES_L2_TM_ILL12, 0x000000F0U, ill12_val[2]);
		if (lane3_active == 1)
			writeReg(base, SERDES_L3_TM_E_ILL1, ill1_val[3]);
		if (lane3_active == 1)
			writeRegMsk(base, SERDES_L3_TM_ILL12, 0x000000F0U, ill12_val[3]);
	}
	if (gen2_calib == 1) {
		if (lane0_active == 1)
			writeReg(base, SERDES_L0_TM_E_ILL2, ill1_val[0]);
		if (lane0_active == 1)
			writeRegMsk(base, SERDES_L0_TM_ILL12, 0x0000000FU, ill12_val[0]);
		if (lane1_active == 1)
			writeReg(base, SERDES_L1_TM_E_ILL2, ill1_val[1]);
		if (lane1_active == 1)
			writeRegMsk(base, SERDES_L1_TM_ILL12, 0x0000000FU, ill12_val[1]);
		if (lane2_active == 1)
			writeReg(base, SERDES_L2_TM_E_ILL2, ill1_val[2]);
		if (lane2_active == 1)
			writeRegMsk(base, SERDES_L2_TM_ILL12, 0x0000000FU, ill12_val[2]);
		if (lane3_active == 1)
			writeReg(base, SERDES_L3_TM_E_ILL2, ill1_val[3]);
		if (lane3_active == 1)
			writeRegMsk(base, SERDES_L3_TM_ILL12, 0x0000000FU, ill12_val[3]);
	}


	if (lane0_active == 1)
		writeRegMsk(base, SERDES_L0_TM_ANA_BYP_7, 0x00000030U, 0x00000000U);
	if (lane1_active == 1)
		writeRegMsk(base, SERDES_L1_TM_ANA_BYP_7, 0x00000030U, 0x00000000U);
	if (lane2_active == 1)
		writeRegMsk(base, SERDES_L2_TM_ANA_BYP_7, 0x00000030U, 0x00000000U);
	if (lane3_active == 1)
		writeRegMsk(base, SERDES_L3_TM_ANA_BYP_7, 0x00000030U, 0x00000000U);

	writeReg(base, SERDES_UPHY_SPARE0, 0);
	if (lane0_active == 1) {
		writeReg(base, SERDES_L0_BIST_CTRL_1, 0);
		writeReg(base, SERDES_L0_BIST_CTRL_2, 0);
		writeReg(base, SERDES_L0_BIST_RUN_LEN_L, 0);
		writeReg(base, SERDES_L0_BIST_ERR_INJ_POINT_L, 0);
		writeReg(base, SERDES_L0_BIST_RUNLEN_ERR_INJ_H, 0);
		writeReg(base, SERDES_L0_BIST_IDLE_TIME, 0);
		writeReg(base, SERDES_L0_BIST_MARKER_L, 0);
		writeReg(base, SERDES_L0_BIST_IDLE_CHAR_L, 0);
		writeReg(base, SERDES_L0_BIST_MARKER_IDLE_H, 0);
		writeReg(base, SERDES_L0_BIST_LOW_PULSE_TIME, 0);
		writeReg(base, SERDES_L0_BIST_TOTAL_PULSE_TIME, 0);
		writeReg(base, SERDES_L0_BIST_TEST_PAT_1, 0);
		writeReg(base, SERDES_L0_BIST_TEST_PAT_2, 0);
		writeReg(base, SERDES_L0_BIST_TEST_PAT_3, 0);
		writeReg(base, SERDES_L0_BIST_TEST_PAT_4, 0);
		writeReg(base, SERDES_L0_BIST_TEST_PAT_MSBS, 0);
		writeReg(base, SERDES_L0_BIST_PKT_NUM, 0);
		writeReg(base, SERDES_L0_BIST_FRM_IDLE_TIME, 0);
		writeReg(base, SERDES_L0_BIST_PKT_CTR_L, 0);
		writeReg(base, SERDES_L0_BIST_PKT_CTR_H, 0);
		writeReg(base, SERDES_L0_BIST_ERR_CTR_L, 0);
		writeReg(base, SERDES_L0_BIST_ERR_CTR_H, 0);
		writeReg(base, SERDES_L0_BIST_FILLER_OUT, 1);
		writeReg(base, SERDES_L0_BIST_FORCE_MK_RST, 0);
		writeReg(base, SERDES_L0_TM_DIG_22, 0);
		writeRegMsk(base, SERDES_RX_PROT_BUS_WIDTH, 0x00000003U, 0x00000001U);
		writeRegMsk(base, SERDES_TX_PROT_BUS_WIDTH, 0x00000003U, 0x00000001U);
		writeRegMsk(base, SERDES_LPBK_CTRL0, 0x00000007U, 0x00000000U);
	}
	if (lane1_active == 1) {
		writeReg(base, SERDES_L1_BIST_CTRL_1, 0);
		writeReg(base, SERDES_L1_BIST_CTRL_2, 0);
		writeReg(base, SERDES_L1_BIST_RUN_LEN_L, 0);
		writeReg(base, SERDES_L1_BIST_ERR_INJ_POINT_L, 0);
		writeReg(base, SERDES_L1_BIST_RUNLEN_ERR_INJ_H, 0);
		writeReg(base, SERDES_L1_BIST_IDLE_TIME, 0);
		writeReg(base, SERDES_L1_BIST_MARKER_L, 0);
		writeReg(base, SERDES_L1_BIST_IDLE_CHAR_L, 0);
		writeReg(base, SERDES_L1_BIST_MARKER_IDLE_H, 0);
		writeReg(base, SERDES_L1_BIST_LOW_PULSE_TIME, 0);
		writeReg(base, SERDES_L1_BIST_TOTAL_PULSE_TIME, 0);
		writeReg(base, SERDES_L1_BIST_TEST_PAT_1, 0);
		writeReg(base, SERDES_L1_BIST_TEST_PAT_2, 0);
		writeReg(base, SERDES_L1_BIST_TEST_PAT_3, 0);
		writeReg(base, SERDES_L1_BIST_TEST_PAT_4, 0);
		writeReg(base, SERDES_L1_BIST_TEST_PAT_MSBS, 0);
		writeReg(base, SERDES_L1_BIST_PKT_NUM, 0);
		writeReg(base, SERDES_L1_BIST_FRM_IDLE_TIME, 0);
		writeReg(base, SERDES_L1_BIST_PKT_CTR_L, 0);
		writeReg(base, SERDES_L1_BIST_PKT_CTR_H, 0);
		writeReg(base, SERDES_L1_BIST_ERR_CTR_L, 0);
		writeReg(base, SERDES_L1_BIST_ERR_CTR_H, 0);
		writeReg(base, SERDES_L1_BIST_FILLER_OUT, 1);
		writeReg(base, SERDES_L1_BIST_FORCE_MK_RST, 0);
		writeReg(base, SERDES_L1_TM_DIG_22, 0);
		writeRegMsk(base, SERDES_RX_PROT_BUS_WIDTH, 0x0000000CU, 0x00000004U);
		writeRegMsk(base, SERDES_TX_PROT_BUS_WIDTH, 0x0000000CU, 0x00000004U);
		writeRegMsk(base, SERDES_LPBK_CTRL0, 0x00000070U, 0x00000000U);
	}
	if (lane2_active == 1) {
		writeReg(base, SERDES_L2_BIST_CTRL_1, 0);
		writeReg(base, SERDES_L2_BIST_CTRL_2, 0);
		writeReg(base, SERDES_L2_BIST_RUN_LEN_L, 0);
		writeReg(base, SERDES_L2_BIST_ERR_INJ_POINT_L, 0);
		writeReg(base, SERDES_L2_BIST_RUNLEN_ERR_INJ_H, 0);
		writeReg(base, SERDES_L2_BIST_IDLE_TIME, 0);
		writeReg(base, SERDES_L2_BIST_MARKER_L, 0);
		writeReg(base, SERDES_L2_BIST_IDLE_CHAR_L, 0);
		writeReg(base, SERDES_L2_BIST_MARKER_IDLE_H, 0);
		writeReg(base, SERDES_L2_BIST_LOW_PULSE_TIME, 0);
		writeReg(base, SERDES_L2_BIST_TOTAL_PULSE_TIME, 0);
		writeReg(base, SERDES_L2_BIST_TEST_PAT_1, 0);
		writeReg(base, SERDES_L2_BIST_TEST_PAT_2, 0);
		writeReg(base, SERDES_L2_BIST_TEST_PAT_3, 0);
		writeReg(base, SERDES_L2_BIST_TEST_PAT_4, 0);
		writeReg(base, SERDES_L2_BIST_TEST_PAT_MSBS, 0);
		writeReg(base, SERDES_L2_BIST_PKT_NUM, 0);
		writeReg(base, SERDES_L2_BIST_FRM_IDLE_TIME, 0);
		writeReg(base, SERDES_L2_BIST_PKT_CTR_L, 0);
		writeReg(base, SERDES_L2_BIST_PKT_CTR_H, 0);
		writeReg(base, SERDES_L2_BIST_ERR_CTR_L, 0);
		writeReg(base, SERDES_L2_BIST_ERR_CTR_H, 0);
		writeReg(base, SERDES_L2_BIST_FILLER_OUT, 1);
		writeReg(base, SERDES_L2_BIST_FORCE_MK_RST, 0);
		writeReg(base, SERDES_L2_TM_DIG_22, 0);
		writeRegMsk(base, SERDES_RX_PROT_BUS_WIDTH, 0x00000030U, 0x00000010U);
		writeRegMsk(base, SERDES_TX_PROT_BUS_WIDTH, 0x00000030U, 0x00000010U);
		writeRegMsk(base, SERDES_LPBK_CTRL1, 0x00000007U, 0x00000000U);
	}
	if (lane3_active == 1) {
		writeReg(base, SERDES_L3_BIST_CTRL_1, 0);
		writeReg(base, SERDES_L3_BIST_CTRL_2, 0);
		writeReg(base, SERDES_L3_BIST_RUN_LEN_L, 0);
		writeReg(base, SERDES_L3_BIST_ERR_INJ_POINT_L, 0);
		writeReg(base, SERDES_L3_BIST_RUNLEN_ERR_INJ_H, 0);
		writeReg(base, SERDES_L3_BIST_IDLE_TIME, 0);
		writeReg(base, SERDES_L3_BIST_MARKER_L, 0);
		writeReg(base, SERDES_L3_BIST_IDLE_CHAR_L, 0);
		writeReg(base, SERDES_L3_BIST_MARKER_IDLE_H, 0);
		writeReg(base, SERDES_L3_BIST_LOW_PULSE_TIME, 0);
		writeReg(base, SERDES_L3_BIST_TOTAL_PULSE_TIME, 0);
		writeReg(base, SERDES_L3_BIST_TEST_PAT_1, 0);
		writeReg(base, SERDES_L3_BIST_TEST_PAT_2, 0);
		writeReg(base, SERDES_L3_BIST_TEST_PAT_3, 0);
		writeReg(base, SERDES_L3_BIST_TEST_PAT_4, 0);
		writeReg(base, SERDES_L3_BIST_TEST_PAT_MSBS, 0);
		writeReg(base, SERDES_L3_BIST_PKT_NUM, 0);
		writeReg(base, SERDES_L3_BIST_FRM_IDLE_TIME, 0);
		writeReg(base, SERDES_L3_BIST_PKT_CTR_L, 0);
		writeReg(base, SERDES_L3_BIST_PKT_CTR_H, 0);
		writeReg(base, SERDES_L3_BIST_ERR_CTR_L, 0);
		writeReg(base, SERDES_L3_BIST_ERR_CTR_H, 0);
		writeReg(base, SERDES_L3_BIST_FILLER_OUT, 1);
		writeReg(base, SERDES_L3_BIST_FORCE_MK_RST, 0);
		writeReg(base, SERDES_L3_TM_DIG_22, 0);
		writeRegMsk(base, SERDES_RX_PROT_BUS_WIDTH, 0x000000C0U, 0x00000040U);
		writeRegMsk(base, SERDES_TX_PROT_BUS_WIDTH, 0x000000C0U, 0x00000040U);
		writeRegMsk(base, SERDES_LPBK_CTRL1, 0x00000070U, 0x00000000U);
	}
	return 1;
}


static int serdes_illcalib(uint32_t *base,
		uint32_t lane3_protocol, uint32_t lane3_rate,
		uint32_t lane2_protocol, uint32_t lane2_rate,
		uint32_t lane1_protocol, uint32_t lane1_rate,
		uint32_t lane0_protocol, uint32_t lane0_rate)
// Protocol values
// pcie = 1; sata = 2; usb = 3; dp = 4; sgmii = 5
// Rate values
// pcie_gen1 = 0; pcie_gen2 = 1;
// sata_gen1 = 1; sata_gen2 = 2; sata_gen3 = 3;
// usb = 0; sgmii = 0; DP = 0;
{
	unsigned int rdata = 0;
	unsigned int sata_gen2 = 1;
	// unsigned int temp_ill12=0;
	// unsigned int temp_PLL_REF_SEL_OFFSET;
	// unsigned int temp_TM_IQ_ILL1;
	// unsigned int temp_TM_E_ILL1;
	// unsigned int temp_tx_dig_tm_61;
	// unsigned int temp_tm_dig_6;
	// unsigned int temp_pll_fbdiv_frac_3_msb_offset;

	if ((lane0_protocol == 2) || (lane0_protocol == 1)) {
		writeReg(base, SERDES_L0_TM_IQ_ILL7, 0xF3);
		writeReg(base, SERDES_L0_TM_E_ILL7, 0xF3);
		writeReg(base, SERDES_L0_TM_IQ_ILL8, 0xF3);
		writeReg(base, SERDES_L0_TM_E_ILL8, 0xF3);
	}
	if ((lane1_protocol == 2) || (lane1_protocol == 1)) {
		writeReg(base, SERDES_L1_TM_IQ_ILL7, 0xF3);
		writeReg(base, SERDES_L1_TM_E_ILL7, 0xF3);
		writeReg(base, SERDES_L1_TM_IQ_ILL8, 0xF3);
		writeReg(base, SERDES_L1_TM_E_ILL8, 0xF3);
	}
	if ((lane2_protocol == 2) || (lane2_protocol == 1)) {
		writeReg(base, SERDES_L2_TM_IQ_ILL7, 0xF3);
		writeReg(base, SERDES_L2_TM_E_ILL7, 0xF3);
		writeReg(base, SERDES_L2_TM_IQ_ILL8, 0xF3);
		writeReg(base, SERDES_L2_TM_E_ILL8, 0xF3);
	}
	if ((lane3_protocol == 2) || (lane3_protocol == 1)) {
		writeReg(base, SERDES_L3_TM_IQ_ILL7, 0xF3);
		writeReg(base, SERDES_L3_TM_E_ILL7, 0xF3);
		writeReg(base, SERDES_L3_TM_IQ_ILL8, 0xF3);
		writeReg(base, SERDES_L3_TM_E_ILL8, 0xF3);
	}

	if (sata_gen2 == 1) {
		rdata = readReg(base, SERDES_UPHY_SPARE0);
		rdata = (rdata & 0xDF);
		writeReg(base, SERDES_UPHY_SPARE0, rdata);
	}

	// PCIe settings
	// If lane-0 is PCIe, we need to run pcie dynamic search on all active pcie lanes
	// and reset sequence on all active lanes
	if (lane0_protocol == 1) {
		if (lane0_rate == 0) {
			serdes_illcalib_pcie_gen1(base, 0, lane3_protocol, lane3_rate, lane2_protocol, lane2_rate, lane1_protocol, lane1_rate, lane0_protocol, 0, 0);
		}
		else {
			serdes_illcalib_pcie_gen1(base, 0, lane3_protocol, lane3_rate, lane2_protocol, lane2_rate, lane1_protocol, lane1_rate, lane0_protocol, 0, 0);
			serdes_illcalib_pcie_gen1(base, 0, lane3_protocol, lane3_rate, lane2_protocol, lane2_rate, lane1_protocol, lane1_rate, lane0_protocol, lane0_rate, 1);
		}
	}


	return 1;
}


unsigned long psu_serdes_init_data(uint32_t *base)
{
	/*
	 * SERDES INITIALIZATION
	 */
	/*
	 * GT REFERENCE CLOCK SOURCE SELECTION
	 */
	/*
	* Register : PLL_REF_SEL0 @ 0XFD410000

	* PLL0 Reference Selection. 0x0 - 5MHz, 0x1 - 9.6MHz, 0x2 - 10MHz, 0x3 - 1
	* 2MHz, 0x4 - 13MHz, 0x5 - 19.2MHz, 0x6 - 20MHz, 0x7 - 24MHz, 0x8 - 26MHz,
	*  0x9 - 27MHz, 0xA - 38.4MHz, 0xB - 40MHz, 0xC - 52MHz, 0xD - 100MHz, 0xE
	*  - 108MHz, 0xF - 125MHz, 0x10 - 135MHz, 0x11 - 150 MHz. 0x12 to 0x1F - R
	* eserved
	*  PSU_SERDES_PLL_REF_SEL0_PLLREFSEL0                          0xD

	* PLL0 Reference Selection Register
	* (OFFSET, MASK, VALUE)      (0XFD410000, 0x0000001FU ,0x0000000DU)
	*/
	writeRegMsk(base, SERDES_PLL_REF_SEL0_OFFSET, 0x0000001FU, 0x0000000DU);
	/*##################################################################### */

	/*
	* Register : PLL_REF_SEL1 @ 0XFD410004

	* PLL1 Reference Selection. 0x0 - 5MHz, 0x1 - 9.6MHz, 0x2 - 10MHz, 0x3 - 1
	* 2MHz, 0x4 - 13MHz, 0x5 - 19.2MHz, 0x6 - 20MHz, 0x7 - 24MHz, 0x8 - 26MHz,
	*  0x9 - 27MHz, 0xA - 38.4MHz, 0xB - 40MHz, 0xC - 52MHz, 0xD - 100MHz, 0xE
	*  - 108MHz, 0xF - 125MHz, 0x10 - 135MHz, 0x11 - 150 MHz. 0x12 to 0x1F - R
	* eserved
	*  PSU_SERDES_PLL_REF_SEL1_PLLREFSEL1                          0xD

	* PLL1 Reference Selection Register
	* (OFFSET, MASK, VALUE)      (0XFD410004, 0x0000001FU ,0x0000000DU)
	*/
	writeRegMsk(base, SERDES_PLL_REF_SEL1_OFFSET, 0x0000001FU, 0x0000000DU);
	/*##################################################################### */

	/*
	* Register : PLL_REF_SEL2 @ 0XFD410008

	* PLL2 Reference Selection. 0x0 - 5MHz, 0x1 - 9.6MHz, 0x2 - 10MHz, 0x3 - 1
	* 2MHz, 0x4 - 13MHz, 0x5 - 19.2MHz, 0x6 - 20MHz, 0x7 - 24MHz, 0x8 - 26MHz,
	*  0x9 - 27MHz, 0xA - 38.4MHz, 0xB - 40MHz, 0xC - 52MHz, 0xD - 100MHz, 0xE
	*  - 108MHz, 0xF - 125MHz, 0x10 - 135MHz, 0x11 - 150 MHz. 0x12 to 0x1F - R
	* eserved
	*  PSU_SERDES_PLL_REF_SEL2_PLLREFSEL2                          0x11

	* PLL2 Reference Selection Register
	* (OFFSET, MASK, VALUE)      (0XFD410008, 0x0000001FU ,0x00000011U)
	*/
	writeRegMsk(base, SERDES_PLL_REF_SEL2_OFFSET, 0x0000001FU, 0x00000011U);
	/*##################################################################### */

	/*
	* Register : PLL_REF_SEL3 @ 0XFD41000C

	* PLL3 Reference Selection. 0x0 - 5MHz, 0x1 - 9.6MHz, 0x2 - 10MHz, 0x3 - 1
	* 2MHz, 0x4 - 13MHz, 0x5 - 19.2MHz, 0x6 - 20MHz, 0x7 - 24MHz, 0x8 - 26MHz,
	*  0x9 - 27MHz, 0xA - 38.4MHz, 0xB - 40MHz, 0xC - 52MHz, 0xD - 100MHz, 0xE
	*  - 108MHz, 0xF - 125MHz, 0x10 - 135MHz, 0x11 - 150 MHz. 0x12 to 0x1F - R
	* eserved
	*  PSU_SERDES_PLL_REF_SEL3_PLLREFSEL3                          0x9

	* PLL3 Reference Selection Register
	* (OFFSET, MASK, VALUE)      (0XFD41000C, 0x0000001FU ,0x00000009U)
	*/
	writeRegMsk(base, SERDES_PLL_REF_SEL3_OFFSET, 0x0000001FU, 0x00000009U);
	/*##################################################################### */

	/*
	 * GT REFERENCE CLOCK FREQUENCY SELECTION
	 */
	/*
	* Register : L0_L0_REF_CLK_SEL @ 0XFD402860

	* Sel of lane 0 ref clock local mux. Set to 1 to select lane 0 slicer outp
	* ut. Set to 0 to select lane0 ref clock mux output.
	*  PSU_SERDES_L0_L0_REF_CLK_SEL_L0_REF_CLK_LCL_SEL             0x0

	* Bit 2 of lane 0 ref clock mux one hot sel. Set to 1 to select lane 2 sli
	* cer output from ref clock network
	*  PSU_SERDES_L0_L0_REF_CLK_SEL_L0_REF_CLK_SEL_2               0x1

	* Lane0 Ref Clock Selection Register
	* (OFFSET, MASK, VALUE)      (0XFD402860, 0x00000084U ,0x00000004U)
	*/
	writeRegMsk(base, SERDES_L0_L0_REF_CLK_SEL_OFFSET,
			0x00000084U, 0x00000004U);
	/*##################################################################### */

	/*
	* Register : L0_L1_REF_CLK_SEL @ 0XFD402864

	* Sel of lane 1 ref clock local mux. Set to 1 to select lane 1 slicer outp
	* ut. Set to 0 to select lane1 ref clock mux output.
	*  PSU_SERDES_L0_L1_REF_CLK_SEL_L1_REF_CLK_LCL_SEL             0x0

	* Bit 2 of lane 1 ref clock mux one hot sel. Set to 1 to select lane 2 sli
	* cer output from ref clock network
	*  PSU_SERDES_L0_L1_REF_CLK_SEL_L1_REF_CLK_SEL_2               0x1

	* Lane1 Ref Clock Selection Register
	* (OFFSET, MASK, VALUE)      (0XFD402864, 0x00000084U ,0x00000004U)
	*/
	writeRegMsk(base, SERDES_L0_L1_REF_CLK_SEL_OFFSET,
			0x00000084U, 0x00000004U);
	/*##################################################################### */

	/*
	* Register : L0_L2_REF_CLK_SEL @ 0XFD402868

	* Sel of lane 2 ref clock local mux. Set to 1 to select lane 1 slicer outp
	* ut. Set to 0 to select lane2 ref clock mux output.
	*  PSU_SERDES_L0_L2_REF_CLK_SEL_L2_REF_CLK_LCL_SEL             0x0

	* Bit 1 of lane 2 ref clock mux one hot sel. Set to 1 to select lane 1 sli
	* cer output from ref clock network
	*  PSU_SERDES_L0_L2_REF_CLK_SEL_L2_REF_CLK_SEL_1               0x1

	* Lane2 Ref Clock Selection Register
	* (OFFSET, MASK, VALUE)      (0XFD402868, 0x00000082U ,0x00000002U)
	*/
	writeRegMsk(base, SERDES_L0_L2_REF_CLK_SEL_OFFSET,
			0x00000082U, 0x00000002U);
	/*##################################################################### */

	/*
	* Register : L0_L3_REF_CLK_SEL @ 0XFD40286C

	* Sel of lane 3 ref clock local mux. Set to 1 to select lane 3 slicer outp
	* ut. Set to 0 to select lane3 ref clock mux output.
	*  PSU_SERDES_L0_L3_REF_CLK_SEL_L3_REF_CLK_LCL_SEL             0x1

	* Lane3 Ref Clock Selection Register
	* (OFFSET, MASK, VALUE)      (0XFD40286C, 0x00000080U ,0x00000080U)
	*/
	writeRegMsk(base, SERDES_L0_L3_REF_CLK_SEL_OFFSET,
			0x00000080U, 0x00000080U);
	/*##################################################################### */

	/*
	 * ENABLE SPREAD SPECTRUM
	 */
	/*
	* Register : L1_TM_PLL_DIG_37 @ 0XFD406094

	* Enable/Disable coarse code satureation limiting logic
	*  PSU_SERDES_L1_TM_PLL_DIG_37_TM_ENABLE_COARSE_SATURATION     0x1

	* Test mode register 37
	* (OFFSET, MASK, VALUE)      (0XFD406094, 0x00000010U ,0x00000010U)
	*/
	writeRegMsk(base, SERDES_L1_TM_PLL_DIG_37_OFFSET,
			0x00000010U, 0x00000010U);
	/*##################################################################### */

	/*
	* Register : L2_PLL_SS_STEPS_0_LSB @ 0XFD40A368

	* Spread Spectrum No of Steps [7:0]
	*  PSU_SERDES_L2_PLL_SS_STEPS_0_LSB_SS_NUM_OF_STEPS_0_LSB      0x18

	* Spread Spectrum No of Steps bits 7:0
	* (OFFSET, MASK, VALUE)      (0XFD40A368, 0x000000FFU ,0x00000018U)
	*/
	writeRegMsk(base, SERDES_L2_PLL_SS_STEPS_0_LSB_OFFSET,
			0x000000FFU, 0x00000018U);
	/*##################################################################### */

	/*
	* Register : L2_PLL_SS_STEPS_1_MSB @ 0XFD40A36C

	* Spread Spectrum No of Steps [10:8]
	*  PSU_SERDES_L2_PLL_SS_STEPS_1_MSB_SS_NUM_OF_STEPS_1_MSB      0x3

	* Spread Spectrum No of Steps bits 10:8
	* (OFFSET, MASK, VALUE)      (0XFD40A36C, 0x00000007U ,0x00000003U)
	*/
	writeRegMsk(base, SERDES_L2_PLL_SS_STEPS_1_MSB_OFFSET,
			0x00000007U, 0x00000003U);
	/*##################################################################### */

	/*
	* Register : L3_PLL_SS_STEPS_0_LSB @ 0XFD40E368

	* Spread Spectrum No of Steps [7:0]
	*  PSU_SERDES_L3_PLL_SS_STEPS_0_LSB_SS_NUM_OF_STEPS_0_LSB      0x58

	* Spread Spectrum No of Steps bits 7:0
	* (OFFSET, MASK, VALUE)      (0XFD40E368, 0x000000FFU ,0x00000058U)
	*/
	writeRegMsk(base, SERDES_L3_PLL_SS_STEPS_0_LSB_OFFSET,
			0x000000FFU, 0x00000058U);
	/*##################################################################### */

	/*
	* Register : L3_PLL_SS_STEPS_1_MSB @ 0XFD40E36C

	* Spread Spectrum No of Steps [10:8]
	*  PSU_SERDES_L3_PLL_SS_STEPS_1_MSB_SS_NUM_OF_STEPS_1_MSB      0x3

	* Spread Spectrum No of Steps bits 10:8
	* (OFFSET, MASK, VALUE)      (0XFD40E36C, 0x00000007U ,0x00000003U)
	*/
	writeRegMsk(base, SERDES_L3_PLL_SS_STEPS_1_MSB_OFFSET,
			0x00000007U, 0x00000003U);
	/*##################################################################### */

	/*
	* Register : L1_PLL_SS_STEPS_0_LSB @ 0XFD406368

	* Spread Spectrum No of Steps [7:0]
	*  PSU_SERDES_L1_PLL_SS_STEPS_0_LSB_SS_NUM_OF_STEPS_0_LSB      0x22

	* Spread Spectrum No of Steps bits 7:0
	* (OFFSET, MASK, VALUE)      (0XFD406368, 0x000000FFU ,0x00000022U)
	*/
	writeRegMsk(base, SERDES_L1_PLL_SS_STEPS_0_LSB_OFFSET,
			0x000000FFU, 0x00000022U);
	/*##################################################################### */

	/*
	* Register : L1_PLL_SS_STEPS_1_MSB @ 0XFD40636C

	* Spread Spectrum No of Steps [10:8]
	*  PSU_SERDES_L1_PLL_SS_STEPS_1_MSB_SS_NUM_OF_STEPS_1_MSB      0x4

	* Spread Spectrum No of Steps bits 10:8
	* (OFFSET, MASK, VALUE)      (0XFD40636C, 0x00000007U ,0x00000004U)
	*/
	writeRegMsk(base, SERDES_L1_PLL_SS_STEPS_1_MSB_OFFSET,
			0x00000007U, 0x00000004U);
	/*##################################################################### */

	/*
	* Register : L1_PLL_SS_STEP_SIZE_0_LSB @ 0XFD406370

	* Step Size for Spread Spectrum [7:0]
	*  PSU_SERDES_L1_PLL_SS_STEP_SIZE_0_LSB_SS_STEP_SIZE_0_LSB     0xED

	* Step Size for Spread Spectrum LSB
	* (OFFSET, MASK, VALUE)      (0XFD406370, 0x000000FFU ,0x000000EDU)
	*/
	writeRegMsk(base, SERDES_L1_PLL_SS_STEP_SIZE_0_LSB_OFFSET,
			0x000000FFU, 0x000000EDU);
	/*##################################################################### */

	/*
	* Register : L1_PLL_SS_STEP_SIZE_1 @ 0XFD406374

	* Step Size for Spread Spectrum [15:8]
	*  PSU_SERDES_L1_PLL_SS_STEP_SIZE_1_SS_STEP_SIZE_1             0x55

	* Step Size for Spread Spectrum 1
	* (OFFSET, MASK, VALUE)      (0XFD406374, 0x000000FFU ,0x00000055U)
	*/
	writeRegMsk(base, SERDES_L1_PLL_SS_STEP_SIZE_1_OFFSET,
			0x000000FFU, 0x00000055U);
	/*##################################################################### */

	/*
	* Register : L1_PLL_SS_STEP_SIZE_2 @ 0XFD406378

	* Step Size for Spread Spectrum [23:16]
	*  PSU_SERDES_L1_PLL_SS_STEP_SIZE_2_SS_STEP_SIZE_2             0x1

	* Step Size for Spread Spectrum 2
	* (OFFSET, MASK, VALUE)      (0XFD406378, 0x000000FFU ,0x00000001U)
	*/
	writeRegMsk(base, SERDES_L1_PLL_SS_STEP_SIZE_2_OFFSET,
			0x000000FFU, 0x00000001U);
	/*##################################################################### */

	/*
	* Register : L1_PLL_SS_STEP_SIZE_3_MSB @ 0XFD40637C

	* Step Size for Spread Spectrum [25:24]
	*  PSU_SERDES_L1_PLL_SS_STEP_SIZE_3_MSB_SS_STEP_SIZE_3_MSB     0x0

	* Enable/Disable test mode force on SS step size
	*  PSU_SERDES_L1_PLL_SS_STEP_SIZE_3_MSB_FORCE_SS_STEP_SIZE     0x1

	* Enable/Disable test mode force on SS no of steps
	*  PSU_SERDES_L1_PLL_SS_STEP_SIZE_3_MSB_FORCE_SS_NUM_OF_STEPS  0x1

	* Enable force on enable Spread Spectrum
	* (OFFSET, MASK, VALUE)      (0XFD40637C, 0x00000033U ,0x00000030U)
	*/
	writeRegMsk(base, SERDES_L1_PLL_SS_STEP_SIZE_3_MSB_OFFSET,
			0x00000033U, 0x00000030U);
	/*##################################################################### */

	/*
	* Register : L2_PLL_SS_STEP_SIZE_0_LSB @ 0XFD40A370

	* Step Size for Spread Spectrum [7:0]
	*  PSU_SERDES_L2_PLL_SS_STEP_SIZE_0_LSB_SS_STEP_SIZE_0_LSB     0xD3

	* Step Size for Spread Spectrum LSB
	* (OFFSET, MASK, VALUE)      (0XFD40A370, 0x000000FFU ,0x000000D3U)
	*/
	writeRegMsk(base, SERDES_L2_PLL_SS_STEP_SIZE_0_LSB_OFFSET,
			0x000000FFU, 0x000000D3U);
	/*##################################################################### */

	/*
	* Register : L2_PLL_SS_STEP_SIZE_1 @ 0XFD40A374

	* Step Size for Spread Spectrum [15:8]
	*  PSU_SERDES_L2_PLL_SS_STEP_SIZE_1_SS_STEP_SIZE_1             0xDA

	* Step Size for Spread Spectrum 1
	* (OFFSET, MASK, VALUE)      (0XFD40A374, 0x000000FFU ,0x000000DAU)
	*/
	writeRegMsk(base, SERDES_L2_PLL_SS_STEP_SIZE_1_OFFSET,
			0x000000FFU, 0x000000DAU);
	/*##################################################################### */

	/*
	* Register : L2_PLL_SS_STEP_SIZE_2 @ 0XFD40A378

	* Step Size for Spread Spectrum [23:16]
	*  PSU_SERDES_L2_PLL_SS_STEP_SIZE_2_SS_STEP_SIZE_2             0x2

	* Step Size for Spread Spectrum 2
	* (OFFSET, MASK, VALUE)      (0XFD40A378, 0x000000FFU ,0x00000002U)
	*/
	writeRegMsk(base, SERDES_L2_PLL_SS_STEP_SIZE_2_OFFSET,
			0x000000FFU, 0x00000002U);
	/*##################################################################### */

	/*
	* Register : L2_PLL_SS_STEP_SIZE_3_MSB @ 0XFD40A37C

	* Step Size for Spread Spectrum [25:24]
	*  PSU_SERDES_L2_PLL_SS_STEP_SIZE_3_MSB_SS_STEP_SIZE_3_MSB     0x0

	* Enable/Disable test mode force on SS step size
	*  PSU_SERDES_L2_PLL_SS_STEP_SIZE_3_MSB_FORCE_SS_STEP_SIZE     0x1

	* Enable/Disable test mode force on SS no of steps
	*  PSU_SERDES_L2_PLL_SS_STEP_SIZE_3_MSB_FORCE_SS_NUM_OF_STEPS  0x1

	* Enable test mode forcing on enable Spread Spectrum
	*  PSU_SERDES_L2_PLL_SS_STEP_SIZE_3_MSB_TM_FORCE_EN_SS         0x1

	* Enable force on enable Spread Spectrum
	* (OFFSET, MASK, VALUE)      (0XFD40A37C, 0x000000B3U ,0x000000B0U)
	*/
	writeRegMsk(base, SERDES_L2_PLL_SS_STEP_SIZE_3_MSB_OFFSET,
			0x000000B3U, 0x000000B0U);
	/*##################################################################### */

	/*
	* Register : L3_PLL_SS_STEP_SIZE_0_LSB @ 0XFD40E370

	* Step Size for Spread Spectrum [7:0]
	*  PSU_SERDES_L3_PLL_SS_STEP_SIZE_0_LSB_SS_STEP_SIZE_0_LSB     0x7C

	* Step Size for Spread Spectrum LSB
	* (OFFSET, MASK, VALUE)      (0XFD40E370, 0x000000FFU ,0x0000007CU)
	*/
	writeRegMsk(base, SERDES_L3_PLL_SS_STEP_SIZE_0_LSB_OFFSET,
			0x000000FFU, 0x0000007CU);
	/*##################################################################### */

	/*
	* Register : L3_PLL_SS_STEP_SIZE_1 @ 0XFD40E374

	* Step Size for Spread Spectrum [15:8]
	*  PSU_SERDES_L3_PLL_SS_STEP_SIZE_1_SS_STEP_SIZE_1             0x33

	* Step Size for Spread Spectrum 1
	* (OFFSET, MASK, VALUE)      (0XFD40E374, 0x000000FFU ,0x00000033U)
	*/
	writeRegMsk(base, SERDES_L3_PLL_SS_STEP_SIZE_1_OFFSET,
			0x000000FFU, 0x00000033U);
	/*##################################################################### */

	/*
	* Register : L3_PLL_SS_STEP_SIZE_2 @ 0XFD40E378

	* Step Size for Spread Spectrum [23:16]
	*  PSU_SERDES_L3_PLL_SS_STEP_SIZE_2_SS_STEP_SIZE_2             0x2

	* Step Size for Spread Spectrum 2
	* (OFFSET, MASK, VALUE)      (0XFD40E378, 0x000000FFU ,0x00000002U)
	*/
	writeRegMsk(base, SERDES_L3_PLL_SS_STEP_SIZE_2_OFFSET,
			0x000000FFU, 0x00000002U);
	/*##################################################################### */

	/*
	* Register : L3_PLL_SS_STEP_SIZE_3_MSB @ 0XFD40E37C

	* Step Size for Spread Spectrum [25:24]
	*  PSU_SERDES_L3_PLL_SS_STEP_SIZE_3_MSB_SS_STEP_SIZE_3_MSB     0x0

	* Enable/Disable test mode force on SS step size
	*  PSU_SERDES_L3_PLL_SS_STEP_SIZE_3_MSB_FORCE_SS_STEP_SIZE     0x1

	* Enable/Disable test mode force on SS no of steps
	*  PSU_SERDES_L3_PLL_SS_STEP_SIZE_3_MSB_FORCE_SS_NUM_OF_STEPS  0x1

	* Enable force on enable Spread Spectrum
	* (OFFSET, MASK, VALUE)      (0XFD40E37C, 0x00000033U ,0x00000030U)
	*/
	writeRegMsk(base, SERDES_L3_PLL_SS_STEP_SIZE_3_MSB_OFFSET,
			0x00000033U, 0x00000030U);
	/*##################################################################### */

	/*
	* Register : L1_TM_DIG_6 @ 0XFD40506C

	* Bypass Descrambler
	*  PSU_SERDES_L1_TM_DIG_6_BYPASS_DESCRAM                       0x1

	* Enable Bypass for <1> TM_DIG_CTRL_6
	*  PSU_SERDES_L1_TM_DIG_6_FORCE_BYPASS_DESCRAM                 0x1

	* Data path test modes in decoder and descram
	* (OFFSET, MASK, VALUE)      (0XFD40506C, 0x00000003U ,0x00000003U)
	*/
	writeRegMsk(base, SERDES_L1_TM_DIG_6_OFFSET, 0x00000003U, 0x00000003U);
	/*##################################################################### */

	/*
	* Register : L1_TX_DIG_TM_61 @ 0XFD4040F4

	* Bypass scrambler signal
	*  PSU_SERDES_L1_TX_DIG_TM_61_BYPASS_SCRAM                     0x1

	* Enable/disable scrambler bypass signal
	*  PSU_SERDES_L1_TX_DIG_TM_61_FORCE_BYPASS_SCRAM               0x1

	* MPHY PLL Gear and bypass scrambler
	* (OFFSET, MASK, VALUE)      (0XFD4040F4, 0x00000003U ,0x00000003U)
	*/
	writeRegMsk(base, SERDES_L1_TX_DIG_TM_61_OFFSET,
			0x00000003U, 0x00000003U);
	/*##################################################################### */

	/*
	* Register : L2_PLL_FBDIV_FRAC_3_MSB @ 0XFD40A360

	* Enable test mode force on fractional mode enable
	*  PSU_SERDES_L2_PLL_FBDIV_FRAC_3_MSB_TM_FORCE_EN_FRAC         0x1

	* Fractional feedback division control and fractional value for feedback d
	* ivision bits 26:24
	* (OFFSET, MASK, VALUE)      (0XFD40A360, 0x00000040U ,0x00000040U)
	*/
	writeRegMsk(base, SERDES_L2_PLL_FBDIV_FRAC_3_MSB_OFFSET,
			0x00000040U, 0x00000040U);
	/*##################################################################### */

	/*
	* Register : L2_TM_DIG_6 @ 0XFD40906C

	* Bypass 8b10b decoder
	*  PSU_SERDES_L2_TM_DIG_6_BYPASS_DECODER                       0x1

	* Enable Bypass for <3> TM_DIG_CTRL_6
	*  PSU_SERDES_L2_TM_DIG_6_FORCE_BYPASS_DEC                     0x1

	* Bypass Descrambler
	*  PSU_SERDES_L2_TM_DIG_6_BYPASS_DESCRAM                       0x1

	* Enable Bypass for <1> TM_DIG_CTRL_6
	*  PSU_SERDES_L2_TM_DIG_6_FORCE_BYPASS_DESCRAM                 0x1

	* Data path test modes in decoder and descram
	* (OFFSET, MASK, VALUE)      (0XFD40906C, 0x0000000FU ,0x0000000FU)
	*/
	writeRegMsk(base, SERDES_L2_TM_DIG_6_OFFSET, 0x0000000FU, 0x0000000FU);
	/*##################################################################### */

	/*
	* Register : L2_TX_DIG_TM_61 @ 0XFD4080F4

	* Enable/disable encoder bypass signal
	*  PSU_SERDES_L2_TX_DIG_TM_61_BYPASS_ENC                       0x1

	* Bypass scrambler signal
	*  PSU_SERDES_L2_TX_DIG_TM_61_BYPASS_SCRAM                     0x1

	* Enable/disable scrambler bypass signal
	*  PSU_SERDES_L2_TX_DIG_TM_61_FORCE_BYPASS_SCRAM               0x1

	* MPHY PLL Gear and bypass scrambler
	* (OFFSET, MASK, VALUE)      (0XFD4080F4, 0x0000000BU ,0x0000000BU)
	*/
	writeRegMsk(base, SERDES_L2_TX_DIG_TM_61_OFFSET,
			0x0000000BU, 0x0000000BU);
	/*##################################################################### */

	/*
	 * ENABLE CHICKEN BIT FOR PCIE AND USB
	 */
	/*
	* Register : L0_TM_AUX_0 @ 0XFD4010CC

	* Spare- not used
	*  PSU_SERDES_L0_TM_AUX_0_BIT_2                                1

	* Spare registers
	* (OFFSET, MASK, VALUE)      (0XFD4010CC, 0x00000020U ,0x00000020U)
	*/
	writeRegMsk(base, SERDES_L0_TM_AUX_0_OFFSET, 0x00000020U, 0x00000020U);
	/*##################################################################### */

	/*
	* Register : L1_TM_AUX_0 @ 0XFD4050CC

	* Spare- not used
	*  PSU_SERDES_L1_TM_AUX_0_BIT_2                                1

	* Spare registers
	* (OFFSET, MASK, VALUE)      (0XFD4050CC, 0x00000020U ,0x00000020U)
	*/
	writeRegMsk(base, SERDES_L1_TM_AUX_0_OFFSET, 0x00000020U, 0x00000020U);
	/*##################################################################### */

	/*
	 * ENABLING EYE SURF
	 */
	/*
	* Register : L0_TM_DIG_8 @ 0XFD401074

	* Enable Eye Surf
	*  PSU_SERDES_L0_TM_DIG_8_EYESURF_ENABLE                       0x1

	* Test modes for Elastic buffer and enabling Eye Surf
	* (OFFSET, MASK, VALUE)      (0XFD401074, 0x00000010U ,0x00000010U)
	*/
	writeRegMsk(base, SERDES_L0_TM_DIG_8_OFFSET, 0x00000010U, 0x00000010U);
	/*##################################################################### */

	/*
	* Register : L1_TM_DIG_8 @ 0XFD405074

	* Enable Eye Surf
	*  PSU_SERDES_L1_TM_DIG_8_EYESURF_ENABLE                       0x1

	* Test modes for Elastic buffer and enabling Eye Surf
	* (OFFSET, MASK, VALUE)      (0XFD405074, 0x00000010U ,0x00000010U)
	*/
	writeRegMsk(base, SERDES_L1_TM_DIG_8_OFFSET, 0x00000010U, 0x00000010U);
	/*##################################################################### */

	/*
	* Register : L2_TM_DIG_8 @ 0XFD409074

	* Enable Eye Surf
	*  PSU_SERDES_L2_TM_DIG_8_EYESURF_ENABLE                       0x1

	* Test modes for Elastic buffer and enabling Eye Surf
	* (OFFSET, MASK, VALUE)      (0XFD409074, 0x00000010U ,0x00000010U)
	*/
	writeRegMsk(base, SERDES_L2_TM_DIG_8_OFFSET, 0x00000010U, 0x00000010U);
	/*##################################################################### */

	/*
	* Register : L3_TM_DIG_8 @ 0XFD40D074

	* Enable Eye Surf
	*  PSU_SERDES_L3_TM_DIG_8_EYESURF_ENABLE                       0x1

	* Test modes for Elastic buffer and enabling Eye Surf
	* (OFFSET, MASK, VALUE)      (0XFD40D074, 0x00000010U ,0x00000010U)
	*/
	writeRegMsk(base, SERDES_L3_TM_DIG_8_OFFSET, 0x00000010U, 0x00000010U);
	/*##################################################################### */

	/*
	 * ILL SETTINGS FOR GAIN AND LOCK SETTINGS
	 */
	/*
	* Register : L0_TM_MISC2 @ 0XFD40189C

	* ILL calib counts BYPASSED with calcode bits
	*  PSU_SERDES_L0_TM_MISC2_ILL_CAL_BYPASS_COUNTS                0x1

	* sampler cal
	* (OFFSET, MASK, VALUE)      (0XFD40189C, 0x00000080U ,0x00000080U)
	*/
	writeRegMsk(base, SERDES_L0_TM_MISC2_OFFSET, 0x00000080U, 0x00000080U);
	/*##################################################################### */

	/*
	* Register : L0_TM_IQ_ILL1 @ 0XFD4018F8

	* IQ ILL F0 CALCODE bypass value. MPHY : G1a, PCIE : Gen 1, SATA : Gen1 ,
	* USB3 : SS
	*  PSU_SERDES_L0_TM_IQ_ILL1_ILL_BYPASS_IQ_CALCODE_F0           0x64

	* iqpi cal code
	* (OFFSET, MASK, VALUE)      (0XFD4018F8, 0x000000FFU ,0x00000064U)
	*/
	writeRegMsk(base, SERDES_L0_TM_IQ_ILL1_OFFSET,
			0x000000FFU, 0x00000064U);
	/*##################################################################### */

	/*
	* Register : L0_TM_IQ_ILL2 @ 0XFD4018FC

	* IQ ILL F1 CALCODE bypass value. MPHY : G1b, PCIE : Gen2, SATA: Gen2
	*  PSU_SERDES_L0_TM_IQ_ILL2_ILL_BYPASS_IQ_CALCODE_F1           0x64

	* iqpi cal code
	* (OFFSET, MASK, VALUE)      (0XFD4018FC, 0x000000FFU ,0x00000064U)
	*/
	writeRegMsk(base, SERDES_L0_TM_IQ_ILL2_OFFSET,
			0x000000FFU, 0x00000064U);
	/*##################################################################### */

	/*
	* Register : L0_TM_ILL12 @ 0XFD401990

	* G1A pll ctr bypass value
	*  PSU_SERDES_L0_TM_ILL12_G1A_PLL_CTR_BYP_VAL                  0x11

	* ill pll counter values
	* (OFFSET, MASK, VALUE)      (0XFD401990, 0x000000FFU ,0x00000011U)
	*/
	writeRegMsk(base, SERDES_L0_TM_ILL12_OFFSET, 0x000000FFU, 0x00000011U);
	/*##################################################################### */

	/*
	* Register : L0_TM_E_ILL1 @ 0XFD401924

	* E ILL F0 CALCODE bypass value. MPHY : G1a, PCIE : Gen 1, SATA : Gen1 , U
	* SB3 : SS
	*  PSU_SERDES_L0_TM_E_ILL1_ILL_BYPASS_E_CALCODE_F0             0x4

	* epi cal code
	* (OFFSET, MASK, VALUE)      (0XFD401924, 0x000000FFU ,0x00000004U)
	*/
	writeRegMsk(base, SERDES_L0_TM_E_ILL1_OFFSET, 0x000000FFU, 0x00000004U);
	/*##################################################################### */

	/*
	* Register : L0_TM_E_ILL2 @ 0XFD401928

	* E ILL F1 CALCODE bypass value. MPHY : G1b, PCIE : Gen2, SATA: Gen2
	*  PSU_SERDES_L0_TM_E_ILL2_ILL_BYPASS_E_CALCODE_F1             0xFE

	* epi cal code
	* (OFFSET, MASK, VALUE)      (0XFD401928, 0x000000FFU ,0x000000FEU)
	*/
	writeRegMsk(base, SERDES_L0_TM_E_ILL2_OFFSET, 0x000000FFU, 0x000000FEU);
	/*##################################################################### */

	/*
	* Register : L0_TM_IQ_ILL3 @ 0XFD401900

	* IQ ILL F2CALCODE bypass value. MPHY : G2a, SATA : Gen3
	*  PSU_SERDES_L0_TM_IQ_ILL3_ILL_BYPASS_IQ_CALCODE_F2           0x64

	* iqpi cal code
	* (OFFSET, MASK, VALUE)      (0XFD401900, 0x000000FFU ,0x00000064U)
	*/
	writeRegMsk(base, SERDES_L0_TM_IQ_ILL3_OFFSET,
			0x000000FFU, 0x00000064U);
	/*##################################################################### */

	/*
	* Register : L0_TM_E_ILL3 @ 0XFD40192C

	* E ILL F2CALCODE bypass value. MPHY : G2a, SATA : Gen3
	*  PSU_SERDES_L0_TM_E_ILL3_ILL_BYPASS_E_CALCODE_F2             0x0

	* epi cal code
	* (OFFSET, MASK, VALUE)      (0XFD40192C, 0x000000FFU ,0x00000000U)
	*/
	writeRegMsk(base, SERDES_L0_TM_E_ILL3_OFFSET, 0x000000FFU, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : L0_TM_ILL8 @ 0XFD401980

	* ILL calibration code change wait time
	*  PSU_SERDES_L0_TM_ILL8_ILL_CAL_ITER_WAIT                     0xFF

	* ILL cal routine control
	* (OFFSET, MASK, VALUE)      (0XFD401980, 0x000000FFU ,0x000000FFU)
	*/
	writeRegMsk(base, SERDES_L0_TM_ILL8_OFFSET, 0x000000FFU, 0x000000FFU);
	/*##################################################################### */

	/*
	* Register : L0_TM_IQ_ILL8 @ 0XFD401914

	* IQ ILL polytrim bypass value
	*  PSU_SERDES_L0_TM_IQ_ILL8_ILL_BYPASS_IQ_POLYTRIM_VAL         0xF7

	* iqpi polytrim
	* (OFFSET, MASK, VALUE)      (0XFD401914, 0x000000FFU ,0x000000F7U)
	*/
	writeRegMsk(base, SERDES_L0_TM_IQ_ILL8_OFFSET,
			0x000000FFU, 0x000000F7U);
	/*##################################################################### */

	/*
	* Register : L0_TM_IQ_ILL9 @ 0XFD401918

	* bypass IQ polytrim
	*  PSU_SERDES_L0_TM_IQ_ILL9_ILL_BYPASS_IQ_POLYTIM              0x1

	* enables for lf,constant gm trim and polytirm
	* (OFFSET, MASK, VALUE)      (0XFD401918, 0x00000001U ,0x00000001U)
	*/
	writeRegMsk(base, SERDES_L0_TM_IQ_ILL9_OFFSET,
			0x00000001U, 0x00000001U);
	/*##################################################################### */

	/*
	* Register : L0_TM_E_ILL8 @ 0XFD401940

	* E ILL polytrim bypass value
	*  PSU_SERDES_L0_TM_E_ILL8_ILL_BYPASS_E_POLYTRIM_VAL           0xF7

	* epi polytrim
	* (OFFSET, MASK, VALUE)      (0XFD401940, 0x000000FFU ,0x000000F7U)
	*/
	writeRegMsk(base, SERDES_L0_TM_E_ILL8_OFFSET, 0x000000FFU, 0x000000F7U);
	/*##################################################################### */

	/*
	* Register : L0_TM_E_ILL9 @ 0XFD401944

	* bypass E polytrim
	*  PSU_SERDES_L0_TM_E_ILL9_ILL_BYPASS_E_POLYTIM                0x1

	* enables for lf,constant gm trim and polytirm
	* (OFFSET, MASK, VALUE)      (0XFD401944, 0x00000001U ,0x00000001U)
	*/
	writeRegMsk(base, SERDES_L0_TM_E_ILL9_OFFSET, 0x00000001U, 0x00000001U);
	/*##################################################################### */

	/*
	* Register : L0_TM_ILL13 @ 0XFD401994

	* ILL cal idle val refcnt
	*  PSU_SERDES_L0_TM_ILL13_ILL_CAL_IDLE_VAL_REFCNT              0x7

	* ill cal idle value count
	* (OFFSET, MASK, VALUE)      (0XFD401994, 0x00000007U ,0x00000007U)
	*/
	writeRegMsk(base, SERDES_L0_TM_ILL13_OFFSET, 0x00000007U, 0x00000007U);
	/*##################################################################### */

	/*
	* Register : L1_TM_MISC2 @ 0XFD40589C

	* ILL calib counts BYPASSED with calcode bits
	*  PSU_SERDES_L1_TM_MISC2_ILL_CAL_BYPASS_COUNTS                0x1

	* sampler cal
	* (OFFSET, MASK, VALUE)      (0XFD40589C, 0x00000080U ,0x00000080U)
	*/
	writeRegMsk(base, SERDES_L1_TM_MISC2_OFFSET, 0x00000080U, 0x00000080U);
	/*##################################################################### */

	/*
	* Register : L1_TM_IQ_ILL1 @ 0XFD4058F8

	* IQ ILL F0 CALCODE bypass value. MPHY : G1a, PCIE : Gen 1, SATA : Gen1 ,
	* USB3 : SS
	*  PSU_SERDES_L1_TM_IQ_ILL1_ILL_BYPASS_IQ_CALCODE_F0           0x64

	* iqpi cal code
	* (OFFSET, MASK, VALUE)      (0XFD4058F8, 0x000000FFU ,0x00000064U)
	*/
	writeRegMsk(base, SERDES_L1_TM_IQ_ILL1_OFFSET,
			0x000000FFU, 0x00000064U);
	/*##################################################################### */

	/*
	* Register : L1_TM_IQ_ILL2 @ 0XFD4058FC

	* IQ ILL F1 CALCODE bypass value. MPHY : G1b, PCIE : Gen2, SATA: Gen2
	*  PSU_SERDES_L1_TM_IQ_ILL2_ILL_BYPASS_IQ_CALCODE_F1           0x64

	* iqpi cal code
	* (OFFSET, MASK, VALUE)      (0XFD4058FC, 0x000000FFU ,0x00000064U)
	*/
	writeRegMsk(base, SERDES_L1_TM_IQ_ILL2_OFFSET,
			0x000000FFU, 0x00000064U);
	/*##################################################################### */

	/*
	* Register : L1_TM_ILL12 @ 0XFD405990

	* G1A pll ctr bypass value
	*  PSU_SERDES_L1_TM_ILL12_G1A_PLL_CTR_BYP_VAL                  0x10

	* ill pll counter values
	* (OFFSET, MASK, VALUE)      (0XFD405990, 0x000000FFU ,0x00000010U)
	*/
	writeRegMsk(base, SERDES_L1_TM_ILL12_OFFSET, 0x000000FFU, 0x00000010U);
	/*##################################################################### */

	/*
	* Register : L1_TM_E_ILL1 @ 0XFD405924

	* E ILL F0 CALCODE bypass value. MPHY : G1a, PCIE : Gen 1, SATA : Gen1 , U
	* SB3 : SS
	*  PSU_SERDES_L1_TM_E_ILL1_ILL_BYPASS_E_CALCODE_F0             0xFE

	* epi cal code
	* (OFFSET, MASK, VALUE)      (0XFD405924, 0x000000FFU ,0x000000FEU)
	*/
	writeRegMsk(base, SERDES_L1_TM_E_ILL1_OFFSET, 0x000000FFU, 0x000000FEU);
	/*##################################################################### */

	/*
	* Register : L1_TM_E_ILL2 @ 0XFD405928

	* E ILL F1 CALCODE bypass value. MPHY : G1b, PCIE : Gen2, SATA: Gen2
	*  PSU_SERDES_L1_TM_E_ILL2_ILL_BYPASS_E_CALCODE_F1             0x0

	* epi cal code
	* (OFFSET, MASK, VALUE)      (0XFD405928, 0x000000FFU ,0x00000000U)
	*/
	writeRegMsk(base, SERDES_L1_TM_E_ILL2_OFFSET, 0x000000FFU, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : L1_TM_IQ_ILL3 @ 0XFD405900

	* IQ ILL F2CALCODE bypass value. MPHY : G2a, SATA : Gen3
	*  PSU_SERDES_L1_TM_IQ_ILL3_ILL_BYPASS_IQ_CALCODE_F2           0x64

	* iqpi cal code
	* (OFFSET, MASK, VALUE)      (0XFD405900, 0x000000FFU ,0x00000064U)
	*/
	writeRegMsk(base, SERDES_L1_TM_IQ_ILL3_OFFSET,
			0x000000FFU, 0x00000064U);
	/*##################################################################### */

	/*
	* Register : L1_TM_E_ILL3 @ 0XFD40592C

	* E ILL F2CALCODE bypass value. MPHY : G2a, SATA : Gen3
	*  PSU_SERDES_L1_TM_E_ILL3_ILL_BYPASS_E_CALCODE_F2             0x0

	* epi cal code
	* (OFFSET, MASK, VALUE)      (0XFD40592C, 0x000000FFU ,0x00000000U)
	*/
	writeRegMsk(base, SERDES_L1_TM_E_ILL3_OFFSET, 0x000000FFU, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : L1_TM_ILL8 @ 0XFD405980

	* ILL calibration code change wait time
	*  PSU_SERDES_L1_TM_ILL8_ILL_CAL_ITER_WAIT                     0xFF

	* ILL cal routine control
	* (OFFSET, MASK, VALUE)      (0XFD405980, 0x000000FFU ,0x000000FFU)
	*/
	writeRegMsk(base, SERDES_L1_TM_ILL8_OFFSET, 0x000000FFU, 0x000000FFU);
	/*##################################################################### */

	/*
	* Register : L1_TM_IQ_ILL8 @ 0XFD405914

	* IQ ILL polytrim bypass value
	*  PSU_SERDES_L1_TM_IQ_ILL8_ILL_BYPASS_IQ_POLYTRIM_VAL         0xF7

	* iqpi polytrim
	* (OFFSET, MASK, VALUE)      (0XFD405914, 0x000000FFU ,0x000000F7U)
	*/
	writeRegMsk(base, SERDES_L1_TM_IQ_ILL8_OFFSET,
			0x000000FFU, 0x000000F7U);
	/*##################################################################### */

	/*
	* Register : L1_TM_IQ_ILL9 @ 0XFD405918

	* bypass IQ polytrim
	*  PSU_SERDES_L1_TM_IQ_ILL9_ILL_BYPASS_IQ_POLYTIM              0x1

	* enables for lf,constant gm trim and polytirm
	* (OFFSET, MASK, VALUE)      (0XFD405918, 0x00000001U ,0x00000001U)
	*/
	writeRegMsk(base, SERDES_L1_TM_IQ_ILL9_OFFSET,
			0x00000001U, 0x00000001U);
	/*##################################################################### */

	/*
	* Register : L1_TM_E_ILL8 @ 0XFD405940

	* E ILL polytrim bypass value
	*  PSU_SERDES_L1_TM_E_ILL8_ILL_BYPASS_E_POLYTRIM_VAL           0xF7

	* epi polytrim
	* (OFFSET, MASK, VALUE)      (0XFD405940, 0x000000FFU ,0x000000F7U)
	*/
	writeRegMsk(base, SERDES_L1_TM_E_ILL8_OFFSET, 0x000000FFU, 0x000000F7U);
	/*##################################################################### */

	/*
	* Register : L1_TM_E_ILL9 @ 0XFD405944

	* bypass E polytrim
	*  PSU_SERDES_L1_TM_E_ILL9_ILL_BYPASS_E_POLYTIM                0x1

	* enables for lf,constant gm trim and polytirm
	* (OFFSET, MASK, VALUE)      (0XFD405944, 0x00000001U ,0x00000001U)
	*/
	writeRegMsk(base, SERDES_L1_TM_E_ILL9_OFFSET, 0x00000001U, 0x00000001U);
	/*##################################################################### */

	/*
	* Register : L1_TM_ILL13 @ 0XFD405994

	* ILL cal idle val refcnt
	*  PSU_SERDES_L1_TM_ILL13_ILL_CAL_IDLE_VAL_REFCNT              0x7

	* ill cal idle value count
	* (OFFSET, MASK, VALUE)      (0XFD405994, 0x00000007U ,0x00000007U)
	*/
	writeRegMsk(base, SERDES_L1_TM_ILL13_OFFSET, 0x00000007U, 0x00000007U);
	/*##################################################################### */

	/*
	* Register : L2_TM_MISC2 @ 0XFD40989C

	* ILL calib counts BYPASSED with calcode bits
	*  PSU_SERDES_L2_TM_MISC2_ILL_CAL_BYPASS_COUNTS                0x1

	* sampler cal
	* (OFFSET, MASK, VALUE)      (0XFD40989C, 0x00000080U ,0x00000080U)
	*/
	writeRegMsk(base, SERDES_L2_TM_MISC2_OFFSET, 0x00000080U, 0x00000080U);
	/*##################################################################### */

	/*
	* Register : L2_TM_IQ_ILL1 @ 0XFD4098F8

	* IQ ILL F0 CALCODE bypass value. MPHY : G1a, PCIE : Gen 1, SATA : Gen1 ,
	* USB3 : SS
	*  PSU_SERDES_L2_TM_IQ_ILL1_ILL_BYPASS_IQ_CALCODE_F0           0x96

	* iqpi cal code
	* (OFFSET, MASK, VALUE)      (0XFD4098F8, 0x000000FFU ,0x00000096U)
	*/
	writeRegMsk(base, SERDES_L2_TM_IQ_ILL1_OFFSET,
			0x000000FFU, 0x00000096U);
	/*##################################################################### */

	/*
	* Register : L2_TM_IQ_ILL2 @ 0XFD4098FC

	* IQ ILL F1 CALCODE bypass value. MPHY : G1b, PCIE : Gen2, SATA: Gen2
	*  PSU_SERDES_L2_TM_IQ_ILL2_ILL_BYPASS_IQ_CALCODE_F1           0x96

	* iqpi cal code
	* (OFFSET, MASK, VALUE)      (0XFD4098FC, 0x000000FFU ,0x00000096U)
	*/
	writeRegMsk(base, SERDES_L2_TM_IQ_ILL2_OFFSET,
			0x000000FFU, 0x00000096U);
	/*##################################################################### */

	/*
	* Register : L2_TM_ILL12 @ 0XFD409990

	* G1A pll ctr bypass value
	*  PSU_SERDES_L2_TM_ILL12_G1A_PLL_CTR_BYP_VAL                  0x1

	* ill pll counter values
	* (OFFSET, MASK, VALUE)      (0XFD409990, 0x000000FFU ,0x00000001U)
	*/
	writeRegMsk(base, SERDES_L2_TM_ILL12_OFFSET, 0x000000FFU, 0x00000001U);
	/*##################################################################### */

	/*
	* Register : L2_TM_E_ILL1 @ 0XFD409924

	* E ILL F0 CALCODE bypass value. MPHY : G1a, PCIE : Gen 1, SATA : Gen1 , U
	* SB3 : SS
	*  PSU_SERDES_L2_TM_E_ILL1_ILL_BYPASS_E_CALCODE_F0             0x9C

	* epi cal code
	* (OFFSET, MASK, VALUE)      (0XFD409924, 0x000000FFU ,0x0000009CU)
	*/
	writeRegMsk(base, SERDES_L2_TM_E_ILL1_OFFSET, 0x000000FFU, 0x0000009CU);
	/*##################################################################### */

	/*
	* Register : L2_TM_E_ILL2 @ 0XFD409928

	* E ILL F1 CALCODE bypass value. MPHY : G1b, PCIE : Gen2, SATA: Gen2
	*  PSU_SERDES_L2_TM_E_ILL2_ILL_BYPASS_E_CALCODE_F1             0x39

	* epi cal code
	* (OFFSET, MASK, VALUE)      (0XFD409928, 0x000000FFU ,0x00000039U)
	*/
	writeRegMsk(base, SERDES_L2_TM_E_ILL2_OFFSET, 0x000000FFU, 0x00000039U);
	/*##################################################################### */

	/*
	* Register : L2_TM_ILL11 @ 0XFD40998C

	* G2A_PCIe1 PLL ctr bypass value
	*  PSU_SERDES_L2_TM_ILL11_G2A_PCIEG1_PLL_CTR_11_8_BYP_VAL      0x2

	* ill pll counter values
	* (OFFSET, MASK, VALUE)      (0XFD40998C, 0x000000F0U ,0x00000020U)
	*/
	writeRegMsk(base, SERDES_L2_TM_ILL11_OFFSET, 0x000000F0U, 0x00000020U);
	/*##################################################################### */

	/*
	* Register : L2_TM_IQ_ILL3 @ 0XFD409900

	* IQ ILL F2CALCODE bypass value. MPHY : G2a, SATA : Gen3
	*  PSU_SERDES_L2_TM_IQ_ILL3_ILL_BYPASS_IQ_CALCODE_F2           0x96

	* iqpi cal code
	* (OFFSET, MASK, VALUE)      (0XFD409900, 0x000000FFU ,0x00000096U)
	*/
	writeRegMsk(base, SERDES_L2_TM_IQ_ILL3_OFFSET,
			0x000000FFU, 0x00000096U);
	/*##################################################################### */

	/*
	* Register : L2_TM_E_ILL3 @ 0XFD40992C

	* E ILL F2CALCODE bypass value. MPHY : G2a, SATA : Gen3
	*  PSU_SERDES_L2_TM_E_ILL3_ILL_BYPASS_E_CALCODE_F2             0x64

	* epi cal code
	* (OFFSET, MASK, VALUE)      (0XFD40992C, 0x000000FFU ,0x00000064U)
	*/
	writeRegMsk(base, SERDES_L2_TM_E_ILL3_OFFSET, 0x000000FFU, 0x00000064U);
	/*##################################################################### */

	/*
	* Register : L2_TM_ILL8 @ 0XFD409980

	* ILL calibration code change wait time
	*  PSU_SERDES_L2_TM_ILL8_ILL_CAL_ITER_WAIT                     0xFF

	* ILL cal routine control
	* (OFFSET, MASK, VALUE)      (0XFD409980, 0x000000FFU ,0x000000FFU)
	*/
	writeRegMsk(base, SERDES_L2_TM_ILL8_OFFSET, 0x000000FFU, 0x000000FFU);
	/*##################################################################### */

	/*
	* Register : L2_TM_IQ_ILL8 @ 0XFD409914

	* IQ ILL polytrim bypass value
	*  PSU_SERDES_L2_TM_IQ_ILL8_ILL_BYPASS_IQ_POLYTRIM_VAL         0xF7

	* iqpi polytrim
	* (OFFSET, MASK, VALUE)      (0XFD409914, 0x000000FFU ,0x000000F7U)
	*/
	writeRegMsk(base, SERDES_L2_TM_IQ_ILL8_OFFSET,
			0x000000FFU, 0x000000F7U);
	/*##################################################################### */

	/*
	* Register : L2_TM_IQ_ILL9 @ 0XFD409918

	* bypass IQ polytrim
	*  PSU_SERDES_L2_TM_IQ_ILL9_ILL_BYPASS_IQ_POLYTIM              0x1

	* enables for lf,constant gm trim and polytirm
	* (OFFSET, MASK, VALUE)      (0XFD409918, 0x00000001U ,0x00000001U)
	*/
	writeRegMsk(base, SERDES_L2_TM_IQ_ILL9_OFFSET,
			0x00000001U, 0x00000001U);
	/*##################################################################### */

	/*
	* Register : L2_TM_E_ILL8 @ 0XFD409940

	* E ILL polytrim bypass value
	*  PSU_SERDES_L2_TM_E_ILL8_ILL_BYPASS_E_POLYTRIM_VAL           0xF7

	* epi polytrim
	* (OFFSET, MASK, VALUE)      (0XFD409940, 0x000000FFU ,0x000000F7U)
	*/
	writeRegMsk(base, SERDES_L2_TM_E_ILL8_OFFSET, 0x000000FFU, 0x000000F7U);
	/*##################################################################### */

	/*
	* Register : L2_TM_E_ILL9 @ 0XFD409944

	* bypass E polytrim
	*  PSU_SERDES_L2_TM_E_ILL9_ILL_BYPASS_E_POLYTIM                0x1

	* enables for lf,constant gm trim and polytirm
	* (OFFSET, MASK, VALUE)      (0XFD409944, 0x00000001U ,0x00000001U)
	*/
	writeRegMsk(base, SERDES_L2_TM_E_ILL9_OFFSET, 0x00000001U, 0x00000001U);
	/*##################################################################### */

	/*
	* Register : L2_TM_ILL13 @ 0XFD409994

	* ILL cal idle val refcnt
	*  PSU_SERDES_L2_TM_ILL13_ILL_CAL_IDLE_VAL_REFCNT              0x7

	* ill cal idle value count
	* (OFFSET, MASK, VALUE)      (0XFD409994, 0x00000007U ,0x00000007U)
	*/
	writeRegMsk(base, SERDES_L2_TM_ILL13_OFFSET, 0x00000007U, 0x00000007U);
	/*##################################################################### */

	/*
	* Register : L3_TM_ILL13 @ 0XFD40D994

	* ILL cal idle val refcnt
	*  PSU_SERDES_L3_TM_ILL13_ILL_CAL_IDLE_VAL_REFCNT              0x7

	* ill cal idle value count
	* (OFFSET, MASK, VALUE)      (0XFD40D994, 0x00000007U ,0x00000007U)
	*/
	writeRegMsk(base, SERDES_L3_TM_ILL13_OFFSET, 0x00000007U, 0x00000007U);
	/*##################################################################### */

	/*
	 * SYMBOL LOCK AND WAIT
	 */
	/*
	* Register : L0_TM_DIG_10 @ 0XFD40107C

	* CDR lock wait time. (1-16 us). cdr_lock_wait_time = 4'b xxxx + 4'b 0001
	*  PSU_SERDES_L0_TM_DIG_10_CDR_BIT_LOCK_TIME                   0x1

	* test control for changing cdr lock wait time
	* (OFFSET, MASK, VALUE)      (0XFD40107C, 0x0000000FU ,0x00000001U)
	*/
	writeRegMsk(base, SERDES_L0_TM_DIG_10_OFFSET, 0x0000000FU, 0x00000001U);
	/*##################################################################### */

	/*
	* Register : L1_TM_DIG_10 @ 0XFD40507C

	* CDR lock wait time. (1-16 us). cdr_lock_wait_time = 4'b xxxx + 4'b 0001
	*  PSU_SERDES_L1_TM_DIG_10_CDR_BIT_LOCK_TIME                   0x1

	* test control for changing cdr lock wait time
	* (OFFSET, MASK, VALUE)      (0XFD40507C, 0x0000000FU ,0x00000001U)
	*/
	writeRegMsk(base, SERDES_L1_TM_DIG_10_OFFSET, 0x0000000FU, 0x00000001U);
	/*##################################################################### */

	/*
	* Register : L2_TM_DIG_10 @ 0XFD40907C

	* CDR lock wait time. (1-16 us). cdr_lock_wait_time = 4'b xxxx + 4'b 0001
	*  PSU_SERDES_L2_TM_DIG_10_CDR_BIT_LOCK_TIME                   0x1

	* test control for changing cdr lock wait time
	* (OFFSET, MASK, VALUE)      (0XFD40907C, 0x0000000FU ,0x00000001U)
	*/
	writeRegMsk(base, SERDES_L2_TM_DIG_10_OFFSET, 0x0000000FU, 0x00000001U);
	/*##################################################################### */

	/*
	* Register : L3_TM_DIG_10 @ 0XFD40D07C

	* CDR lock wait time. (1-16 us). cdr_lock_wait_time = 4'b xxxx + 4'b 0001
	*  PSU_SERDES_L3_TM_DIG_10_CDR_BIT_LOCK_TIME                   0x1

	* test control for changing cdr lock wait time
	* (OFFSET, MASK, VALUE)      (0XFD40D07C, 0x0000000FU ,0x00000001U)
	*/
	writeRegMsk(base, SERDES_L3_TM_DIG_10_OFFSET, 0x0000000FU, 0x00000001U);
	/*##################################################################### */

	/*
	 * SIOU SETTINGS FOR BYPASS CONTROL,HSRX-DIG
	 */
	/*
	* Register : L0_TM_RST_DLY @ 0XFD4019A4

	* Delay apb reset by specified amount
	*  PSU_SERDES_L0_TM_RST_DLY_APB_RST_DLY                        0xFF

	* reset delay for apb reset w.r.t pso of hsrx
	* (OFFSET, MASK, VALUE)      (0XFD4019A4, 0x000000FFU ,0x000000FFU)
	*/
	writeRegMsk(base, SERDES_L0_TM_RST_DLY_OFFSET,
			0x000000FFU, 0x000000FFU);
	/*##################################################################### */

	/*
	* Register : L0_TM_ANA_BYP_15 @ 0XFD401038

	* Enable Bypass for <7> of TM_ANA_BYPS_15
	*  PSU_SERDES_L0_TM_ANA_BYP_15_FORCE_UPHY_ENABLE_LOW_LEAKAGE   0x1

	* Bypass control for pcs-pma interface. EQ supplies, main master supply an
	* d ps for samp c2c
	* (OFFSET, MASK, VALUE)      (0XFD401038, 0x00000040U ,0x00000040U)
	*/
	writeRegMsk(base, SERDES_L0_TM_ANA_BYP_15_OFFSET,
			0x00000040U, 0x00000040U);
	/*##################################################################### */

	/*
	* Register : L0_TM_ANA_BYP_12 @ 0XFD40102C

	* Enable Bypass for <7> of TM_ANA_BYPS_12
	*  PSU_SERDES_L0_TM_ANA_BYP_12_FORCE_UPHY_PSO_HSRXDIG          0x1

	* Bypass control for pcs-pma interface. Hsrx supply, hsrx des, and cdr ena
	* ble controls
	* (OFFSET, MASK, VALUE)      (0XFD40102C, 0x00000040U ,0x00000040U)
	*/
	writeRegMsk(base, SERDES_L0_TM_ANA_BYP_12_OFFSET,
			0x00000040U, 0x00000040U);
	/*##################################################################### */

	/*
	* Register : L1_TM_RST_DLY @ 0XFD4059A4

	* Delay apb reset by specified amount
	*  PSU_SERDES_L1_TM_RST_DLY_APB_RST_DLY                        0xFF

	* reset delay for apb reset w.r.t pso of hsrx
	* (OFFSET, MASK, VALUE)      (0XFD4059A4, 0x000000FFU ,0x000000FFU)
	*/
	writeRegMsk(base, SERDES_L1_TM_RST_DLY_OFFSET,
			0x000000FFU, 0x000000FFU);
	/*##################################################################### */

	/*
	* Register : L1_TM_ANA_BYP_15 @ 0XFD405038

	* Enable Bypass for <7> of TM_ANA_BYPS_15
	*  PSU_SERDES_L1_TM_ANA_BYP_15_FORCE_UPHY_ENABLE_LOW_LEAKAGE   0x1

	* Bypass control for pcs-pma interface. EQ supplies, main master supply an
	* d ps for samp c2c
	* (OFFSET, MASK, VALUE)      (0XFD405038, 0x00000040U ,0x00000040U)
	*/
	writeRegMsk(base, SERDES_L1_TM_ANA_BYP_15_OFFSET,
			0x00000040U, 0x00000040U);
	/*##################################################################### */

	/*
	* Register : L1_TM_ANA_BYP_12 @ 0XFD40502C

	* Enable Bypass for <7> of TM_ANA_BYPS_12
	*  PSU_SERDES_L1_TM_ANA_BYP_12_FORCE_UPHY_PSO_HSRXDIG          0x1

	* Bypass control for pcs-pma interface. Hsrx supply, hsrx des, and cdr ena
	* ble controls
	* (OFFSET, MASK, VALUE)      (0XFD40502C, 0x00000040U ,0x00000040U)
	*/
	writeRegMsk(base, SERDES_L1_TM_ANA_BYP_12_OFFSET,
			0x00000040U, 0x00000040U);
	/*##################################################################### */

	/*
	* Register : L2_TM_RST_DLY @ 0XFD4099A4

	* Delay apb reset by specified amount
	*  PSU_SERDES_L2_TM_RST_DLY_APB_RST_DLY                        0xFF

	* reset delay for apb reset w.r.t pso of hsrx
	* (OFFSET, MASK, VALUE)      (0XFD4099A4, 0x000000FFU ,0x000000FFU)
	*/
	writeRegMsk(base, SERDES_L2_TM_RST_DLY_OFFSET,
			0x000000FFU, 0x000000FFU);
	/*##################################################################### */

	/*
	* Register : L2_TM_ANA_BYP_15 @ 0XFD409038

	* Enable Bypass for <7> of TM_ANA_BYPS_15
	*  PSU_SERDES_L2_TM_ANA_BYP_15_FORCE_UPHY_ENABLE_LOW_LEAKAGE   0x1

	* Bypass control for pcs-pma interface. EQ supplies, main master supply an
	* d ps for samp c2c
	* (OFFSET, MASK, VALUE)      (0XFD409038, 0x00000040U ,0x00000040U)
	*/
	writeRegMsk(base, SERDES_L2_TM_ANA_BYP_15_OFFSET,
			0x00000040U, 0x00000040U);
	/*##################################################################### */

	/*
	* Register : L2_TM_ANA_BYP_12 @ 0XFD40902C

	* Enable Bypass for <7> of TM_ANA_BYPS_12
	*  PSU_SERDES_L2_TM_ANA_BYP_12_FORCE_UPHY_PSO_HSRXDIG          0x1

	* Bypass control for pcs-pma interface. Hsrx supply, hsrx des, and cdr ena
	* ble controls
	* (OFFSET, MASK, VALUE)      (0XFD40902C, 0x00000040U ,0x00000040U)
	*/
	writeRegMsk(base, SERDES_L2_TM_ANA_BYP_12_OFFSET,
			0x00000040U, 0x00000040U);
	/*##################################################################### */

	/*
	* Register : L3_TM_RST_DLY @ 0XFD40D9A4

	* Delay apb reset by specified amount
	*  PSU_SERDES_L3_TM_RST_DLY_APB_RST_DLY                        0xFF

	* reset delay for apb reset w.r.t pso of hsrx
	* (OFFSET, MASK, VALUE)      (0XFD40D9A4, 0x000000FFU ,0x000000FFU)
	*/
	writeRegMsk(base, SERDES_L3_TM_RST_DLY_OFFSET,
			0x000000FFU, 0x000000FFU);
	/*##################################################################### */

	/*
	* Register : L3_TM_ANA_BYP_15 @ 0XFD40D038

	* Enable Bypass for <7> of TM_ANA_BYPS_15
	*  PSU_SERDES_L3_TM_ANA_BYP_15_FORCE_UPHY_ENABLE_LOW_LEAKAGE   0x1

	* Bypass control for pcs-pma interface. EQ supplies, main master supply an
	* d ps for samp c2c
	* (OFFSET, MASK, VALUE)      (0XFD40D038, 0x00000040U ,0x00000040U)
	*/
	writeRegMsk(base, SERDES_L3_TM_ANA_BYP_15_OFFSET,
			0x00000040U, 0x00000040U);
	/*##################################################################### */

	/*
	* Register : L3_TM_ANA_BYP_12 @ 0XFD40D02C

	* Enable Bypass for <7> of TM_ANA_BYPS_12
	*  PSU_SERDES_L3_TM_ANA_BYP_12_FORCE_UPHY_PSO_HSRXDIG          0x1

	* Bypass control for pcs-pma interface. Hsrx supply, hsrx des, and cdr ena
	* ble controls
	* (OFFSET, MASK, VALUE)      (0XFD40D02C, 0x00000040U ,0x00000040U)
	*/
	writeRegMsk(base, SERDES_L3_TM_ANA_BYP_12_OFFSET,
			0x00000040U, 0x00000040U);
	/*##################################################################### */

	/*
	 * DISABLE FPL/FFL
	 */
	/*
	* Register : L0_TM_MISC3 @ 0XFD4019AC

	* CDR fast phase lock control
	*  PSU_SERDES_L0_TM_MISC3_CDR_EN_FPL                           0x0

	* CDR fast frequency lock control
	*  PSU_SERDES_L0_TM_MISC3_CDR_EN_FFL                           0x0

	* debug bus selection bit, cdr fast phase and freq controls
	* (OFFSET, MASK, VALUE)      (0XFD4019AC, 0x00000003U ,0x00000000U)
	*/
	writeRegMsk(base, SERDES_L0_TM_MISC3_OFFSET, 0x00000003U, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : L1_TM_MISC3 @ 0XFD4059AC

	* CDR fast phase lock control
	*  PSU_SERDES_L1_TM_MISC3_CDR_EN_FPL                           0x0

	* CDR fast frequency lock control
	*  PSU_SERDES_L1_TM_MISC3_CDR_EN_FFL                           0x0

	* debug bus selection bit, cdr fast phase and freq controls
	* (OFFSET, MASK, VALUE)      (0XFD4059AC, 0x00000003U ,0x00000000U)
	*/
	writeRegMsk(base, SERDES_L1_TM_MISC3_OFFSET, 0x00000003U, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : L2_TM_MISC3 @ 0XFD4099AC

	* CDR fast phase lock control
	*  PSU_SERDES_L2_TM_MISC3_CDR_EN_FPL                           0x0

	* CDR fast frequency lock control
	*  PSU_SERDES_L2_TM_MISC3_CDR_EN_FFL                           0x0

	* debug bus selection bit, cdr fast phase and freq controls
	* (OFFSET, MASK, VALUE)      (0XFD4099AC, 0x00000003U ,0x00000000U)
	*/
	writeRegMsk(base, SERDES_L2_TM_MISC3_OFFSET, 0x00000003U, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : L3_TM_MISC3 @ 0XFD40D9AC

	* CDR fast phase lock control
	*  PSU_SERDES_L3_TM_MISC3_CDR_EN_FPL                           0x0

	* CDR fast frequency lock control
	*  PSU_SERDES_L3_TM_MISC3_CDR_EN_FFL                           0x0

	* debug bus selection bit, cdr fast phase and freq controls
	* (OFFSET, MASK, VALUE)      (0XFD40D9AC, 0x00000003U ,0x00000000U)
	*/
	writeRegMsk(base, SERDES_L3_TM_MISC3_OFFSET, 0x00000003U, 0x00000000U);
	/*##################################################################### */

	/*
	 * DISABLE DYNAMIC OFFSET CALIBRATION
	 */
	/*
	* Register : L0_TM_EQ11 @ 0XFD401978

	* Force EQ offset correction algo off if not forced on
	*  PSU_SERDES_L0_TM_EQ11_FORCE_EQ_OFFS_OFF                     0x1

	* eq dynamic offset correction
	* (OFFSET, MASK, VALUE)      (0XFD401978, 0x00000010U ,0x00000010U)
	*/
	writeRegMsk(base, SERDES_L0_TM_EQ11_OFFSET, 0x00000010U, 0x00000010U);
	/*##################################################################### */

	/*
	* Register : L1_TM_EQ11 @ 0XFD405978

	* Force EQ offset correction algo off if not forced on
	*  PSU_SERDES_L1_TM_EQ11_FORCE_EQ_OFFS_OFF                     0x1

	* eq dynamic offset correction
	* (OFFSET, MASK, VALUE)      (0XFD405978, 0x00000010U ,0x00000010U)
	*/
	writeRegMsk(base, SERDES_L1_TM_EQ11_OFFSET, 0x00000010U, 0x00000010U);
	/*##################################################################### */

	/*
	* Register : L2_TM_EQ11 @ 0XFD409978

	* Force EQ offset correction algo off if not forced on
	*  PSU_SERDES_L2_TM_EQ11_FORCE_EQ_OFFS_OFF                     0x1

	* eq dynamic offset correction
	* (OFFSET, MASK, VALUE)      (0XFD409978, 0x00000010U ,0x00000010U)
	*/
	writeRegMsk(base, SERDES_L2_TM_EQ11_OFFSET, 0x00000010U, 0x00000010U);
	/*##################################################################### */

	/*
	* Register : L3_TM_EQ11 @ 0XFD40D978

	* Force EQ offset correction algo off if not forced on
	*  PSU_SERDES_L3_TM_EQ11_FORCE_EQ_OFFS_OFF                     0x1

	* eq dynamic offset correction
	* (OFFSET, MASK, VALUE)      (0XFD40D978, 0x00000010U ,0x00000010U)
	*/
	writeRegMsk(base, SERDES_L3_TM_EQ11_OFFSET, 0x00000010U, 0x00000010U);
	/*##################################################################### */

	/*
	 * SERDES ILL CALIB
	 */
	serdes_illcalib(base, 4, 0, 2, 3, 3, 0, 1, 1);

	/*##################################################################### */

	/*
	 * DISABLE ECO FOR PCIE
	 */
	/*
	* Register : eco_0 @ 0XFD3D001C

	* For future use
	*  PSU_SIOU_ECO_0_FIELD                                        0x1

	* ECO Register for future use
	* (OFFSET, MASK, VALUE)      (0XFD3D001C, 0xFFFFFFFFU ,0x00000001U)
	*/
	uint32_t *siou = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, 0xfd3d0000);
	if (MAP_FAILED == base) {
		fprintf(stderr, "pcie-phy: fail to map SIOU registers memory\n");
	}
	writeRegMsk(siou, 0x001C, 0xFFFFFFFFU, 0x00000001U);
	// writeRegMsk(base, SIOU_ECO_0_OFFSET, 0xFFFFFFFFU, 0x00000001U);
	munmap((void *)siou, 0x1000);
	/*##################################################################### */

	/*
	 * GT LANE SETTINGS
	 */
	/*
	* Register : ICM_CFG0 @ 0XFD410010

	* Controls UPHY Lane 0 protocol configuration. 0 - PowerDown, 1 - PCIe .0,
	*  2 - Sata0, 3 - USB0, 4 - DP.1, 5 - SGMII0, 6 - Unused, 7 - Unused
	*  PSU_SERDES_ICM_CFG0_L0_ICM_CFG                              1

	* Controls UPHY Lane 1 protocol configuration. 0 - PowerDown, 1 - PCIe.1,
	* 2 - Sata1, 3 - USB0, 4 - DP.0, 5 - SGMII1, 6 - Unused, 7 - Unused
	*  PSU_SERDES_ICM_CFG0_L1_ICM_CFG                              3

	* ICM Configuration Register 0
	* (OFFSET, MASK, VALUE)      (0XFD410010, 0x00000077U ,0x00000031U)
	*/
	writeRegMsk(base, SERDES_ICM_CFG0_OFFSET, 0x00000077U, 0x00000031U);
	/*##################################################################### */

	/*
	* Register : ICM_CFG1 @ 0XFD410014

	* Controls UPHY Lane 2 protocol configuration. 0 - PowerDown, 1 - PCIe.1,
	* 2 - Sata0, 3 - USB0, 4 - DP.1, 5 - SGMII2, 6 - Unused, 7 - Unused
	*  PSU_SERDES_ICM_CFG1_L2_ICM_CFG                              2

	* Controls UPHY Lane 3 protocol configuration. 0 - PowerDown, 1 - PCIe.3,
	* 2 - Sata1, 3 - USB1, 4 - DP.0, 5 - SGMII3, 6 - Unused, 7 - Unused
	*  PSU_SERDES_ICM_CFG1_L3_ICM_CFG                              4

	* ICM Configuration Register 1
	* (OFFSET, MASK, VALUE)      (0XFD410014, 0x00000077U ,0x00000042U)
	*/
	writeRegMsk(base, SERDES_ICM_CFG1_OFFSET, 0x00000077U, 0x00000042U);
	/*##################################################################### */

	/*
	 * CHECKING PLL LOCK
	 */
	/*
	 * ENABLE SERIAL DATA MUX DEEMPH
	 */
	/*
	* Register : L3_TXPMD_TM_45 @ 0XFD40CCB4

	* Enable/disable DP post2 path
	*  PSU_SERDES_L3_TXPMD_TM_45_DP_TM_TX_DP_ENABLE_POST2_PATH     0x1

	* Override enable/disable of DP post2 path
	*  PSU_SERDES_L3_TXPMD_TM_45_DP_TM_TX_OVRD_DP_ENABLE_POST2_PATH 0x1

	* Override enable/disable of DP post1 path
	*  PSU_SERDES_L3_TXPMD_TM_45_DP_TM_TX_OVRD_DP_ENABLE_POST1_PATH 0x1

	* Enable/disable DP main path
	*  PSU_SERDES_L3_TXPMD_TM_45_DP_TM_TX_DP_ENABLE_MAIN_PATH      0x1

	* Override enable/disable of DP main path
	*  PSU_SERDES_L3_TXPMD_TM_45_DP_TM_TX_OVRD_DP_ENABLE_MAIN_PATH 0x1

	* Post or pre or main DP path selection
	* (OFFSET, MASK, VALUE)      (0XFD40CCB4, 0x00000037U ,0x00000037U)
	*/
	writeRegMsk(base, SERDES_L3_TXPMD_TM_45_OFFSET,
			0x00000037U, 0x00000037U);
	/*##################################################################### */

	/*
	* Register : L2_TX_ANA_TM_118 @ 0XFD4081D8

	* Test register force for enabling/disablign TX deemphasis bits <17:0>
	*  PSU_SERDES_L2_TX_ANA_TM_118_FORCE_TX_DEEMPH_17_0            0x1

	* Enable Override of TX deemphasis
	* (OFFSET, MASK, VALUE)      (0XFD4081D8, 0x00000001U ,0x00000001U)
	*/
	writeRegMsk(base, SERDES_L2_TX_ANA_TM_118_OFFSET,
			0x00000001U, 0x00000001U);
	/*##################################################################### */

	/*
	* Register : L3_TX_ANA_TM_118 @ 0XFD40C1D8

	* Test register force for enabling/disablign TX deemphasis bits <17:0>
	*  PSU_SERDES_L3_TX_ANA_TM_118_FORCE_TX_DEEMPH_17_0            0x1

	* Enable Override of TX deemphasis
	* (OFFSET, MASK, VALUE)      (0XFD40C1D8, 0x00000001U ,0x00000001U)
	*/
	writeRegMsk(base, SERDES_L3_TX_ANA_TM_118_OFFSET,
			0x00000001U, 0x00000001U);
	/*##################################################################### */

	/*
	 * CDR AND RX EQUALIZATION SETTINGS
	 */
	/*
	* Register : L2_TM_CDR5 @ 0XFD409C14

	* FPHL FSM accumulate cycles
	*  PSU_SERDES_L2_TM_CDR5_FPHL_FSM_ACC_CYCLES                   0x7

	* FFL Phase0 int gain aka 2ol SD update rate
	*  PSU_SERDES_L2_TM_CDR5_FFL_PH0_INT_GAIN                      0x6

	* Fast phase lock controls -- FSM accumulator cycle control and phase 0 in
	* t gain control.
	* (OFFSET, MASK, VALUE)      (0XFD409C14, 0x000000FFU ,0x000000E6U)
	*/
	writeRegMsk(base, SERDES_L2_TM_CDR5_OFFSET, 0x000000FFU, 0x000000E6U);
	/*##################################################################### */

	/*
	* Register : L2_TM_CDR16 @ 0XFD409C40

	* FFL Phase0 prop gain aka 1ol SD update rate
	*  PSU_SERDES_L2_TM_CDR16_FFL_PH0_PROP_GAIN                    0xC

	* Fast phase lock controls -- phase 0 prop gain
	* (OFFSET, MASK, VALUE)      (0XFD409C40, 0x0000001FU ,0x0000000CU)
	*/
	writeRegMsk(base, SERDES_L2_TM_CDR16_OFFSET, 0x0000001FU, 0x0000000CU);
	/*##################################################################### */

	/*
	* Register : L2_TM_EQ0 @ 0XFD40994C

	* EQ stg 2 controls BYPASSED
	*  PSU_SERDES_L2_TM_EQ0_EQ_STG2_CTRL_BYP                       1

	* eq stg1 and stg2 controls
	* (OFFSET, MASK, VALUE)      (0XFD40994C, 0x00000020U ,0x00000020U)
	*/
	writeRegMsk(base, SERDES_L2_TM_EQ0_OFFSET, 0x00000020U, 0x00000020U);
	/*##################################################################### */

	/*
	* Register : L2_TM_EQ1 @ 0XFD409950

	* EQ STG2 RL PROG
	*  PSU_SERDES_L2_TM_EQ1_EQ_STG2_RL_PROG                        0x2

	* EQ stg 2 preamp mode val
	*  PSU_SERDES_L2_TM_EQ1_EQ_STG2_PREAMP_MODE_VAL                0x1

	* eq stg1 and stg2 controls
	* (OFFSET, MASK, VALUE)      (0XFD409950, 0x00000007U ,0x00000006U)
	*/
	writeRegMsk(base, SERDES_L2_TM_EQ1_OFFSET, 0x00000007U, 0x00000006U);
	/*##################################################################### */

	/*
	 * GEM SERDES SETTINGS
	 */
	/*
	 * ENABLE PRE EMPHAIS AND VOLTAGE SWING
	 */
	/*
	* Register : L3_TXPMD_TM_48 @ 0XFD40CCC0

	* Margining factor value
	*  PSU_SERDES_L3_TXPMD_TM_48_TM_RESULTANT_MARGINING_FACTOR     0

	* Margining factor
	* (OFFSET, MASK, VALUE)      (0XFD40CCC0, 0x0000001FU ,0x00000000U)
	*/
	writeRegMsk(base, SERDES_L3_TXPMD_TM_48_OFFSET,
			0x0000001FU, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : L2_TX_ANA_TM_18 @ 0XFD408048

	* pipe_TX_Deemph. 0: -6dB de-emphasis, 1: -3.5dB de-emphasis, 2 : No de-em
	* phasis, Others: reserved
	*  PSU_SERDES_L2_TX_ANA_TM_18_PIPE_TX_DEEMPH_7_0               0x1

	* Override for PIPE TX de-emphasis
	* (OFFSET, MASK, VALUE)      (0XFD408048, 0x000000FFU ,0x00000001U)
	*/
	writeRegMsk(base, SERDES_L2_TX_ANA_TM_18_OFFSET,
			0x000000FFU, 0x00000001U);
	/*##################################################################### */

	/*
	* Register : L3_TX_ANA_TM_18 @ 0XFD40C048

	* pipe_TX_Deemph. 0: -6dB de-emphasis, 1: -3.5dB de-emphasis, 2 : No de-em
	* phasis, Others: reserved
	*  PSU_SERDES_L3_TX_ANA_TM_18_PIPE_TX_DEEMPH_7_0               0

	* Override for PIPE TX de-emphasis
	* (OFFSET, MASK, VALUE)      (0XFD40C048, 0x000000FFU ,0x00000000U)
	*/
	writeRegMsk(base, SERDES_L3_TX_ANA_TM_18_OFFSET,
			0x000000FFU, 0x00000000U);
	/*##################################################################### */


	return 1;
}


static int pcie_phyDeassertPcieResets(void)
{
	int ret = 0;

	platformctl_t ctl3 = {
		.action = pctl_set,
		.type = pctl_devreset,
		.devreset.dev = pctl_devreset_fpd_pcie_bridge,
		.devreset.state = 0,
	};
	ret = platformctl(&ctl3);
	if (ret != 0) {
		fprintf(stderr, "pcie-phy: fail to deassert PCIE BRIDGE reset\n");
		return ret;
	}

	platformctl_t ctl4 = {
		.action = pctl_set,
		.type = pctl_devreset,
		.devreset.dev = pctl_devreset_fpd_pcie_cfg,
		.devreset.state = 0,
	};
	ret = platformctl(&ctl4);
	if (ret != 0) {
		fprintf(stderr, "pcie-phy: fail to deassert PCIE CFG reset\n");
		return ret;
	}

	return ret;
}


void pcie_phyConfigPcie(uint32_t *pcireg, uint32_t *serdes)
{
	/*
	 * UPDATING TWO PCIE REGISTERS DEFAULT VALUES, AS THESE REGISTERS HAVE INCO
	 * RRECT RESET VALUES IN SILICON.
	 */
	/*
	* Register : ATTR_25 @ 0XFD480064

	* If TRUE Completion Timeout Disable is supported. This is required to be
	* TRUE for Endpoint and either setting allowed for Root ports. Drives Devi
	* ce Capability 2 [4]; EP=0x0001; RP=0x0001
	*  PSU_PCIE_ATTRIB_ATTR_25_ATTR_CPL_TIMEOUT_DISABLE_SUPPORTED  0X1

	* ATTR_25
	* (OFFSET, MASK, VALUE)      (0XFD480064, 0x00000200U ,0x00000200U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_25_OFFSET, 0x00000200U, 0x00000200U);
	/*##################################################################### */

	/*
	 * PCIE SETTINGS
	 */
	/*
	* Register : ATTR_7 @ 0XFD48001C

	* Specifies mask/settings for Base Address Register (BAR) 0. If BAR is not
	*  to be implemented, set to 32'h00000000. Bits are defined as follows: Me
	* mory Space BAR [0] = Mem Space Indicator (set to 0) [2:1] = Type field (
	* 10 for 64-bit, 00 for 32-bit) [3] = Prefetchable (0 or 1) [31:4] = Mask
	* for writable bits of BAR; if 32-bit BAR, set uppermost 31:n bits to 1, w
	* here 2^n=memory aperture size in bytes. If 64-bit BAR, set uppermost 63:
	* n bits of \'7bBAR1,BAR0\'7d to 1. IO Space BAR 0] = IO Space Indicator (
	* set to 1) [1] = Reserved (set to 0) [31:2] = Mask for writable bits of B
	* AR; set uppermost 31:n bits to 1, where 2^n=i/o aperture size in bytes.;
	*  EP=0x0004; RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_7_ATTR_BAR0                            0x0

	* ATTR_7
	* (OFFSET, MASK, VALUE)      (0XFD48001C, 0x0000FFFFU ,0x00000000U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_7_OFFSET, 0x0000FFFFU, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : ATTR_8 @ 0XFD480020

	* Specifies mask/settings for Base Address Register (BAR) 0. If BAR is not
	*  to be implemented, set to 32'h00000000. Bits are defined as follows: Me
	* mory Space BAR [0] = Mem Space Indicator (set to 0) [2:1] = Type field (
	* 10 for 64-bit, 00 for 32-bit) [3] = Prefetchable (0 or 1) [31:4] = Mask
	* for writable bits of BAR; if 32-bit BAR, set uppermost 31:n bits to 1, w
	* here 2^n=memory aperture size in bytes. If 64-bit BAR, set uppermost 63:
	* n bits of \'7bBAR1,BAR0\'7d to 1. IO Space BAR 0] = IO Space Indicator (
	* set to 1) [1] = Reserved (set to 0) [31:2] = Mask for writable bits of B
	* AR; set uppermost 31:n bits to 1, where 2^n=i/o aperture size in bytes.;
	*  EP=0xFFF0; RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_8_ATTR_BAR0                            0x0

	* ATTR_8
	* (OFFSET, MASK, VALUE)      (0XFD480020, 0x0000FFFFU ,0x00000000U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_8_OFFSET, 0x0000FFFFU, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : ATTR_9 @ 0XFD480024

	* Specifies mask/settings for Base Address Register (BAR) 1 if BAR0 is a 3
	* 2-bit BAR, or the upper bits of \'7bBAR1,BAR0\'7d if BAR0 is a 64-bit BA
	* R. If BAR is not to be implemented, set to 32'h00000000. See BAR0 descri
	* ption if this functions as the upper bits of a 64-bit BAR. Bits are defi
	* ned as follows: Memory Space BAR (not upper bits of BAR0) [0] = Mem Spac
	* e Indicator (set to 0) [2:1] = Type field (10 for 64-bit, 00 for 32-bit)
	*  [3] = Prefetchable (0 or 1) [31:4] = Mask for writable bits of BAR; if
	* 32-bit BAR, set uppermost 31:n bits to 1, where 2^n=memory aperture size
	*  in bytes. If 64-bit BAR, set uppermost 63:n bits of \'7bBAR2,BAR1\'7d t
	* o 1. IO Space BAR 0] = IO Space Indicator (set to 1) [1] = Reserved (set
	*  to 0) [31:2] = Mask for writable bits of BAR; set uppermost 31:n bits t
	* o 1, where 2^n=i/o aperture size in bytes.; EP=0xFFFF; RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_9_ATTR_BAR1                            0x0

	* ATTR_9
	* (OFFSET, MASK, VALUE)      (0XFD480024, 0x0000FFFFU ,0x00000000U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_9_OFFSET, 0x0000FFFFU, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : ATTR_10 @ 0XFD480028

	* Specifies mask/settings for Base Address Register (BAR) 1 if BAR0 is a 3
	* 2-bit BAR, or the upper bits of \'7bBAR1,BAR0\'7d if BAR0 is a 64-bit BA
	* R. If BAR is not to be implemented, set to 32'h00000000. See BAR0 descri
	* ption if this functions as the upper bits of a 64-bit BAR. Bits are defi
	* ned as follows: Memory Space BAR (not upper bits of BAR0) [0] = Mem Spac
	* e Indicator (set to 0) [2:1] = Type field (10 for 64-bit, 00 for 32-bit)
	*  [3] = Prefetchable (0 or 1) [31:4] = Mask for writable bits of BAR; if
	* 32-bit BAR, set uppermost 31:n bits to 1, where 2^n=memory aperture size
	*  in bytes. If 64-bit BAR, set uppermost 63:n bits of \'7bBAR2,BAR1\'7d t
	* o 1. IO Space BAR 0] = IO Space Indicator (set to 1) [1] = Reserved (set
	*  to 0) [31:2] = Mask for writable bits of BAR; set uppermost 31:n bits t
	* o 1, where 2^n=i/o aperture size in bytes.; EP=0xFFFF; RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_10_ATTR_BAR1                           0x0

	* ATTR_10
	* (OFFSET, MASK, VALUE)      (0XFD480028, 0x0000FFFFU ,0x00000000U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_10_OFFSET, 0x0000FFFFU, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : ATTR_11 @ 0XFD48002C

	* For an endpoint, specifies mask/settings for Base Address Register (BAR)
	*  2 if BAR1 is a 32-bit BAR, or the upper bits of \'7bBAR2,BAR1\'7d if BA
	* R1 is the lower part of a 64-bit BAR. If BAR is not to be implemented, s
	* et to 32'h00000000. See BAR1 description if this functions as the upper
	* bits of a 64-bit BAR. For a switch or root: This must be set to 00FF_FFF
	* F. For an endpoint, bits are defined as follows: Memory Space BAR (not u
	* pper bits of BAR1) [0] = Mem Space Indicator (set to 0) [2:1] = Type fie
	* ld (10 for 64-bit, 00 for 32-bit) [3] = Prefetchable (0 or 1) [31:4] = M
	* ask for writable bits of BAR; if 32-bit BAR, set uppermost 31:n bits to
	* 1, where 2^n=memory aperture size in bytes. If 64-bit BAR, set uppermost
	*  63:n bits of \'7bBAR3,BAR2\'7d to 1. IO Space BAR 0] = IO Space Indicat
	* or (set to 1) [1] = Reserved (set to 0) [31:2] = Mask for writable bits
	* of BAR; set uppermost 31:n bits to 1, where 2^n=i/o aperture size in byt
	* es.; EP=0x0004; RP=0xFFFF
	*  PSU_PCIE_ATTRIB_ATTR_11_ATTR_BAR2                           0xFFFF

	* ATTR_11
	* (OFFSET, MASK, VALUE)      (0XFD48002C, 0x0000FFFFU ,0x0000FFFFU)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_11_OFFSET, 0x0000FFFFU, 0x0000FFFFU);
	/*##################################################################### */

	/*
	* Register : ATTR_12 @ 0XFD480030

	* For an endpoint, specifies mask/settings for Base Address Register (BAR)
	*  2 if BAR1 is a 32-bit BAR, or the upper bits of \'7bBAR2,BAR1\'7d if BA
	* R1 is the lower part of a 64-bit BAR. If BAR is not to be implemented, s
	* et to 32'h00000000. See BAR1 description if this functions as the upper
	* bits of a 64-bit BAR. For a switch or root: This must be set to 00FF_FFF
	* F. For an endpoint, bits are defined as follows: Memory Space BAR (not u
	* pper bits of BAR1) [0] = Mem Space Indicator (set to 0) [2:1] = Type fie
	* ld (10 for 64-bit, 00 for 32-bit) [3] = Prefetchable (0 or 1) [31:4] = M
	* ask for writable bits of BAR; if 32-bit BAR, set uppermost 31:n bits to
	* 1, where 2^n=memory aperture size in bytes. If 64-bit BAR, set uppermost
	*  63:n bits of \'7bBAR3,BAR2\'7d to 1. IO Space BAR 0] = IO Space Indicat
	* or (set to 1) [1] = Reserved (set to 0) [31:2] = Mask for writable bits
	* of BAR; set uppermost 31:n bits to 1, where 2^n=i/o aperture size in byt
	* es.; EP=0xFFF0; RP=0x00FF
	*  PSU_PCIE_ATTRIB_ATTR_12_ATTR_BAR2                           0xFF

	* ATTR_12
	* (OFFSET, MASK, VALUE)      (0XFD480030, 0x0000FFFFU ,0x000000FFU)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_12_OFFSET, 0x0000FFFFU, 0x000000FFU);
	/*##################################################################### */

	/*
	* Register : ATTR_13 @ 0XFD480034

	* For an endpoint, specifies mask/settings for Base Address Register (BAR)
	*  3 if BAR2 is a 32-bit BAR, or the upper bits of \'7bBAR3,BAR2\'7d if BA
	* R2 is the lower part of a 64-bit BAR. If BAR is not to be implemented, s
	* et to 32'h00000000. See BAR2 description if this functions as the upper
	* bits of a 64-bit BAR. For a switch or root, this must be set to: FFFF_00
	* 00 = IO Limit/Base Registers not implemented FFFF_F0F0 = IO Limit/Base R
	* egisters use 16-bit decode FFFF_F1F1 = IO Limit/Base Registers use 32-bi
	* t decode For an endpoint, bits are defined as follows: Memory Space BAR
	* (not upper bits of BAR2) [0] = Mem Space Indicator (set to 0) [2:1] = Ty
	* pe field (10 for 64-bit, 00 for 32-bit) [3] = Prefetchable (0 or 1) [31:
	* 4] = Mask for writable bits of BAR; if 32-bit BAR, set uppermost 31:n bi
	* ts to 1, where 2^n=memory aperture size in bytes. If 64-bit BAR, set upp
	* ermost 63:n bits of \'7bBAR4,BAR3\'7d to 1. IO Space BAR 0] = IO Space I
	* ndicator (set to 1) [1] = Reserved (set to 0) [31:2] = Mask for writable
	*  bits of BAR; set uppermost 31:n bits to 1, where 2^n=i/o aperture size
	* in bytes.; EP=0xFFFF; RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_13_ATTR_BAR3                           0x0

	* ATTR_13
	* (OFFSET, MASK, VALUE)      (0XFD480034, 0x0000FFFFU ,0x00000000U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_13_OFFSET, 0x0000FFFFU, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : ATTR_14 @ 0XFD480038

	* For an endpoint, specifies mask/settings for Base Address Register (BAR)
	*  3 if BAR2 is a 32-bit BAR, or the upper bits of \'7bBAR3,BAR2\'7d if BA
	* R2 is the lower part of a 64-bit BAR. If BAR is not to be implemented, s
	* et to 32'h00000000. See BAR2 description if this functions as the upper
	* bits of a 64-bit BAR. For a switch or root, this must be set to: FFFF_00
	* 00 = IO Limit/Base Registers not implemented FFFF_F0F0 = IO Limit/Base R
	* egisters use 16-bit decode FFFF_F1F1 = IO Limit/Base Registers use 32-bi
	* t decode For an endpoint, bits are defined as follows: Memory Space BAR
	* (not upper bits of BAR2) [0] = Mem Space Indicator (set to 0) [2:1] = Ty
	* pe field (10 for 64-bit, 00 for 32-bit) [3] = Prefetchable (0 or 1) [31:
	* 4] = Mask for writable bits of BAR; if 32-bit BAR, set uppermost 31:n bi
	* ts to 1, where 2^n=memory aperture size in bytes. If 64-bit BAR, set upp
	* ermost 63:n bits of \'7bBAR4,BAR3\'7d to 1. IO Space BAR 0] = IO Space I
	* ndicator (set to 1) [1] = Reserved (set to 0) [31:2] = Mask for writable
	*  bits of BAR; set uppermost 31:n bits to 1, where 2^n=i/o aperture size
	* in bytes.; EP=0xFFFF; RP=0xFFFF
	*  PSU_PCIE_ATTRIB_ATTR_14_ATTR_BAR3                           0xFFFF

	* ATTR_14
	* (OFFSET, MASK, VALUE)      (0XFD480038, 0x0000FFFFU ,0x0000FFFFU)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_14_OFFSET, 0x0000FFFFU, 0x0000FFFFU);
	/*##################################################################### */

	/*
	* Register : ATTR_15 @ 0XFD48003C

	* For an endpoint, specifies mask/settings for Base Address Register (BAR)
	*  4 if BAR3 is a 32-bit BAR, or the upper bits of \'7bBAR4,BAR3\'7d if BA
	* R3 is the lower part of a 64-bit BAR. If BAR is not to be implemented, s
	* et to 32'h00000000. See BAR3 description if this functions as the upper
	* bits of a 64-bit BAR. For a switch or root: This must be set to FFF0_FFF
	* 0. For an endpoint, bits are defined as follows: Memory Space BAR (not u
	* pper bits of BAR3) [0] = Mem Space Indicator (set to 0) [2:1] = Type fie
	* ld (10 for 64-bit, 00 for 32-bit) [3] = Prefetchable (0 or 1) [31:4] = M
	* ask for writable bits of BAR; if 32-bit BAR, set uppermost 31:n bits to
	* 1, where 2^n=memory aperture size in bytes. If 64-bit BAR, set uppermost
	*  63:n bits of \'7bBAR5,BAR4\'7d to 1. IO Space BAR 0] = IO Space Indicat
	* or (set to 1) [1] = Reserved (set to 0) [31:2] = Mask for writable bits
	* of BAR; set uppermost 31:n bits to 1, where 2^n=i/o aperture size in byt
	* es.; EP=0x0004; RP=0xFFF0
	*  PSU_PCIE_ATTRIB_ATTR_15_ATTR_BAR4                           0xFFF0

	* ATTR_15
	* (OFFSET, MASK, VALUE)      (0XFD48003C, 0x0000FFFFU ,0x0000FFF0U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_15_OFFSET, 0x0000FFFFU, 0x0000FFF0U);
	/*##################################################################### */

	/*
	* Register : ATTR_16 @ 0XFD480040

	* For an endpoint, specifies mask/settings for Base Address Register (BAR)
	*  4 if BAR3 is a 32-bit BAR, or the upper bits of \'7bBAR4,BAR3\'7d if BA
	* R3 is the lower part of a 64-bit BAR. If BAR is not to be implemented, s
	* et to 32'h00000000. See BAR3 description if this functions as the upper
	* bits of a 64-bit BAR. For a switch or root: This must be set to FFF0_FFF
	* 0. For an endpoint, bits are defined as follows: Memory Space BAR (not u
	* pper bits of BAR3) [0] = Mem Space Indicator (set to 0) [2:1] = Type fie
	* ld (10 for 64-bit, 00 for 32-bit) [3] = Prefetchable (0 or 1) [31:4] = M
	* ask for writable bits of BAR; if 32-bit BAR, set uppermost 31:n bits to
	* 1, where 2^n=memory aperture size in bytes. If 64-bit BAR, set uppermost
	*  63:n bits of \'7bBAR5,BAR4\'7d to 1. IO Space BAR 0] = IO Space Indicat
	* or (set to 1) [1] = Reserved (set to 0) [31:2] = Mask for writable bits
	* of BAR; set uppermost 31:n bits to 1, where 2^n=i/o aperture size in byt
	* es.; EP=0xFFF0; RP=0xFFF0
	*  PSU_PCIE_ATTRIB_ATTR_16_ATTR_BAR4                           0xFFF0

	* ATTR_16
	* (OFFSET, MASK, VALUE)      (0XFD480040, 0x0000FFFFU ,0x0000FFF0U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_16_OFFSET, 0x0000FFFFU, 0x0000FFF0U);
	/*##################################################################### */

	/*
	* Register : ATTR_17 @ 0XFD480044

	* For an endpoint, specifies mask/settings for Base Address Register (BAR)
	*  5 if BAR4 is a 32-bit BAR, or the upper bits of \'7bBAR5,BAR4\'7d if BA
	* R4 is the lower part of a 64-bit BAR. If BAR is not to be implemented, s
	* et to 32'h00000000. See BAR4 description if this functions as the upper
	* bits of a 64-bit BAR. For a switch or root, this must be set to: 0000_00
	* 00 = Prefetchable Memory Limit/Base Registers not implemented FFF0_FFF0
	* = 32-bit Prefetchable Memory Limit/Base implemented FFF1_FFF1 = 64-bit P
	* refetchable Memory Limit/Base implemented For an endpoint, bits are defi
	* ned as follows: Memory Space BAR (not upper bits of BAR4) [0] = Mem Spac
	* e Indicator (set to 0) [2:1] = Type field (00 for 32-bit; BAR5 cannot be
	*  lower part of a 64-bit BAR) [3] = Prefetchable (0 or 1) [31:4] = Mask f
	* or writable bits of BAR; set uppermost 31:n bits to 1, where 2^n=memory
	* aperture size in bytes. IO Space BAR 0] = IO Space Indicator (set to 1)
	* [1] = Reserved (set to 0) [31:2] = Mask for writable bits of BAR; set up
	* permost 31:n bits to 1, where 2^n=i/o aperture size in bytes.; EP=0xFFFF
	* ; RP=0xFFF1
	*  PSU_PCIE_ATTRIB_ATTR_17_ATTR_BAR5                           0xFFF1

	* ATTR_17
	* (OFFSET, MASK, VALUE)      (0XFD480044, 0x0000FFFFU ,0x0000FFF1U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_17_OFFSET, 0x0000FFFFU, 0x0000FFF1U);
	/*##################################################################### */

	/*
	* Register : ATTR_18 @ 0XFD480048

	* For an endpoint, specifies mask/settings for Base Address Register (BAR)
	*  5 if BAR4 is a 32-bit BAR, or the upper bits of \'7bBAR5,BAR4\'7d if BA
	* R4 is the lower part of a 64-bit BAR. If BAR is not to be implemented, s
	* et to 32'h00000000. See BAR4 description if this functions as the upper
	* bits of a 64-bit BAR. For a switch or root, this must be set to: 0000_00
	* 00 = Prefetchable Memory Limit/Base Registers not implemented FFF0_FFF0
	* = 32-bit Prefetchable Memory Limit/Base implemented FFF1_FFF1 = 64-bit P
	* refetchable Memory Limit/Base implemented For an endpoint, bits are defi
	* ned as follows: Memory Space BAR (not upper bits of BAR4) [0] = Mem Spac
	* e Indicator (set to 0) [2:1] = Type field (00 for 32-bit; BAR5 cannot be
	*  lower part of a 64-bit BAR) [3] = Prefetchable (0 or 1) [31:4] = Mask f
	* or writable bits of BAR; set uppermost 31:n bits to 1, where 2^n=memory
	* aperture size in bytes. IO Space BAR 0] = IO Space Indicator (set to 1)
	* [1] = Reserved (set to 0) [31:2] = Mask for writable bits of BAR; set up
	* permost 31:n bits to 1, where 2^n=i/o aperture size in bytes.; EP=0xFFFF
	* ; RP=0xFFF1
	*  PSU_PCIE_ATTRIB_ATTR_18_ATTR_BAR5                           0xFFF1

	* ATTR_18
	* (OFFSET, MASK, VALUE)      (0XFD480048, 0x0000FFFFU ,0x0000FFF1U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_18_OFFSET, 0x0000FFFFU, 0x0000FFF1U);
	/*##################################################################### */

	/*
	* Register : ATTR_27 @ 0XFD48006C

	* Specifies maximum payload supported. Valid settings are: 0- 128 bytes, 1
	* - 256 bytes, 2- 512 bytes, 3- 1024 bytes. Transferred to the Device Capa
	* bilities register. The values: 4-2048 bytes, 5- 4096 bytes are not suppo
	* rted; EP=0x0001; RP=0x0001
	*  PSU_PCIE_ATTRIB_ATTR_27_ATTR_DEV_CAP_MAX_PAYLOAD_SUPPORTED  1

	* Endpoint L1 Acceptable Latency. Records the latency that the endpoint ca
	* n withstand on transitions from L1 state to L0 (if L1 state supported).
	* Valid settings are: 0h less than 1us, 1h 1 to 2us, 2h 2 to 4us, 3h 4 to
	* 8us, 4h 8 to 16us, 5h 16 to 32us, 6h 32 to 64us, 7h more than 64us. For
	* Endpoints only. Must be 0h for other devices.; EP=0x0007; RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_27_ATTR_DEV_CAP_ENDPOINT_L1_LATENCY    0x0

	* ATTR_27
	* (OFFSET, MASK, VALUE)      (0XFD48006C, 0x00000738U ,0x00000100U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_27_OFFSET, 0x00000738U, 0x00000100U);
	/*##################################################################### */

	/*
	* Register : ATTR_50 @ 0XFD4800C8

	* Identifies the type of device/port as follows: 0000b PCI Express Endpoin
	* t device, 0001b Legacy PCI Express Endpoint device, 0100b Root Port of P
	* CI Express Root Complex, 0101b Upstream Port of PCI Express Switch, 0110
	* b Downstream Port of PCI Express Switch, 0111b PCIE Express to PCI/PCI-X
	*  Bridge, 1000b PCI/PCI-X to PCI Express Bridge. Transferred to PCI Expre
	* ss Capabilities register. Must be consistent with IS_SWITCH and UPSTREAM
	* _FACING settings.; EP=0x0000; RP=0x0004
	*  PSU_PCIE_ATTRIB_ATTR_50_ATTR_PCIE_CAP_DEVICE_PORT_TYPE      4

	* PCIe Capability's Next Capability Offset pointer to the next item in the
	*  capabilities list, or 00h if this is the final capability.; EP=0x009C;
	* RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_50_ATTR_PCIE_CAP_NEXTPTR               0

	* ATTR_50
	* (OFFSET, MASK, VALUE)      (0XFD4800C8, 0x0000FFF0U ,0x00000040U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_50_OFFSET, 0x0000FFF0U, 0x00000040U);
	/*##################################################################### */

	/*
	* Register : ATTR_105 @ 0XFD4801A4

	* Number of credits that should be advertised for Completion data received
	*  on Virtual Channel 0. The bytes advertised must be less than or equal t
	* o the bram bytes available. See VC0_RX_RAM_LIMIT; EP=0x0172; RP=0x00CD
	*  PSU_PCIE_ATTRIB_ATTR_105_ATTR_VC0_TOTAL_CREDITS_CD          0xCD

	* ATTR_105
	* (OFFSET, MASK, VALUE)      (0XFD4801A4, 0x000007FFU ,0x000000CDU)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_105_OFFSET,
			0x000007FFU, 0x000000CDU);
	/*##################################################################### */

	/*
	* Register : ATTR_106 @ 0XFD4801A8

	* Number of credits that should be advertised for Completion headers recei
	* ved on Virtual Channel 0. The sum of the posted, non posted, and complet
	* ion header credits must be <= 80; EP=0x0048; RP=0x0024
	*  PSU_PCIE_ATTRIB_ATTR_106_ATTR_VC0_TOTAL_CREDITS_CH          0x24

	* Number of credits that should be advertised for Non-Posted headers recei
	* ved on Virtual Channel 0. The number of non posted data credits advertis
	* ed by the block is equal to the number of non posted header credits. The
	*  sum of the posted, non posted, and completion header credits must be <=
	*  80; EP=0x0004; RP=0x000C
	*  PSU_PCIE_ATTRIB_ATTR_106_ATTR_VC0_TOTAL_CREDITS_NPH         0xC

	* ATTR_106
	* (OFFSET, MASK, VALUE)      (0XFD4801A8, 0x00003FFFU ,0x00000624U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_106_OFFSET,
			0x00003FFFU, 0x00000624U);
	/*##################################################################### */

	/*
	* Register : ATTR_107 @ 0XFD4801AC

	* Number of credits that should be advertised for Non-Posted data received
	*  on Virtual Channel 0. The number of non posted data credits advertised
	* by the block is equal to two times the number of non posted header credi
	* ts if atomic operations are supported or is equal to the number of non p
	* osted header credits if atomic operations are not supported. The bytes a
	* dvertised must be less than or equal to the bram bytes available. See VC
	* 0_RX_RAM_LIMIT; EP=0x0008; RP=0x0018
	*  PSU_PCIE_ATTRIB_ATTR_107_ATTR_VC0_TOTAL_CREDITS_NPD         0x18

	* ATTR_107
	* (OFFSET, MASK, VALUE)      (0XFD4801AC, 0x000007FFU ,0x00000018U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_107_OFFSET,
			0x000007FFU, 0x00000018U);
	/*##################################################################### */

	/*
	* Register : ATTR_108 @ 0XFD4801B0

	* Number of credits that should be advertised for Posted data received on
	* Virtual Channel 0. The bytes advertised must be less than or equal to th
	* e bram bytes available. See VC0_RX_RAM_LIMIT; EP=0x0020; RP=0x00B5
	*  PSU_PCIE_ATTRIB_ATTR_108_ATTR_VC0_TOTAL_CREDITS_PD          0xB5

	* ATTR_108
	* (OFFSET, MASK, VALUE)      (0XFD4801B0, 0x000007FFU ,0x000000B5U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_108_OFFSET,
			0x000007FFU, 0x000000B5U);
	/*##################################################################### */

	/*
	* Register : ATTR_109 @ 0XFD4801B4

	* Not currently in use. Invert ECRC generated by block when trn_tecrc_gen_
	* n and trn_terrfwd_n are asserted.; EP=0x0000; RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_109_ATTR_TECRC_EP_INV                  0x0

	* Enables td bit clear and ECRC trim on received TLP's FALSE == don't trim
	*  TRUE == trim.; EP=0x0001; RP=0x0001
	*  PSU_PCIE_ATTRIB_ATTR_109_ATTR_RECRC_CHK_TRIM                0x1

	* Enables ECRC check on received TLP's 0 == don't check 1 == always check
	* 3 == check if enabled by ECRC check enable bit of AER cap structure; EP=
	* 0x0003; RP=0x0003
	*  PSU_PCIE_ATTRIB_ATTR_109_ATTR_RECRC_CHK                     0x3

	* Index of last packet buffer used by TX TLM (i.e. number of buffers - 1).
	*  Calculated from max payload size supported and the number of brams conf
	* igured for transmit; EP=0x001C; RP=0x001C
	*  PSU_PCIE_ATTRIB_ATTR_109_ATTR_VC0_TX_LASTPACKET             0x1c

	* Number of credits that should be advertised for Posted headers received
	* on Virtual Channel 0. The sum of the posted, non posted, and completion
	* header credits must be <= 80; EP=0x0004; RP=0x0020
	*  PSU_PCIE_ATTRIB_ATTR_109_ATTR_VC0_TOTAL_CREDITS_PH          0x20

	* ATTR_109
	* (OFFSET, MASK, VALUE)      (0XFD4801B4, 0x0000FFFFU ,0x00007E20U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_109_OFFSET,
			0x0000FFFFU, 0x00007E20U);
	/*##################################################################### */

	/*
	* Register : ATTR_34 @ 0XFD480088

	* Specifies values to be transferred to Header Type register. Bit 7 should
	*  be set to '0' indicating single-function device. Bit 0 identifies heade
	* r as Type 0 or Type 1, with '0' indicating a Type 0 header.; EP=0x0000;
	* RP=0x0001
	*  PSU_PCIE_ATTRIB_ATTR_34_ATTR_HEADER_TYPE                    0x1

	* ATTR_34
	* (OFFSET, MASK, VALUE)      (0XFD480088, 0x000000FFU ,0x00000001U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_34_OFFSET, 0x000000FFU, 0x00000001U);
	/*##################################################################### */

	/*
	* Register : ATTR_53 @ 0XFD4800D4

	* PM Capability's Next Capability Offset pointer to the next item in the c
	* apabilities list, or 00h if this is the final capability.; EP=0x0048; RP
	* =0x0060
	*  PSU_PCIE_ATTRIB_ATTR_53_ATTR_PM_CAP_NEXTPTR                 0x60

	* ATTR_53
	* (OFFSET, MASK, VALUE)      (0XFD4800D4, 0x000000FFU ,0x00000060U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_53_OFFSET, 0x000000FFU, 0x00000060U);
	/*##################################################################### */

	/*
	* Register : ATTR_41 @ 0XFD4800A4

	* MSI Per-Vector Masking Capable. The value is transferred to the MSI Cont
	* rol Register[8]. When set, adds Mask and Pending Dword to Cap structure;
	*  EP=0x0000; RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_41_ATTR_MSI_CAP_PER_VECTOR_MASKING_CAPABLE 0x0

	* Indicates that the MSI structures exists. If this is FALSE, then the MSI
	*  structure cannot be accessed via either the link or the management port
	* .; EP=0x0001; RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_41_ATTR_MSI_CAP_ON                     0

	* MSI Capability's Next Capability Offset pointer to the next item in the
	* capabilities list, or 00h if this is the final capability.; EP=0x0060; R
	* P=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_41_ATTR_MSI_CAP_NEXTPTR                0x0

	* Indicates that the MSI structures exists. If this is FALSE, then the MSI
	*  structure cannot be accessed via either the link or the management port
	* .; EP=0x0001; RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_41_ATTR_MSI_CAP_ON                     0

	* ATTR_41
	* (OFFSET, MASK, VALUE)      (0XFD4800A4, 0x000003FFU ,0x00000000U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_41_OFFSET, 0x000003FFU, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : ATTR_97 @ 0XFD480184

	* Maximum Link Width. Valid settings are: 000001b x1, 000010b x2, 000100b
	* x4, 001000b x8.; EP=0x0004; RP=0x0004
	*  PSU_PCIE_ATTRIB_ATTR_97_ATTR_LINK_CAP_MAX_LINK_WIDTH        0x1

	* Used by LTSSM to set Maximum Link Width. Valid settings are: 000001b [x1
	* ], 000010b [x2], 000100b [x4], 001000b [x8].; EP=0x0004; RP=0x0004
	*  PSU_PCIE_ATTRIB_ATTR_97_ATTR_LTSSM_MAX_LINK_WIDTH           0x1

	* ATTR_97
	* (OFFSET, MASK, VALUE)      (0XFD480184, 0x00000FFFU ,0x00000041U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_97_OFFSET, 0x00000FFFU, 0x00000041U);
	/*##################################################################### */

	/*
	* Register : ATTR_100 @ 0XFD480190

	* TRUE specifies upstream-facing port. FALSE specifies downstream-facing p
	* ort.; EP=0x0001; RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_100_ATTR_UPSTREAM_FACING               0x0

	* ATTR_100
	* (OFFSET, MASK, VALUE)      (0XFD480190, 0x00000040U ,0x00000000U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_100_OFFSET,
			0x00000040U, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : ATTR_101 @ 0XFD480194

	* Enable the routing of message TLPs to the user through the TRN RX interf
	* ace. A bit value of 1 enables routing of the message TLP to the user. Me
	* ssages are always decoded by the message decoder. Bit 0 - ERR COR, Bit 1
	*  - ERR NONFATAL, Bit 2 - ERR FATAL, Bit 3 - INTA Bit 4 - INTB, Bit 5 - I
	* NTC, Bit 6 - INTD, Bit 7 PM_PME, Bit 8 - PME_TO_ACK, Bit 9 - unlock, Bit
	*  10 PME_Turn_Off; EP=0x0000; RP=0x07FF
	*  PSU_PCIE_ATTRIB_ATTR_101_ATTR_ENABLE_MSG_ROUTE              0x7FF

	* Disable BAR filtering. Does not change the behavior of the bar hit outpu
	* ts; EP=0x0000; RP=0x0001
	*  PSU_PCIE_ATTRIB_ATTR_101_ATTR_DISABLE_BAR_FILTERING         0x1

	* ATTR_101
	* (OFFSET, MASK, VALUE)      (0XFD480194, 0x0000FFE2U ,0x0000FFE2U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_101_OFFSET,
			0x0000FFE2U, 0x0000FFE2U);
	/*##################################################################### */

	/*
	* Register : ATTR_37 @ 0XFD480094

	* Link Bandwidth notification capability. Indicates support for the link b
	* andwidth notification status and interrupt mechanism. Required for Root.
	* ; EP=0x0000; RP=0x0001
	*  PSU_PCIE_ATTRIB_ATTR_37_ATTR_LINK_CAP_LINK_BANDWIDTH_NOTIFICATION_CAP 0x1

	* Maximum Link Speed. Valid settings are: 0001b [2.5 GT/s], 0010b [5.0 GT/
	* s and 2.5 GT/s].; EP=0x0002; RP=0x0002
	*  PSU_PCIE_ATTRIB_ATTR_37_ATTR_LINK_CAP_MAX_LINK_SPEED        0x2

	* Sets the ASPM Optionality Compliance bit, to comply with the 2.1 ASPM Op
	* tionality ECN. Transferred to the Link Capabilities register.; EP=0x0001
	* ; RP=0x0001
	*  PSU_PCIE_ATTRIB_ATTR_37_ATTR_LINK_CAP_ASPM_OPTIONALITY      0x1

	* ATTR_37
	* (OFFSET, MASK, VALUE)      (0XFD480094, 0x00007E00U ,0x00004A00U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_37_OFFSET, 0x00007E00U, 0x00004A00U);
	/*##################################################################### */

	/*
	* Register : ATTR_93 @ 0XFD480174

	* Enables the Replay Timer to use the user-defined LL_REPLAY_TIMEOUT value
	*  (or combined with the built-in value, depending on LL_REPLAY_TIMEOUT_FU
	* NC). If FALSE, the built-in value is used.; EP=0x0000; RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_93_ATTR_LL_REPLAY_TIMEOUT_EN           0x1

	* Sets a user-defined timeout for the Replay Timer to force cause the retr
	* ansmission of unacknowledged TLPs; refer to LL_REPLAY_TIMEOUT_EN and LL_
	* REPLAY_TIMEOUT_FUNC to see how this value is used. The unit for this att
	* ribute is in symbol times, which is 4ns at GEN1 speeds and 2ns at GEN2.;
	*  EP=0x0000; RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_93_ATTR_LL_REPLAY_TIMEOUT              0x1000

	* ATTR_93
	* (OFFSET, MASK, VALUE)      (0XFD480174, 0x0000FFFFU ,0x00009000U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_93_OFFSET, 0x0000FFFFU, 0x00009000U);
	/*##################################################################### */

	/*
	* Register : ID @ 0XFD480200

	* Device ID for the the PCIe Cap Structure Device ID field
	*  PSU_PCIE_ATTRIB_ID_CFG_DEV_ID                               0xd021

	* Vendor ID for the PCIe Cap Structure Vendor ID field
	*  PSU_PCIE_ATTRIB_ID_CFG_VEND_ID                              0x10ee

	* ID
	* (OFFSET, MASK, VALUE)      (0XFD480200, 0xFFFFFFFFU ,0x10EED021U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ID_OFFSET, 0xFFFFFFFFU, 0x10EED021U);
	/*##################################################################### */

	/*
	* Register : SUBSYS_ID @ 0XFD480204

	* Subsystem ID for the the PCIe Cap Structure Subsystem ID field
	*  PSU_PCIE_ATTRIB_SUBSYS_ID_CFG_SUBSYS_ID                     0x7

	* Subsystem Vendor ID for the PCIe Cap Structure Subsystem Vendor ID field
	*  PSU_PCIE_ATTRIB_SUBSYS_ID_CFG_SUBSYS_VEND_ID                0x10ee

	* SUBSYS_ID
	* (OFFSET, MASK, VALUE)      (0XFD480204, 0xFFFFFFFFU ,0x10EE0007U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_SUBSYS_ID_OFFSET,
			0xFFFFFFFFU, 0x10EE0007U);
	/*##################################################################### */

	/*
	* Register : REV_ID @ 0XFD480208

	* Revision ID for the the PCIe Cap Structure
	*  PSU_PCIE_ATTRIB_REV_ID_CFG_REV_ID                           0x0

	* REV_ID
	* (OFFSET, MASK, VALUE)      (0XFD480208, 0x000000FFU ,0x00000000U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_REV_ID_OFFSET, 0x000000FFU, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : ATTR_24 @ 0XFD480060

	* Code identifying basic function, subclass and applicable programming int
	* erface. Transferred to the Class Code register.; EP=0x8000; RP=0x8000
	*  PSU_PCIE_ATTRIB_ATTR_24_ATTR_CLASS_CODE                     0x400

	* ATTR_24
	* (OFFSET, MASK, VALUE)      (0XFD480060, 0x0000FFFFU ,0x00000400U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_24_OFFSET, 0x0000FFFFU, 0x00000400U);
	/*##################################################################### */

	/*
	* Register : ATTR_25 @ 0XFD480064

	* Code identifying basic function, subclass and applicable programming int
	* erface. Transferred to the Class Code register.; EP=0x0005; RP=0x0006
	*  PSU_PCIE_ATTRIB_ATTR_25_ATTR_CLASS_CODE                     0x6

	* INTX Interrupt Generation Capable. If FALSE, this will cause Command[10]
	*  to be hardwired to 0.; EP=0x0001; RP=0x0001
	*  PSU_PCIE_ATTRIB_ATTR_25_ATTR_CMD_INTX_IMPLEMENTED           0

	* ATTR_25
	* (OFFSET, MASK, VALUE)      (0XFD480064, 0x000001FFU ,0x00000006U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_25_OFFSET, 0x000001FFU, 0x00000006U);
	/*##################################################################### */

	/*
	* Register : ATTR_4 @ 0XFD480010

	* Indicates that the AER structures exists. If this is FALSE, then the AER
	*  structure cannot be accessed via either the link or the management port
	* , and AER will be considered to not be present for error management task
	* s (such as what types of error messages are sent if an error is detected
	* ).; EP=0x0001; RP=0x0001
	*  PSU_PCIE_ATTRIB_ATTR_4_ATTR_AER_CAP_ON                      0

	* Indicates that the AER structures exists. If this is FALSE, then the AER
	*  structure cannot be accessed via either the link or the management port
	* , and AER will be considered to not be present for error management task
	* s (such as what types of error messages are sent if an error is detected
	* ).; EP=0x0001; RP=0x0001
	*  PSU_PCIE_ATTRIB_ATTR_4_ATTR_AER_CAP_ON                      0

	* ATTR_4
	* (OFFSET, MASK, VALUE)      (0XFD480010, 0x00001000U ,0x00000000U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_4_OFFSET, 0x00001000U, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : ATTR_89 @ 0XFD480164

	* VSEC's Next Capability Offset pointer to the next item in the capabiliti
	* es list, or 000h if this is the final capability.; EP=0x0140; RP=0x0140
	*  PSU_PCIE_ATTRIB_ATTR_89_ATTR_VSEC_CAP_NEXTPTR               0

	* ATTR_89
	* (OFFSET, MASK, VALUE)      (0XFD480164, 0x00001FFEU ,0x00000000U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_89_OFFSET, 0x00001FFEU, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : ATTR_79 @ 0XFD48013C

	* CRS SW Visibility. Indicates RC can return CRS to SW. Transferred to the
	*  Root Capabilities register.; EP=0x0000; RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_79_ATTR_ROOT_CAP_CRS_SW_VISIBILITY     0

	* ATTR_79
	* (OFFSET, MASK, VALUE)      (0XFD48013C, 0x00000020U ,0x00000000U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_79_OFFSET, 0x00000020U, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : ATTR_43 @ 0XFD4800AC

	* Indicates that the MSIX structures exists. If this is FALSE, then the MS
	* IX structure cannot be accessed via either the link or the management po
	* rt.; EP=0x0001; RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_43_ATTR_MSIX_CAP_ON                    0

	* ATTR_43
	* (OFFSET, MASK, VALUE)      (0XFD4800AC, 0x00000100U ,0x00000000U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_43_OFFSET, 0x00000100U, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : ATTR_48 @ 0XFD4800C0

	* MSI-X Table Size. This value is transferred to the MSI-X Message Control
	* [10:0] field. Set to 0 if MSI-X is not enabled. Note that the core does
	* not implement the table; that must be implemented in user logic.; EP=0x0
	* 003; RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_48_ATTR_MSIX_CAP_TABLE_SIZE            0

	* ATTR_48
	* (OFFSET, MASK, VALUE)      (0XFD4800C0, 0x000007FFU ,0x00000000U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_48_OFFSET, 0x000007FFU, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : ATTR_46 @ 0XFD4800B8

	* MSI-X Table Offset. This value is transferred to the MSI-X Table Offset
	* field. Set to 0 if MSI-X is not enabled.; EP=0x0001; RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_46_ATTR_MSIX_CAP_TABLE_OFFSET          0

	* ATTR_46
	* (OFFSET, MASK, VALUE)      (0XFD4800B8, 0x0000FFFFU ,0x00000000U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_46_OFFSET, 0x0000FFFFU, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : ATTR_47 @ 0XFD4800BC

	* MSI-X Table Offset. This value is transferred to the MSI-X Table Offset
	* field. Set to 0 if MSI-X is not enabled.; EP=0x0000; RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_47_ATTR_MSIX_CAP_TABLE_OFFSET          0

	* ATTR_47
	* (OFFSET, MASK, VALUE)      (0XFD4800BC, 0x00001FFFU ,0x00000000U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_47_OFFSET, 0x00001FFFU, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : ATTR_44 @ 0XFD4800B0

	* MSI-X Pending Bit Array Offset This value is transferred to the MSI-X PB
	* A Offset field. Set to 0 if MSI-X is not enabled.; EP=0x0001; RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_44_ATTR_MSIX_CAP_PBA_OFFSET            0

	* ATTR_44
	* (OFFSET, MASK, VALUE)      (0XFD4800B0, 0x0000FFFFU ,0x00000000U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_44_OFFSET, 0x0000FFFFU, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : ATTR_45 @ 0XFD4800B4

	* MSI-X Pending Bit Array Offset This value is transferred to the MSI-X PB
	* A Offset field. Set to 0 if MSI-X is not enabled.; EP=0x1000; RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_45_ATTR_MSIX_CAP_PBA_OFFSET            0

	* ATTR_45
	* (OFFSET, MASK, VALUE)      (0XFD4800B4, 0x0000FFF8U ,0x00000000U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_45_OFFSET, 0x0000FFF8U, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : CB @ 0XFD48031C

	* DT837748 Enable
	*  PSU_PCIE_ATTRIB_CB_CB1                                      0x0

	* ECO Register 1
	* (OFFSET, MASK, VALUE)      (0XFD48031C, 0x00000002U ,0x00000000U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_CB_OFFSET, 0x00000002U, 0x00000000U);
	/*##################################################################### */

	/*
	* Register : ATTR_35 @ 0XFD48008C

	* Active State PM Support. Indicates the level of active state power manag
	* ement supported by the selected PCI Express Link, encoded as follows: 0
	* Reserved, 1 L0s entry supported, 2 Reserved, 3 L0s and L1 entry supporte
	* d.; EP=0x0001; RP=0x0001
	*  PSU_PCIE_ATTRIB_ATTR_35_ATTR_LINK_CAP_ASPM_SUPPORT          0x0

	* Data Link Layer Link Active status notification is supported. This is op
	* tional for Upstream ports.; EP=0x0000; RP=0x0000
	*  PSU_PCIE_ATTRIB_ATTR_35_ATTR_LINK_CAP_DLL_LINK_ACTIVE_REPORTING_CAP 1

	* ATTR_35
	* (OFFSET, MASK, VALUE)      (0XFD48008C, 0x0000B000U ,0x00008000U)
	*/
	writeRegMsk(pcireg, PCIE_ATTRIB_ATTR_35_OFFSET, 0x0000B000U, 0x00008000U);
	/*##################################################################### */
}


static int pcie_phyDeassertPcieCtrlReset(void)
{
	platformctl_t ctl3 = {
		.action = pctl_set,
		.type = pctl_devreset,
		.devreset.dev = pctl_devreset_fpd_pcie_ctrl,
		.devreset.state = 0,
	};
	return platformctl(&ctl3);
}


int tebf0808_pciePsGtrPhyInit(void)
{
	int ret = 0;

	uint32_t *serdes = mmap(NULL, SERDES_SIZE, PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, SERDES_ADDRESS);
	if (MAP_FAILED == serdes) {
		fprintf(stderr, "pcie-phy: fail to map SERDES registers memory\n");
		return -1;
	}
	uint32_t *pcireg = mmap(NULL, PCIREG_SIZE,
			PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
			-1,
			PCIREG_ADDRESS);
	if (MAP_FAILED == pcireg) {
		munmap((void *)serdes, SERDES_SIZE);
		munmap((void *)pcireg, PCIREG_SIZE);
		fprintf(stderr, "pcie-phy: fail to map PCIE registers memory\n");
		return -1;
	}

	printf("pcie-phy: initialise PCI Express PHY\n");

	ret = pcie_phyInitPcieClock();
	if (ret != 0) {
		munmap((void *)serdes, SERDES_SIZE);
		munmap((void *)pcireg, PCIREG_SIZE);
		fprintf(stderr, "pcie-phy: fail to config clock\n");
		return ret;
	}
	ret = pcie_phyDeassertGtReset();
	if (ret != 0) {
		munmap((void *)serdes, SERDES_SIZE);
		munmap((void *)pcireg, PCIREG_SIZE);
		fprintf(stderr, "pcie-phy: fail to deassert reset on PS GTR\n");
		return ret;
	}
	ret = pcie_phyAssertResets();
	if (ret != 0) {
		munmap((void *)serdes, SERDES_SIZE);
		munmap((void *)pcireg, PCIREG_SIZE);
		fprintf(stderr, "pcie-phy: fail to assert reset on all 3 PCIE peripherals\n");
		return ret;
	}
	int retNegated = serdes_fixcal_code(serdes);
	if (retNegated != 1) {
		munmap((void *)serdes, SERDES_SIZE);
		munmap((void *)pcireg, PCIREG_SIZE);
		fprintf(stderr, "pcie-phy: fail to fix calibration of SERDES peripheral\n");
		return -1;
	}

	(void)psu_serdes_init_data(serdes);

	ret = pcie_phyDeassertPcieResets();
	if (ret != 0) {
		munmap((void *)serdes, SERDES_SIZE);
		munmap((void *)pcireg, PCIREG_SIZE);
		fprintf(stderr, "pcie-phy: fail to deassert PCIE CFG and PCIE BRIDGE reset\n");
		return ret;
	}

	pcie_phyConfigPcie(pcireg, serdes);

	ret = pcie_phyDeassertPcieCtrlReset();
	if (ret != 0) {
		munmap((void *)serdes, SERDES_SIZE);
		munmap((void *)pcireg, PCIREG_SIZE);
		fprintf(stderr, "pcie-phy: fail to deassert PCIE CTRL reset\n");
		return ret;
	}

	/* Wait for PHY ready */
	int wait_ret = mask_poll(serdes, SERDES_L0_PLL_STATUS_READ_1_OFFSET, 0x00000010U);
	if (wait_ret == 0) {
		munmap((void *)serdes, SERDES_SIZE);
		munmap((void *)pcireg, PCIREG_SIZE);
		fprintf(stderr, "pcie-phy: timeout on PS GTR 0 ready wait\n");
		return -1;
	}

	/* Unmap memory */
	munmap((void *)serdes, SERDES_SIZE);
	munmap((void *)pcireg, PCIREG_SIZE);

	return ret;
}
