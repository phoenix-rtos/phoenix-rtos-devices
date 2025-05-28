/*
 * Phoenix-RTOS
 *
 * ZynqMP PCI Express driver
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
#include <string.h>
#include <unistd.h>
#include <endian.h>
#include <sys/interrupt.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/platform.h>
#include <sys/threads.h>
#include <posix/utils.h>

#include <phoenix/arch/aarch64/zynqmp/zynqmp.h>

#include <board_config.h>

#include <i2c.h>

#include <fsbl2.h>
#include <fsbl2_integration.h>

#define XFsbl_Out32(Addr, Data)		Xil_Out32(Addr, Data)
#define XFsbl_In32(Addr)		Xil_In32(Addr)

#define GPIO_BASEADDR      0XFF0A0000U
#define GPIO_DATA_1    ( ( GPIO_BASEADDR ) + 0X00000044U )
#define GPIO_DIRM_1    ( ( GPIO_BASEADDR ) + 0X00000244U )
#define GPIO_OEN_1     ( ( GPIO_BASEADDR ) + 0X00000248U )
#define GPIO_MIO31_MASK	0x00000020U

#define DELAY_1_US			0x1U
#define DELAY_5_US			0x5U
#define DELAY_32_US			0x20U
#define DELAY_500_US	  0x200U
#define DELAY_1000_US	  0x400U

#define SI5345_I2C_ADDRESS 0x69

#define SI5345_PAGE_ADDRESS 0x01

#define SERDES_SIZE    0x20000
#define SERDES_ADDRESS 0xfd400000

#define BREG_SIZE	   0x1000
#define BREG_ADDRESS   0xfd0e0000

#define PCIREG_SIZE    0x1000
#define PCIREG_ADDRESS 0xfd480000

#define CFG_SIZE 0x10000000
#define CFG_ADDRESS 0x8000000000

#define GIC_PCIE_INTX_IRQ 148

#define L0_TM_PLL_DIG_37          0x2094
#define L0_L0_REF_CLK_SEL         0x2860
#define L0_PLL_SS_STEPS_0_LSB     0x2368
#define L0_PLL_SS_STEPS_1_MSB     0x236c
#define L0_PLL_SS_STEP_SIZE_0_LSB 0x2370
#define L0_PLL_SS_STEP_SIZE_1     0x2374
#define L0_PLL_SS_STEP_SIZE_2     0x2378
#define L0_PLL_SS_STEP_SIZE_3_MSB 0x237c
#define L0_PLL_STATUS_READ_1      0x23e4
#define PLL_REF_SEL0              0x10000
#define ICM_CFG0                  0x10010

#define BRCFG_PCIE_RX0			0x00000000
#define BRCFG_PCIE_RX1			0x00000004
#define BRCFG_INTERRUPT			0x00000010
#define BRCFG_PCIE_RX_MSG_FILTER	0x00000020

#define E_BREG_CAPABILITIES		0x00000200
#define E_BREG_CONTROL			0x00000208
#define E_BREG_BASE_LO			0x00000210
#define E_BREG_BASE_HI			0x00000214
#define E_ECAM_CAPABILITIES		0x00000220
#define E_ECAM_CONTROL			0x00000228
#define E_ECAM_BASE_LO			0x00000230
#define E_ECAM_BASE_HI			0x00000234

#define I_MSII_CAPABILITIES		0x00000300
#define I_MSII_CONTROL			0x00000308
#define I_MSII_BASE_LO			0x00000310
#define I_MSII_BASE_HI			0x00000314
#define I_ISUB_CONTROL			0x000003E8

#define PS_LINKUP_OFFSET		0x00000238

#define upper_32_bits(n) ((uint32_t)(((n) >> 16) >> 16))

#define lower_32_bits(n) ((uint32_t)((n) & 0xffffffff))

/* Data structure used by the driver */
static struct {
	uint32_t *breg;
	uint32_t *pcireg;
	uint32_t *cfg;
	handle_t cond;               /**< Conditional variable for synchronizing interrupts */
	handle_t inth;               /**< Interrupt handler object */
	handle_t lock;               /**< Mutex used with the conditional variable for synchronization */
} pcie;


typedef struct
{
	uint16_t address;
	uint8_t value;
} si5345_revd_register_t;

static int serdes_rst_seq (u32 pllsel, u32 lane3_protocol, u32 lane3_rate, u32 lane2_protocol, u32 lane2_rate, u32 lane1_protocol, u32 lane1_rate, u32 lane0_protocol, u32 lane0_rate);

static int serdes_bist_static_settings(u32 lane_active);

static int serdes_bist_run(u32 lane_active);

static int serdes_bist_result(u32 lane_active);

static int serdes_illcalib_pcie_gen1 (u32 pllsel, u32 lane3_protocol, u32 lane3_rate, u32 lane2_protocol, u32 lane2_rate, u32 lane1_protocol, u32 lane1_rate, u32 lane0_protocol, u32 lane0_rate, u32 gen2_calib);

static int serdes_illcalib (u32 lane3_protocol, u32 lane3_rate, u32 lane2_protocol, u32 lane2_rate, u32 lane1_protocol, u32 lane1_rate, u32 lane0_protocol, u32 lane0_rate);

unsigned long psu_serdes_init_data(void)
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
	PSU_Mask_Write(SERDES_PLL_REF_SEL0_OFFSET, 0x0000001FU, 0x0000000DU);
/*##################################################################### */

    /*
    * GT REFERENCE CLOCK FREQUENCY SELECTION
    */
    /*
    * Register : L0_L0_REF_CLK_SEL @ 0XFD402860

    * Sel of lane 0 ref clock local mux. Set to 1 to select lane 0 slicer outp
    * ut. Set to 0 to select lane0 ref clock mux output.
    *  PSU_SERDES_L0_L0_REF_CLK_SEL_L0_REF_CLK_LCL_SEL             0x1

    * Lane0 Ref Clock Selection Register
    * (OFFSET, MASK, VALUE)      (0XFD402860, 0x00000080U ,0x00000080U)
    */
	PSU_Mask_Write(SERDES_L0_L0_REF_CLK_SEL_OFFSET,
		0x00000080U, 0x00000080U);
/*##################################################################### */

    /*
    * ENABLE SPREAD SPECTRUM
    */
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
	PSU_Mask_Write(SERDES_L0_TM_AUX_0_OFFSET, 0x00000020U, 0x00000020U);
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
	PSU_Mask_Write(SERDES_L0_TM_DIG_8_OFFSET, 0x00000010U, 0x00000010U);
/*##################################################################### */

    /*
    * Register : L1_TM_DIG_8 @ 0XFD405074

    * Enable Eye Surf
    *  PSU_SERDES_L1_TM_DIG_8_EYESURF_ENABLE                       0x1

    * Test modes for Elastic buffer and enabling Eye Surf
    * (OFFSET, MASK, VALUE)      (0XFD405074, 0x00000010U ,0x00000010U)
    */
	PSU_Mask_Write(SERDES_L1_TM_DIG_8_OFFSET, 0x00000010U, 0x00000010U);
/*##################################################################### */

    /*
    * Register : L2_TM_DIG_8 @ 0XFD409074

    * Enable Eye Surf
    *  PSU_SERDES_L2_TM_DIG_8_EYESURF_ENABLE                       0x1

    * Test modes for Elastic buffer and enabling Eye Surf
    * (OFFSET, MASK, VALUE)      (0XFD409074, 0x00000010U ,0x00000010U)
    */
	PSU_Mask_Write(SERDES_L2_TM_DIG_8_OFFSET, 0x00000010U, 0x00000010U);
/*##################################################################### */

    /*
    * Register : L3_TM_DIG_8 @ 0XFD40D074

    * Enable Eye Surf
    *  PSU_SERDES_L3_TM_DIG_8_EYESURF_ENABLE                       0x1

    * Test modes for Elastic buffer and enabling Eye Surf
    * (OFFSET, MASK, VALUE)      (0XFD40D074, 0x00000010U ,0x00000010U)
    */
	PSU_Mask_Write(SERDES_L3_TM_DIG_8_OFFSET, 0x00000010U, 0x00000010U);
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
	PSU_Mask_Write(SERDES_L0_TM_MISC2_OFFSET, 0x00000080U, 0x00000080U);
/*##################################################################### */

    /*
    * Register : L0_TM_IQ_ILL1 @ 0XFD4018F8

    * IQ ILL F0 CALCODE bypass value. MPHY : G1a, PCIE : Gen 1, SATA : Gen1 ,
    * USB3 : SS
    *  PSU_SERDES_L0_TM_IQ_ILL1_ILL_BYPASS_IQ_CALCODE_F0           0x64

    * iqpi cal code
    * (OFFSET, MASK, VALUE)      (0XFD4018F8, 0x000000FFU ,0x00000064U)
    */
	PSU_Mask_Write(SERDES_L0_TM_IQ_ILL1_OFFSET,
		0x000000FFU, 0x00000064U);
/*##################################################################### */

    /*
    * Register : L0_TM_IQ_ILL2 @ 0XFD4018FC

    * IQ ILL F1 CALCODE bypass value. MPHY : G1b, PCIE : Gen2, SATA: Gen2
    *  PSU_SERDES_L0_TM_IQ_ILL2_ILL_BYPASS_IQ_CALCODE_F1           0x64

    * iqpi cal code
    * (OFFSET, MASK, VALUE)      (0XFD4018FC, 0x000000FFU ,0x00000064U)
    */
	PSU_Mask_Write(SERDES_L0_TM_IQ_ILL2_OFFSET,
		0x000000FFU, 0x00000064U);
/*##################################################################### */

    /*
    * Register : L0_TM_ILL12 @ 0XFD401990

    * G1A pll ctr bypass value
    *  PSU_SERDES_L0_TM_ILL12_G1A_PLL_CTR_BYP_VAL                  0x11

    * ill pll counter values
    * (OFFSET, MASK, VALUE)      (0XFD401990, 0x000000FFU ,0x00000011U)
    */
	PSU_Mask_Write(SERDES_L0_TM_ILL12_OFFSET, 0x000000FFU, 0x00000011U);
/*##################################################################### */

    /*
    * Register : L0_TM_E_ILL1 @ 0XFD401924

    * E ILL F0 CALCODE bypass value. MPHY : G1a, PCIE : Gen 1, SATA : Gen1 , U
    * SB3 : SS
    *  PSU_SERDES_L0_TM_E_ILL1_ILL_BYPASS_E_CALCODE_F0             0x4

    * epi cal code
    * (OFFSET, MASK, VALUE)      (0XFD401924, 0x000000FFU ,0x00000004U)
    */
	PSU_Mask_Write(SERDES_L0_TM_E_ILL1_OFFSET, 0x000000FFU, 0x00000004U);
/*##################################################################### */

    /*
    * Register : L0_TM_E_ILL2 @ 0XFD401928

    * E ILL F1 CALCODE bypass value. MPHY : G1b, PCIE : Gen2, SATA: Gen2
    *  PSU_SERDES_L0_TM_E_ILL2_ILL_BYPASS_E_CALCODE_F1             0xFE

    * epi cal code
    * (OFFSET, MASK, VALUE)      (0XFD401928, 0x000000FFU ,0x000000FEU)
    */
	PSU_Mask_Write(SERDES_L0_TM_E_ILL2_OFFSET, 0x000000FFU, 0x000000FEU);
/*##################################################################### */

    /*
    * Register : L0_TM_IQ_ILL3 @ 0XFD401900

    * IQ ILL F2CALCODE bypass value. MPHY : G2a, SATA : Gen3
    *  PSU_SERDES_L0_TM_IQ_ILL3_ILL_BYPASS_IQ_CALCODE_F2           0x64

    * iqpi cal code
    * (OFFSET, MASK, VALUE)      (0XFD401900, 0x000000FFU ,0x00000064U)
    */
	PSU_Mask_Write(SERDES_L0_TM_IQ_ILL3_OFFSET,
		0x000000FFU, 0x00000064U);
/*##################################################################### */

    /*
    * Register : L0_TM_E_ILL3 @ 0XFD40192C

    * E ILL F2CALCODE bypass value. MPHY : G2a, SATA : Gen3
    *  PSU_SERDES_L0_TM_E_ILL3_ILL_BYPASS_E_CALCODE_F2             0x0

    * epi cal code
    * (OFFSET, MASK, VALUE)      (0XFD40192C, 0x000000FFU ,0x00000000U)
    */
	PSU_Mask_Write(SERDES_L0_TM_E_ILL3_OFFSET, 0x000000FFU, 0x00000000U);
/*##################################################################### */

    /*
    * Register : L0_TM_ILL8 @ 0XFD401980

    * ILL calibration code change wait time
    *  PSU_SERDES_L0_TM_ILL8_ILL_CAL_ITER_WAIT                     0xFF

    * ILL cal routine control
    * (OFFSET, MASK, VALUE)      (0XFD401980, 0x000000FFU ,0x000000FFU)
    */
	PSU_Mask_Write(SERDES_L0_TM_ILL8_OFFSET, 0x000000FFU, 0x000000FFU);
/*##################################################################### */

    /*
    * Register : L0_TM_IQ_ILL8 @ 0XFD401914

    * IQ ILL polytrim bypass value
    *  PSU_SERDES_L0_TM_IQ_ILL8_ILL_BYPASS_IQ_POLYTRIM_VAL         0xF7

    * iqpi polytrim
    * (OFFSET, MASK, VALUE)      (0XFD401914, 0x000000FFU ,0x000000F7U)
    */
	PSU_Mask_Write(SERDES_L0_TM_IQ_ILL8_OFFSET,
		0x000000FFU, 0x000000F7U);
/*##################################################################### */

    /*
    * Register : L0_TM_IQ_ILL9 @ 0XFD401918

    * bypass IQ polytrim
    *  PSU_SERDES_L0_TM_IQ_ILL9_ILL_BYPASS_IQ_POLYTIM              0x1

    * enables for lf,constant gm trim and polytirm
    * (OFFSET, MASK, VALUE)      (0XFD401918, 0x00000001U ,0x00000001U)
    */
	PSU_Mask_Write(SERDES_L0_TM_IQ_ILL9_OFFSET,
		0x00000001U, 0x00000001U);
/*##################################################################### */

    /*
    * Register : L0_TM_E_ILL8 @ 0XFD401940

    * E ILL polytrim bypass value
    *  PSU_SERDES_L0_TM_E_ILL8_ILL_BYPASS_E_POLYTRIM_VAL           0xF7

    * epi polytrim
    * (OFFSET, MASK, VALUE)      (0XFD401940, 0x000000FFU ,0x000000F7U)
    */
	PSU_Mask_Write(SERDES_L0_TM_E_ILL8_OFFSET, 0x000000FFU, 0x000000F7U);
/*##################################################################### */

    /*
    * Register : L0_TM_E_ILL9 @ 0XFD401944

    * bypass E polytrim
    *  PSU_SERDES_L0_TM_E_ILL9_ILL_BYPASS_E_POLYTIM                0x1

    * enables for lf,constant gm trim and polytirm
    * (OFFSET, MASK, VALUE)      (0XFD401944, 0x00000001U ,0x00000001U)
    */
	PSU_Mask_Write(SERDES_L0_TM_E_ILL9_OFFSET, 0x00000001U, 0x00000001U);
/*##################################################################### */

    /*
    * Register : L0_TM_ILL13 @ 0XFD401994

    * ILL cal idle val refcnt
    *  PSU_SERDES_L0_TM_ILL13_ILL_CAL_IDLE_VAL_REFCNT              0x7

    * ill cal idle value count
    * (OFFSET, MASK, VALUE)      (0XFD401994, 0x00000007U ,0x00000007U)
    */
	PSU_Mask_Write(SERDES_L0_TM_ILL13_OFFSET, 0x00000007U, 0x00000007U);
/*##################################################################### */

    /*
    * Register : L1_TM_ILL13 @ 0XFD405994

    * ILL cal idle val refcnt
    *  PSU_SERDES_L1_TM_ILL13_ILL_CAL_IDLE_VAL_REFCNT              0x7

    * ill cal idle value count
    * (OFFSET, MASK, VALUE)      (0XFD405994, 0x00000007U ,0x00000007U)
    */
	PSU_Mask_Write(SERDES_L1_TM_ILL13_OFFSET, 0x00000007U, 0x00000007U);
/*##################################################################### */

    /*
    * Register : L2_TM_ILL13 @ 0XFD409994

    * ILL cal idle val refcnt
    *  PSU_SERDES_L2_TM_ILL13_ILL_CAL_IDLE_VAL_REFCNT              0x7

    * ill cal idle value count
    * (OFFSET, MASK, VALUE)      (0XFD409994, 0x00000007U ,0x00000007U)
    */
	PSU_Mask_Write(SERDES_L2_TM_ILL13_OFFSET, 0x00000007U, 0x00000007U);
/*##################################################################### */

    /*
    * Register : L3_TM_ILL13 @ 0XFD40D994

    * ILL cal idle val refcnt
    *  PSU_SERDES_L3_TM_ILL13_ILL_CAL_IDLE_VAL_REFCNT              0x7

    * ill cal idle value count
    * (OFFSET, MASK, VALUE)      (0XFD40D994, 0x00000007U ,0x00000007U)
    */
	PSU_Mask_Write(SERDES_L3_TM_ILL13_OFFSET, 0x00000007U, 0x00000007U);
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
	PSU_Mask_Write(SERDES_L0_TM_DIG_10_OFFSET, 0x0000000FU, 0x00000001U);
/*##################################################################### */

    /*
    * Register : L1_TM_DIG_10 @ 0XFD40507C

    * CDR lock wait time. (1-16 us). cdr_lock_wait_time = 4'b xxxx + 4'b 0001
    *  PSU_SERDES_L1_TM_DIG_10_CDR_BIT_LOCK_TIME                   0x1

    * test control for changing cdr lock wait time
    * (OFFSET, MASK, VALUE)      (0XFD40507C, 0x0000000FU ,0x00000001U)
    */
	PSU_Mask_Write(SERDES_L1_TM_DIG_10_OFFSET, 0x0000000FU, 0x00000001U);
/*##################################################################### */

    /*
    * Register : L2_TM_DIG_10 @ 0XFD40907C

    * CDR lock wait time. (1-16 us). cdr_lock_wait_time = 4'b xxxx + 4'b 0001
    *  PSU_SERDES_L2_TM_DIG_10_CDR_BIT_LOCK_TIME                   0x1

    * test control for changing cdr lock wait time
    * (OFFSET, MASK, VALUE)      (0XFD40907C, 0x0000000FU ,0x00000001U)
    */
	PSU_Mask_Write(SERDES_L2_TM_DIG_10_OFFSET, 0x0000000FU, 0x00000001U);
/*##################################################################### */

    /*
    * Register : L3_TM_DIG_10 @ 0XFD40D07C

    * CDR lock wait time. (1-16 us). cdr_lock_wait_time = 4'b xxxx + 4'b 0001
    *  PSU_SERDES_L3_TM_DIG_10_CDR_BIT_LOCK_TIME                   0x1

    * test control for changing cdr lock wait time
    * (OFFSET, MASK, VALUE)      (0XFD40D07C, 0x0000000FU ,0x00000001U)
    */
	PSU_Mask_Write(SERDES_L3_TM_DIG_10_OFFSET, 0x0000000FU, 0x00000001U);
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
	PSU_Mask_Write(SERDES_L0_TM_RST_DLY_OFFSET,
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
	PSU_Mask_Write(SERDES_L0_TM_ANA_BYP_15_OFFSET,
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
	PSU_Mask_Write(SERDES_L0_TM_ANA_BYP_12_OFFSET,
		0x00000040U, 0x00000040U);
/*##################################################################### */

    /*
    * Register : L1_TM_RST_DLY @ 0XFD4059A4

    * Delay apb reset by specified amount
    *  PSU_SERDES_L1_TM_RST_DLY_APB_RST_DLY                        0xFF

    * reset delay for apb reset w.r.t pso of hsrx
    * (OFFSET, MASK, VALUE)      (0XFD4059A4, 0x000000FFU ,0x000000FFU)
    */
	PSU_Mask_Write(SERDES_L1_TM_RST_DLY_OFFSET,
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
	PSU_Mask_Write(SERDES_L1_TM_ANA_BYP_15_OFFSET,
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
	PSU_Mask_Write(SERDES_L1_TM_ANA_BYP_12_OFFSET,
		0x00000040U, 0x00000040U);
/*##################################################################### */

    /*
    * Register : L2_TM_RST_DLY @ 0XFD4099A4

    * Delay apb reset by specified amount
    *  PSU_SERDES_L2_TM_RST_DLY_APB_RST_DLY                        0xFF

    * reset delay for apb reset w.r.t pso of hsrx
    * (OFFSET, MASK, VALUE)      (0XFD4099A4, 0x000000FFU ,0x000000FFU)
    */
	PSU_Mask_Write(SERDES_L2_TM_RST_DLY_OFFSET,
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
	PSU_Mask_Write(SERDES_L2_TM_ANA_BYP_15_OFFSET,
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
	PSU_Mask_Write(SERDES_L2_TM_ANA_BYP_12_OFFSET,
		0x00000040U, 0x00000040U);
/*##################################################################### */

    /*
    * Register : L3_TM_RST_DLY @ 0XFD40D9A4

    * Delay apb reset by specified amount
    *  PSU_SERDES_L3_TM_RST_DLY_APB_RST_DLY                        0xFF

    * reset delay for apb reset w.r.t pso of hsrx
    * (OFFSET, MASK, VALUE)      (0XFD40D9A4, 0x000000FFU ,0x000000FFU)
    */
	PSU_Mask_Write(SERDES_L3_TM_RST_DLY_OFFSET,
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
	PSU_Mask_Write(SERDES_L3_TM_ANA_BYP_15_OFFSET,
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
	PSU_Mask_Write(SERDES_L3_TM_ANA_BYP_12_OFFSET,
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
	PSU_Mask_Write(SERDES_L0_TM_MISC3_OFFSET, 0x00000003U, 0x00000000U);
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
	PSU_Mask_Write(SERDES_L1_TM_MISC3_OFFSET, 0x00000003U, 0x00000000U);
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
	PSU_Mask_Write(SERDES_L2_TM_MISC3_OFFSET, 0x00000003U, 0x00000000U);
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
	PSU_Mask_Write(SERDES_L3_TM_MISC3_OFFSET, 0x00000003U, 0x00000000U);
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
	PSU_Mask_Write(SERDES_L0_TM_EQ11_OFFSET, 0x00000010U, 0x00000010U);
/*##################################################################### */

    /*
    * Register : L1_TM_EQ11 @ 0XFD405978

    * Force EQ offset correction algo off if not forced on
    *  PSU_SERDES_L1_TM_EQ11_FORCE_EQ_OFFS_OFF                     0x1

    * eq dynamic offset correction
    * (OFFSET, MASK, VALUE)      (0XFD405978, 0x00000010U ,0x00000010U)
    */
	PSU_Mask_Write(SERDES_L1_TM_EQ11_OFFSET, 0x00000010U, 0x00000010U);
/*##################################################################### */

    /*
    * Register : L2_TM_EQ11 @ 0XFD409978

    * Force EQ offset correction algo off if not forced on
    *  PSU_SERDES_L2_TM_EQ11_FORCE_EQ_OFFS_OFF                     0x1

    * eq dynamic offset correction
    * (OFFSET, MASK, VALUE)      (0XFD409978, 0x00000010U ,0x00000010U)
    */
	PSU_Mask_Write(SERDES_L2_TM_EQ11_OFFSET, 0x00000010U, 0x00000010U);
/*##################################################################### */

    /*
    * Register : L3_TM_EQ11 @ 0XFD40D978

    * Force EQ offset correction algo off if not forced on
    *  PSU_SERDES_L3_TM_EQ11_FORCE_EQ_OFFS_OFF                     0x1

    * eq dynamic offset correction
    * (OFFSET, MASK, VALUE)      (0XFD40D978, 0x00000010U ,0x00000010U)
    */
	PSU_Mask_Write(SERDES_L3_TM_EQ11_OFFSET, 0x00000010U, 0x00000010U);
/*##################################################################### */

    /*
    * SERDES ILL CALIB
    */
		serdes_illcalib(0,0,0,0,0,0,1,1);

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
	PSU_Mask_Write(SIOU_ECO_0_OFFSET, 0xFFFFFFFFU, 0x00000001U);
/*##################################################################### */

    /*
    * GT LANE SETTINGS
    */
    /*
    * Register : ICM_CFG0 @ 0XFD410010

    * Controls UPHY Lane 0 protocol configuration. 0 - PowerDown, 1 - PCIe .0,
    *  2 - Sata0, 3 - USB0, 4 - DP.1, 5 - SGMII0, 6 - Unused, 7 - Unused
    *  PSU_SERDES_ICM_CFG0_L0_ICM_CFG                              1

    * ICM Configuration Register 0
    * (OFFSET, MASK, VALUE)      (0XFD410010, 0x00000007U ,0x00000001U)
    */
	PSU_Mask_Write(SERDES_ICM_CFG0_OFFSET, 0x00000007U, 0x00000001U);
/*##################################################################### */

    /*
    * CHECKING PLL LOCK
    */
    /*
    * ENABLE SERIAL DATA MUX DEEMPH
    */
    /*
    * CDR AND RX EQUALIZATION SETTINGS
    */
    /*
    * GEM SERDES SETTINGS
    */
    /*
    * ENABLE PRE EMPHAIS AND VOLTAGE SWING
    */

	return 1;
}
unsigned long psu_resetout_init_data(void)
{
    /*
    * TAKING SERDES PERIPHERAL OUT OF RESET RESET
    */
    /*
    * PUTTING PCIE CFG AND BRIDGE IN RESET
    */
    /*
    * Register : RST_FPD_TOP @ 0XFD1A0100

    * PCIE config reset
    *  PSU_CRF_APB_RST_FPD_TOP_PCIE_CFG_RESET                      0X0

    * PCIE bridge block level reset (AXI interface)
    *  PSU_CRF_APB_RST_FPD_TOP_PCIE_BRIDGE_RESET                   0X0

    * FPD Block level software controlled reset
    * (OFFSET, MASK, VALUE)      (0XFD1A0100, 0x000C0000U ,0x00000000U)
    */
	PSU_Mask_Write(CRF_APB_RST_FPD_TOP_OFFSET, 0x000C0000U, 0x00000000U);
/*##################################################################### */

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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_25_OFFSET, 0x00000200U, 0x00000200U);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_7_OFFSET, 0x0000FFFFU, 0x00000000U);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_8_OFFSET, 0x0000FFFFU, 0x00000000U);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_9_OFFSET, 0x0000FFFFU, 0x00000000U);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_10_OFFSET, 0x0000FFFFU, 0x00000000U);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_11_OFFSET, 0x0000FFFFU, 0x0000FFFFU);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_12_OFFSET, 0x0000FFFFU, 0x000000FFU);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_13_OFFSET, 0x0000FFFFU, 0x00000000U);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_14_OFFSET, 0x0000FFFFU, 0x0000FFFFU);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_15_OFFSET, 0x0000FFFFU, 0x0000FFF0U);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_16_OFFSET, 0x0000FFFFU, 0x0000FFF0U);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_17_OFFSET, 0x0000FFFFU, 0x0000FFF1U);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_18_OFFSET, 0x0000FFFFU, 0x0000FFF1U);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_27_OFFSET, 0x00000738U, 0x00000100U);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_50_OFFSET, 0x0000FFF0U, 0x00000040U);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_105_OFFSET,
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_106_OFFSET,
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_107_OFFSET,
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_108_OFFSET,
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_109_OFFSET,
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_34_OFFSET, 0x000000FFU, 0x00000001U);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_53_OFFSET, 0x000000FFU, 0x00000060U);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_41_OFFSET, 0x000003FFU, 0x00000000U);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_97_OFFSET, 0x00000FFFU, 0x00000041U);
/*##################################################################### */

    /*
    * Register : ATTR_100 @ 0XFD480190

    * TRUE specifies upstream-facing port. FALSE specifies downstream-facing p
    * ort.; EP=0x0001; RP=0x0000
    *  PSU_PCIE_ATTRIB_ATTR_100_ATTR_UPSTREAM_FACING               0x0

    * ATTR_100
    * (OFFSET, MASK, VALUE)      (0XFD480190, 0x00000040U ,0x00000000U)
    */
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_100_OFFSET,
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_101_OFFSET,
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_37_OFFSET, 0x00007E00U, 0x00004A00U);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_93_OFFSET, 0x0000FFFFU, 0x00009000U);
/*##################################################################### */

    /*
    * Register : ID @ 0XFD480200

    * Device ID for the the PCIe Cap Structure Device ID field
    *  PSU_PCIE_ATTRIB_ID_CFG_DEV_ID                               0xd011

    * Vendor ID for the PCIe Cap Structure Vendor ID field
    *  PSU_PCIE_ATTRIB_ID_CFG_VEND_ID                              0x10ee

    * ID
    * (OFFSET, MASK, VALUE)      (0XFD480200, 0xFFFFFFFFU ,0x10EED011U)
    */
	PSU_Mask_Write(PCIE_ATTRIB_ID_OFFSET, 0xFFFFFFFFU, 0x10EED011U);
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
	PSU_Mask_Write(PCIE_ATTRIB_SUBSYS_ID_OFFSET,
		0xFFFFFFFFU, 0x10EE0007U);
/*##################################################################### */

    /*
    * Register : REV_ID @ 0XFD480208

    * Revision ID for the the PCIe Cap Structure
    *  PSU_PCIE_ATTRIB_REV_ID_CFG_REV_ID                           0x0

    * REV_ID
    * (OFFSET, MASK, VALUE)      (0XFD480208, 0x000000FFU ,0x00000000U)
    */
	PSU_Mask_Write(PCIE_ATTRIB_REV_ID_OFFSET, 0x000000FFU, 0x00000000U);
/*##################################################################### */

    /*
    * Register : ATTR_24 @ 0XFD480060

    * Code identifying basic function, subclass and applicable programming int
    * erface. Transferred to the Class Code register.; EP=0x8000; RP=0x8000
    *  PSU_PCIE_ATTRIB_ATTR_24_ATTR_CLASS_CODE                     0x8000

    * ATTR_24
    * (OFFSET, MASK, VALUE)      (0XFD480060, 0x0000FFFFU ,0x00008000U)
    */
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_24_OFFSET, 0x0000FFFFU, 0x00008000U);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_25_OFFSET, 0x000001FFU, 0x00000006U);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_4_OFFSET, 0x00001000U, 0x00000000U);
/*##################################################################### */

    /*
    * Register : ATTR_89 @ 0XFD480164

    * VSEC's Next Capability Offset pointer to the next item in the capabiliti
    * es list, or 000h if this is the final capability.; EP=0x0140; RP=0x0140
    *  PSU_PCIE_ATTRIB_ATTR_89_ATTR_VSEC_CAP_NEXTPTR               0

    * ATTR_89
    * (OFFSET, MASK, VALUE)      (0XFD480164, 0x00001FFEU ,0x00000000U)
    */
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_89_OFFSET, 0x00001FFEU, 0x00000000U);
/*##################################################################### */

    /*
    * Register : ATTR_79 @ 0XFD48013C

    * CRS SW Visibility. Indicates RC can return CRS to SW. Transferred to the
    *  Root Capabilities register.; EP=0x0000; RP=0x0000
    *  PSU_PCIE_ATTRIB_ATTR_79_ATTR_ROOT_CAP_CRS_SW_VISIBILITY     0

    * ATTR_79
    * (OFFSET, MASK, VALUE)      (0XFD48013C, 0x00000020U ,0x00000000U)
    */
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_79_OFFSET, 0x00000020U, 0x00000000U);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_43_OFFSET, 0x00000100U, 0x00000000U);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_48_OFFSET, 0x000007FFU, 0x00000000U);
/*##################################################################### */

    /*
    * Register : ATTR_46 @ 0XFD4800B8

    * MSI-X Table Offset. This value is transferred to the MSI-X Table Offset
    * field. Set to 0 if MSI-X is not enabled.; EP=0x0001; RP=0x0000
    *  PSU_PCIE_ATTRIB_ATTR_46_ATTR_MSIX_CAP_TABLE_OFFSET          0

    * ATTR_46
    * (OFFSET, MASK, VALUE)      (0XFD4800B8, 0x0000FFFFU ,0x00000000U)
    */
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_46_OFFSET, 0x0000FFFFU, 0x00000000U);
/*##################################################################### */

    /*
    * Register : ATTR_47 @ 0XFD4800BC

    * MSI-X Table Offset. This value is transferred to the MSI-X Table Offset
    * field. Set to 0 if MSI-X is not enabled.; EP=0x0000; RP=0x0000
    *  PSU_PCIE_ATTRIB_ATTR_47_ATTR_MSIX_CAP_TABLE_OFFSET          0

    * ATTR_47
    * (OFFSET, MASK, VALUE)      (0XFD4800BC, 0x00001FFFU ,0x00000000U)
    */
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_47_OFFSET, 0x00001FFFU, 0x00000000U);
/*##################################################################### */

    /*
    * Register : ATTR_44 @ 0XFD4800B0

    * MSI-X Pending Bit Array Offset This value is transferred to the MSI-X PB
    * A Offset field. Set to 0 if MSI-X is not enabled.; EP=0x0001; RP=0x0000
    *  PSU_PCIE_ATTRIB_ATTR_44_ATTR_MSIX_CAP_PBA_OFFSET            0

    * ATTR_44
    * (OFFSET, MASK, VALUE)      (0XFD4800B0, 0x0000FFFFU ,0x00000000U)
    */
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_44_OFFSET, 0x0000FFFFU, 0x00000000U);
/*##################################################################### */

    /*
    * Register : ATTR_45 @ 0XFD4800B4

    * MSI-X Pending Bit Array Offset This value is transferred to the MSI-X PB
    * A Offset field. Set to 0 if MSI-X is not enabled.; EP=0x1000; RP=0x0000
    *  PSU_PCIE_ATTRIB_ATTR_45_ATTR_MSIX_CAP_PBA_OFFSET            0

    * ATTR_45
    * (OFFSET, MASK, VALUE)      (0XFD4800B4, 0x0000FFF8U ,0x00000000U)
    */
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_45_OFFSET, 0x0000FFF8U, 0x00000000U);
/*##################################################################### */

    /*
    * Register : CB @ 0XFD48031C

    * DT837748 Enable
    *  PSU_PCIE_ATTRIB_CB_CB1                                      0x0

    * ECO Register 1
    * (OFFSET, MASK, VALUE)      (0XFD48031C, 0x00000002U ,0x00000000U)
    */
	PSU_Mask_Write(PCIE_ATTRIB_CB_OFFSET, 0x00000002U, 0x00000000U);
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
	PSU_Mask_Write(PCIE_ATTRIB_ATTR_35_OFFSET, 0x0000B000U, 0x00008000U);
/*##################################################################### */

    /*
    * PUTTING PCIE CONTROL IN RESET
    */
    /*
    * Register : RST_FPD_TOP @ 0XFD1A0100

    * PCIE control block level reset
    *  PSU_CRF_APB_RST_FPD_TOP_PCIE_CTRL_RESET                     0X0

    * FPD Block level software controlled reset
    * (OFFSET, MASK, VALUE)      (0XFD1A0100, 0x00020000U ,0x00000000U)
    */
	PSU_Mask_Write(CRF_APB_RST_FPD_TOP_OFFSET, 0x00020000U, 0x00000000U);
/*##################################################################### */

    /*
    * PCIE GPIO RESET
    */
    /*
    * MASK_DATA_0_LSW LOW BANK [15:0]
    */
    /*
    * MASK_DATA_0_MSW LOW BANK [25:16]
    */
    /*
    * MASK_DATA_1_LSW LOW BANK [41:26]
    */
    /*
    * Register : MASK_DATA_1_LSW @ 0XFF0A0008

    * Operation is the same as MASK_DATA_0_LSW[MASK_0_LSW]
    *  PSU_GPIO_MASK_DATA_1_LSW_MASK_1_LSW                         0xffef

    * Operation is the same as MASK_DATA_0_LSW[DATA_0_LSW]
    *  PSU_GPIO_MASK_DATA_1_LSW_DATA_1_LSW                         0x10

    * Maskable Output Data (GPIO Bank1, MIO, Lower 16bits)
    * (OFFSET, MASK, VALUE)      (0XFF0A0008, 0xFFFFFFFFU ,0xFFEF0010U)
    */
	//PSU_Mask_Write(GPIO_MASK_DATA_1_LSW_OFFSET,
	//	0xFFFFFFFFU, 0xFFEF0010U);
/*##################################################################### */

    /*
    * MASK_DATA_1_MSW HIGH BANK [51:42]
    */
    /*
    * MASK_DATA_1_LSW HIGH BANK [67:52]
    */
    /*
    * MASK_DATA_1_LSW HIGH BANK [77:68]
    */
    /*
    * CHECK PLL LOCK FOR LANE0
    */
    /*
    * Register : L0_PLL_STATUS_READ_1 @ 0XFD4023E4

    * Status Read value of PLL Lock
    *  PSU_SERDES_L0_PLL_STATUS_READ_1_PLL_LOCK_STATUS_READ        1
    * (OFFSET, MASK, VALUE)      (0XFD4023E4, 0x00000010U ,0x00000010U)
		*/
	mask_poll(SERDES_L0_PLL_STATUS_READ_1_OFFSET, 0x00000010U);

/*##################################################################### */

    /*
    * SATA AHCI VENDOR SETTING
    */

	return 1;
}

unsigned long psu_resetin_init_data(void)
{
    /*
    * PUTTING SERDES PERIPHERAL IN RESET
    */
    /*
    * PUTTING PCIE IN RESET
    */
    /*
    * Register : RST_FPD_TOP @ 0XFD1A0100

    * PCIE config reset
    *  PSU_CRF_APB_RST_FPD_TOP_PCIE_CFG_RESET                      0X1

    * PCIE control block level reset
    *  PSU_CRF_APB_RST_FPD_TOP_PCIE_CTRL_RESET                     0X1

    * PCIE bridge block level reset (AXI interface)
    *  PSU_CRF_APB_RST_FPD_TOP_PCIE_BRIDGE_RESET                   0X1

    * FPD Block level software controlled reset
    * (OFFSET, MASK, VALUE)      (0XFD1A0100, 0x000E0000U ,0x000E0000U)
    */
	PSU_Mask_Write(CRF_APB_RST_FPD_TOP_OFFSET, 0x000E0000U, 0x000E0000U);
/*##################################################################### */


	return 1;
}

//Kishore -- ILL calibration code begins
//ILL calibration code begins
#define SERDES_L0_TM_PLL_DIG_33 		0XFD402084
#define SERDES_L1_TM_PLL_DIG_33 		0XFD406084
#define SERDES_L2_TM_PLL_DIG_33 		0XFD40A084
#define SERDES_L3_TM_PLL_DIG_33 		0XFD40E084

#define SERDES_L0_TM_ANA_BYP_4	 		0XFD401010
#define SERDES_L1_TM_ANA_BYP_4	 		0XFD405010
#define SERDES_L2_TM_ANA_BYP_4	 		0XFD409010
#define SERDES_L3_TM_ANA_BYP_4	 		0XFD40D010

#define SERDES_L0_TM_ANA_BYP_7	 		0XFD401018
#define SERDES_L1_TM_ANA_BYP_7	 		0XFD405018
#define SERDES_L2_TM_ANA_BYP_7	 		0XFD409018
#define SERDES_L3_TM_ANA_BYP_7	 		0XFD40D018

#define SERDES_L0_TM_E_ILL7	 		0XFD40193C
#define SERDES_L1_TM_E_ILL7	 		0XFD40593C
#define SERDES_L2_TM_E_ILL7	 		0XFD40993C
#define SERDES_L3_TM_E_ILL7	 		0XFD40D93C

#define SERDES_L0_TM_IQ_ILL7	 		0XFD401910
#define SERDES_L1_TM_IQ_ILL7	 		0XFD405910
#define SERDES_L2_TM_IQ_ILL7	 		0XFD409910
#define SERDES_L3_TM_IQ_ILL7	 		0XFD40D910

#define SERDES_L0_TX_DIG_TM_61	 		0XFD4000F4
#define SERDES_L1_TX_DIG_TM_61	 		0XFD4040F4
#define SERDES_L2_TX_DIG_TM_61	 		0XFD4080F4
#define SERDES_L3_TX_DIG_TM_61	 		0XFD40C0F4

#define SERDES_L0_TM_DIG_6	 		0XFD40106C
#define SERDES_L1_TM_DIG_6	 		0XFD40506C
#define SERDES_L2_TM_DIG_6	 		0XFD40906C
#define SERDES_L3_TM_DIG_6	 		0XFD40D06C

#define SERDES_L0_TM_IQ_ILL1                    0XFD4018F8
#define SERDES_L0_TM_IQ_ILL2                    0XFD4018FC
#define SERDES_L0_TM_ILL11                      0XFD40198C
#define SERDES_L0_TM_ILL12                      0XFD401990
#define SERDES_L0_TM_E_ILL1                     0XFD401924
#define SERDES_L0_TM_E_ILL2                     0XFD401928
#define SERDES_L0_TM_IQ_ILL3                    0XFD401900
#define SERDES_L0_TM_E_ILL3                     0XFD40192C
#define SERDES_L0_TM_ILL8                       0XFD401980
#define SERDES_L0_TM_IQ_ILL8                    0XFD401914
#define SERDES_L0_TM_IQ_ILL9                    0XFD401918
#define SERDES_L0_TM_E_ILL8                     0XFD401940
#define SERDES_L0_TM_E_ILL9                     0XFD401944
#define SERDES_L0_TM_ILL13                      0XFD401994
#define SERDES_L1_TM_MISC2                      0XFD40589C
#define SERDES_L1_TM_IQ_ILL1                    0XFD4058F8
#define SERDES_L1_TM_IQ_ILL2                    0XFD4058FC
#define SERDES_L1_TM_ILL11                      0XFD40598C
#define SERDES_L1_TM_ILL12                      0XFD405990
#define SERDES_L1_TM_E_ILL1                     0XFD405924
#define SERDES_L1_TM_E_ILL2                     0XFD405928
#define SERDES_L1_TM_IQ_ILL3                    0XFD405900
#define SERDES_L1_TM_E_ILL3                     0XFD40592C
#define SERDES_L1_TM_ILL8                       0XFD405980
#define SERDES_L1_TM_IQ_ILL8                    0XFD405914
#define SERDES_L1_TM_IQ_ILL9                    0XFD405918
#define SERDES_L1_TM_E_ILL8                     0XFD405940
#define SERDES_L1_TM_E_ILL9                     0XFD405944
#define SERDES_L1_TM_ILL13                      0XFD405994
#define SERDES_L2_TM_MISC2                      0XFD40989C
#define SERDES_L2_TM_IQ_ILL1                    0XFD4098F8
#define SERDES_L2_TM_IQ_ILL2                    0XFD4098FC
#define SERDES_L2_TM_ILL11                      0XFD40998C
#define SERDES_L2_TM_ILL12                      0XFD409990
#define SERDES_L2_TM_E_ILL1                     0XFD409924
#define SERDES_L2_TM_E_ILL2                     0XFD409928
#define SERDES_L2_TM_IQ_ILL3                    0XFD409900
#define SERDES_L2_TM_E_ILL3                     0XFD40992C
#define SERDES_L2_TM_ILL8                       0XFD409980
#define SERDES_L2_TM_IQ_ILL8                    0XFD409914
#define SERDES_L2_TM_IQ_ILL9                    0XFD409918
#define SERDES_L2_TM_E_ILL8                     0XFD409940
#define SERDES_L2_TM_E_ILL9                     0XFD409944
#define SERDES_L2_TM_ILL13                      0XFD409994
#define SERDES_L3_TM_MISC2                      0XFD40D89C
#define SERDES_L3_TM_IQ_ILL1                    0XFD40D8F8
#define SERDES_L3_TM_IQ_ILL2                    0XFD40D8FC
#define SERDES_L3_TM_ILL11                      0XFD40D98C
#define SERDES_L3_TM_ILL12                      0XFD40D990
#define SERDES_L3_TM_E_ILL1                     0XFD40D924
#define SERDES_L3_TM_E_ILL2                     0XFD40D928
#define SERDES_L3_TM_IQ_ILL3                    0XFD40D900
#define SERDES_L3_TM_E_ILL3                     0XFD40D92C
#define SERDES_L3_TM_ILL8                       0XFD40D980
#define SERDES_L3_TM_IQ_ILL8                    0XFD40D914
#define SERDES_L3_TM_IQ_ILL9                    0XFD40D918
#define SERDES_L3_TM_E_ILL8                     0XFD40D940
#define SERDES_L3_TM_E_ILL9                     0XFD40D944
#define SERDES_L3_TM_ILL13                      0XFD40D994
#undef SERDES_UPHY_SPARE0
#define SERDES_UPHY_SPARE0                      0XFD410098
#undef SERDES_UPHY_SPARE1
#define SERDES_UPHY_SPARE1                      0XFD41009C
#undef SERDES_UPHY_SPARE2
#define SERDES_UPHY_SPARE2                      0XFD4100A0
#undef SERDES_UPHY_SPARE3
#define SERDES_UPHY_SPARE3                      0XFD4100A4

#define SERDES_L0_PLL_FBDIV_FRAC_3_MSB 		0xFD402360
#define SERDES_L1_PLL_FBDIV_FRAC_3_MSB 		0xFD406360
#define SERDES_L2_PLL_FBDIV_FRAC_3_MSB 		0xFD40A360
#define SERDES_L3_PLL_FBDIV_FRAC_3_MSB 		0xFD40E360

#define SERDES_L0_PLL_STATUS_READ_1             0XFD4023E4
#define SERDES_L0_TM_MISC_ST_0	                0XFD401AC8
#define SERDES_L1_PLL_STATUS_READ_1             0XFD4063E4
#define SERDES_L1_TM_MISC_ST_0	                0XFD405AC8
#define SERDES_L2_PLL_STATUS_READ_1             0XFD40A3E4
#define SERDES_L2_TM_MISC_ST_0	                0XFD409AC8
#define SERDES_L3_PLL_STATUS_READ_1             0XFD40E3E4
#define SERDES_L3_TM_MISC_ST_0	                0XFD40DAC8

#define SERDES_L0_BIST_CTRL_1     		0xFD403004
#define SERDES_L0_BIST_CTRL_2     		0xFD403008
#define SERDES_L0_BIST_RUN_LEN_L     		0xFD40300C
#define SERDES_L0_BIST_ERR_INJ_POINT_L     	0xFD403010
#define SERDES_L0_BIST_RUNLEN_ERR_INJ_H     	0xFD403014
#define SERDES_L0_BIST_IDLE_TIME     		0xFD403018
#define SERDES_L0_BIST_MARKER_L     		0xFD40301C
#define SERDES_L0_BIST_IDLE_CHAR_L     		0xFD403020
#define SERDES_L0_BIST_MARKER_IDLE_H     	0xFD403024
#define SERDES_L0_BIST_LOW_PULSE_TIME     	0xFD403028
#define SERDES_L0_BIST_TOTAL_PULSE_TIME     	0xFD40302C
#define SERDES_L0_BIST_TEST_PAT_1     		0xFD403030
#define SERDES_L0_BIST_TEST_PAT_2     		0xFD403034
#define SERDES_L0_BIST_TEST_PAT_3     		0xFD403038
#define SERDES_L0_BIST_TEST_PAT_4     		0xFD40303C
#define SERDES_L0_BIST_TEST_PAT_MSBS     	0xFD403040
#define SERDES_L0_BIST_PKT_NUM     		0xFD403044
#define SERDES_L0_BIST_FRM_IDLE_TIME     	0xFD403048
#define SERDES_L0_BIST_PKT_CTR_L     		0xFD40304C
#define SERDES_L0_BIST_PKT_CTR_H     		0xFD403050
#define SERDES_L0_BIST_ERR_CTR_L     		0xFD403054
#define SERDES_L0_BIST_ERR_CTR_H     		0xFD403058
#define SERDES_L0_BIST_FILLER_OUT     		0xFD403068
#define SERDES_L0_BIST_FORCE_MK_RST     	0xFD40306C

#define SERDES_L1_BIST_CTRL_1     		0xFD407004
#define SERDES_L1_BIST_CTRL_2     		0xFD407008
#define SERDES_L1_BIST_RUN_LEN_L     		0xFD40700C
#define SERDES_L1_BIST_ERR_INJ_POINT_L     	0xFD407010
#define SERDES_L1_BIST_RUNLEN_ERR_INJ_H     	0xFD407014
#define SERDES_L1_BIST_IDLE_TIME     		0xFD407018
#define SERDES_L1_BIST_MARKER_L     		0xFD40701C
#define SERDES_L1_BIST_IDLE_CHAR_L     		0xFD407020
#define SERDES_L1_BIST_MARKER_IDLE_H     	0xFD407024
#define SERDES_L1_BIST_LOW_PULSE_TIME     	0xFD407028
#define SERDES_L1_BIST_TOTAL_PULSE_TIME     	0xFD40702C
#define SERDES_L1_BIST_TEST_PAT_1     		0xFD407030
#define SERDES_L1_BIST_TEST_PAT_2     		0xFD407034
#define SERDES_L1_BIST_TEST_PAT_3     		0xFD407038
#define SERDES_L1_BIST_TEST_PAT_4     		0xFD40703C
#define SERDES_L1_BIST_TEST_PAT_MSBS     	0xFD407040
#define SERDES_L1_BIST_PKT_NUM     		0xFD407044
#define SERDES_L1_BIST_FRM_IDLE_TIME     	0xFD407048
#define SERDES_L1_BIST_PKT_CTR_L     		0xFD40704C
#define SERDES_L1_BIST_PKT_CTR_H     		0xFD407050
#define SERDES_L1_BIST_ERR_CTR_L     		0xFD407054
#define SERDES_L1_BIST_ERR_CTR_H     		0xFD407058
#define SERDES_L1_BIST_FILLER_OUT     		0xFD407068
#define SERDES_L1_BIST_FORCE_MK_RST     	0xFD40706C

#define SERDES_L2_BIST_CTRL_1     		0xFD40B004
#define SERDES_L2_BIST_CTRL_2     		0xFD40B008
#define SERDES_L2_BIST_RUN_LEN_L     		0xFD40B00C
#define SERDES_L2_BIST_ERR_INJ_POINT_L     	0xFD40B010
#define SERDES_L2_BIST_RUNLEN_ERR_INJ_H     	0xFD40B014
#define SERDES_L2_BIST_IDLE_TIME     		0xFD40B018
#define SERDES_L2_BIST_MARKER_L     		0xFD40B01C
#define SERDES_L2_BIST_IDLE_CHAR_L     		0xFD40B020
#define SERDES_L2_BIST_MARKER_IDLE_H     	0xFD40B024
#define SERDES_L2_BIST_LOW_PULSE_TIME     	0xFD40B028
#define SERDES_L2_BIST_TOTAL_PULSE_TIME     	0xFD40B02C
#define SERDES_L2_BIST_TEST_PAT_1     		0xFD40B030
#define SERDES_L2_BIST_TEST_PAT_2     		0xFD40B034
#define SERDES_L2_BIST_TEST_PAT_3     		0xFD40B038
#define SERDES_L2_BIST_TEST_PAT_4     		0xFD40B03C
#define SERDES_L2_BIST_TEST_PAT_MSBS     	0xFD40B040
#define SERDES_L2_BIST_PKT_NUM     		0xFD40B044
#define SERDES_L2_BIST_FRM_IDLE_TIME     	0xFD40B048
#define SERDES_L2_BIST_PKT_CTR_L     		0xFD40B04C
#define SERDES_L2_BIST_PKT_CTR_H     		0xFD40B050
#define SERDES_L2_BIST_ERR_CTR_L     		0xFD40B054
#define SERDES_L2_BIST_ERR_CTR_H     		0xFD40B058
#define SERDES_L2_BIST_FILLER_OUT     		0xFD40B068
#define SERDES_L2_BIST_FORCE_MK_RST     	0xFD40B06C

#define SERDES_L3_BIST_CTRL_1     		0xFD40F004
#define SERDES_L3_BIST_CTRL_2     		0xFD40F008
#define SERDES_L3_BIST_RUN_LEN_L     		0xFD40F00C
#define SERDES_L3_BIST_ERR_INJ_POINT_L     	0xFD40F010
#define SERDES_L3_BIST_RUNLEN_ERR_INJ_H     	0xFD40F014
#define SERDES_L3_BIST_IDLE_TIME     		0xFD40F018
#define SERDES_L3_BIST_MARKER_L     		0xFD40F01C
#define SERDES_L3_BIST_IDLE_CHAR_L     		0xFD40F020
#define SERDES_L3_BIST_MARKER_IDLE_H     	0xFD40F024
#define SERDES_L3_BIST_LOW_PULSE_TIME     	0xFD40F028
#define SERDES_L3_BIST_TOTAL_PULSE_TIME     	0xFD40F02C
#define SERDES_L3_BIST_TEST_PAT_1     		0xFD40F030
#define SERDES_L3_BIST_TEST_PAT_2     		0xFD40F034
#define SERDES_L3_BIST_TEST_PAT_3     		0xFD40F038
#define SERDES_L3_BIST_TEST_PAT_4     		0xFD40F03C
#define SERDES_L3_BIST_TEST_PAT_MSBS     	0xFD40F040
#define SERDES_L3_BIST_PKT_NUM     		0xFD40F044
#define SERDES_L3_BIST_FRM_IDLE_TIME     	0xFD40F048
#define SERDES_L3_BIST_PKT_CTR_L     		0xFD40F04C
#define SERDES_L3_BIST_PKT_CTR_H     		0xFD40F050
#define SERDES_L3_BIST_ERR_CTR_L     		0xFD40F054
#define SERDES_L3_BIST_ERR_CTR_H     		0xFD40F058
#define SERDES_L3_BIST_FILLER_OUT     		0xFD40F068
#define SERDES_L3_BIST_FORCE_MK_RST     	0xFD40F06C

#define SERDES_TX_PROT_BUS_WIDTH     		0xFD410040
#define SERDES_RX_PROT_BUS_WIDTH     		0xFD410044
#define SERDES_LPBK_CTRL0     			0xFD410038
#define SERDES_LPBK_CTRL1     			0xFD41003C
#define SERDES_L0_TM_DIG_22    			0xFD4010AC
#define SERDES_L1_TM_DIG_22    			0xFD4050AC
#define SERDES_L2_TM_DIG_22    			0xFD4090AC
#define SERDES_L3_TM_DIG_22    			0xFD40D0AC
#define SERDES_L0_DATA_BUS_WID     		0xFD403060
#define SERDES_L1_DATA_BUS_WID     		0xFD407060
#define SERDES_L2_DATA_BUS_WID     		0xFD40B060
#define SERDES_L3_DATA_BUS_WID     		0xFD40F060
#define SERDES_L0_TX_ANA_TM_3     		0XFD40000C
#define SERDES_L1_TX_ANA_TM_3     		0XFD40400C
#define SERDES_L2_TX_ANA_TM_3     		0XFD40800C
#define SERDES_L3_TX_ANA_TM_3     		0XFD40C00C

#undef SERDES_PLL_REF_SEL0_OFFSET
#define SERDES_PLL_REF_SEL0_OFFSET  		0xFD410000
#undef SERDES_PLL_REF_SEL1_OFFSET
#define SERDES_PLL_REF_SEL1_OFFSET  		0xFD410004
#undef SERDES_PLL_REF_SEL2_OFFSET
#define SERDES_PLL_REF_SEL2_OFFSET  		0xFD410008
#undef SERDES_PLL_REF_SEL3_OFFSET
#define SERDES_PLL_REF_SEL3_OFFSET  		0xFD41000C
#undef SERDES_ICM_CFG0_OFFSET
#define SERDES_ICM_CFG0_OFFSET     		0xFD410010
#undef SERDES_ICM_CFG1_OFFSET
#define SERDES_ICM_CFG1_OFFSET     		0xFD410014

static int serdes_rst_seq (u32 pllsel, u32 lane3_protocol, u32 lane3_rate, u32 lane2_protocol, u32 lane2_rate, u32 lane1_protocol, u32 lane1_rate, u32 lane0_protocol, u32 lane0_rate)
{
   Xil_Out32(SERDES_UPHY_SPARE0, 0x00000000);
   Xil_Out32(SERDES_L0_TM_ANA_BYP_4, 0x00000040);
   Xil_Out32(SERDES_L1_TM_ANA_BYP_4, 0x00000040);
   Xil_Out32(SERDES_L2_TM_ANA_BYP_4, 0x00000040);
   Xil_Out32(SERDES_L3_TM_ANA_BYP_4, 0x00000040);
   Xil_Out32(SERDES_L0_TM_PLL_DIG_33, 0x00000080);
   Xil_Out32(SERDES_L1_TM_PLL_DIG_33, 0x00000080);
   Xil_Out32(SERDES_L2_TM_PLL_DIG_33, 0x00000080);
   Xil_Out32(SERDES_L3_TM_PLL_DIG_33, 0x00000080);
   Xil_Out32(SERDES_UPHY_SPARE0, 0x00000004);
   mask_delay(50);
   if (lane0_rate == 1) Xil_Out32(SERDES_UPHY_SPARE0, 0x0000000E);
   Xil_Out32(SERDES_UPHY_SPARE0, 0x00000006);
   if (lane0_rate == 1) {
      Xil_Out32(SERDES_L0_TX_ANA_TM_3, 0x00000004);
      Xil_Out32(SERDES_L1_TX_ANA_TM_3, 0x00000004);
      Xil_Out32(SERDES_L2_TX_ANA_TM_3, 0x00000004);
      Xil_Out32(SERDES_L3_TX_ANA_TM_3, 0x00000004);
      Xil_Out32(SERDES_UPHY_SPARE0, 0x00000007);
      mask_delay (400);
      Xil_Out32(SERDES_L0_TX_ANA_TM_3, 0x0000000C);
      Xil_Out32(SERDES_L1_TX_ANA_TM_3, 0x0000000C);
      Xil_Out32(SERDES_L2_TX_ANA_TM_3, 0x0000000C);
      Xil_Out32(SERDES_L3_TX_ANA_TM_3, 0x0000000C);
      mask_delay (15);
      Xil_Out32(SERDES_UPHY_SPARE0, 0x0000000F);
      mask_delay (100);
   }
   if (pllsel == 0) mask_poll(SERDES_L0_PLL_STATUS_READ_1, 0x00000010U);
   if (pllsel == 1) mask_poll(SERDES_L1_PLL_STATUS_READ_1, 0x00000010U);
   if (pllsel == 2) mask_poll(SERDES_L2_PLL_STATUS_READ_1, 0x00000010U);
   if (pllsel == 3) mask_poll(SERDES_L3_PLL_STATUS_READ_1, 0x00000010U);
   mask_delay(50);
   Xil_Out32(SERDES_L0_TM_ANA_BYP_4, 0x000000C0);
   Xil_Out32(SERDES_L1_TM_ANA_BYP_4, 0x000000C0);
   Xil_Out32(SERDES_L2_TM_ANA_BYP_4, 0x000000C0);
   Xil_Out32(SERDES_L3_TM_ANA_BYP_4, 0x000000C0);
   Xil_Out32(SERDES_L0_TM_ANA_BYP_4, 0x00000080);
   Xil_Out32(SERDES_L1_TM_ANA_BYP_4, 0x00000080);
   Xil_Out32(SERDES_L2_TM_ANA_BYP_4, 0x00000080);
   Xil_Out32(SERDES_L3_TM_ANA_BYP_4, 0x00000080);

   Xil_Out32(SERDES_L0_TM_PLL_DIG_33 , 0x000000C0);
   Xil_Out32(SERDES_L1_TM_PLL_DIG_33 , 0x000000C0);
   Xil_Out32(SERDES_L2_TM_PLL_DIG_33 , 0x000000C0);
   Xil_Out32(SERDES_L3_TM_PLL_DIG_33 , 0x000000C0);
   mask_delay(50);
   Xil_Out32(SERDES_L0_TM_PLL_DIG_33 , 0x00000080);
   Xil_Out32(SERDES_L1_TM_PLL_DIG_33 , 0x00000080);
   Xil_Out32(SERDES_L2_TM_PLL_DIG_33 , 0x00000080);
   Xil_Out32(SERDES_L3_TM_PLL_DIG_33 , 0x00000080);
   mask_delay(50);
   Xil_Out32(SERDES_L0_TM_ANA_BYP_4, 0x00000000);
   Xil_Out32(SERDES_L1_TM_ANA_BYP_4, 0x00000000);
   Xil_Out32(SERDES_L2_TM_ANA_BYP_4, 0x00000000);
   Xil_Out32(SERDES_L3_TM_ANA_BYP_4, 0x00000000);
   Xil_Out32(SERDES_L0_TM_PLL_DIG_33 , 0x00000000);
   Xil_Out32(SERDES_L1_TM_PLL_DIG_33 , 0x00000000);
   Xil_Out32(SERDES_L2_TM_PLL_DIG_33 , 0x00000000);
   Xil_Out32(SERDES_L3_TM_PLL_DIG_33 , 0x00000000);
   mask_delay(500);
   return 1;
}


static int serdes_bist_static_settings(u32 lane_active)
{
   if (lane_active == 0)
   {
      Xil_Out32(SERDES_L0_BIST_CTRL_1, (Xil_In32(SERDES_L0_BIST_CTRL_1) & 0xFFFFFF1F));
      Xil_Out32(SERDES_L0_BIST_FILLER_OUT, 0x1 );
      Xil_Out32(SERDES_L0_BIST_FORCE_MK_RST, 0x1 );
      Xil_Out32(SERDES_L0_TM_DIG_22, 0x0020);
      Xil_Out32(SERDES_L0_BIST_CTRL_2, 0x0);
      Xil_Out32(SERDES_L0_BIST_RUN_LEN_L, 0xF4);
      Xil_Out32(SERDES_L0_BIST_ERR_INJ_POINT_L, 0x0);
      Xil_Out32(SERDES_L0_BIST_RUNLEN_ERR_INJ_H, 0x0);
      Xil_Out32(SERDES_L0_BIST_IDLE_TIME,0x00);
      Xil_Out32(SERDES_L0_BIST_MARKER_L, 0xFB);
      Xil_Out32(SERDES_L0_BIST_IDLE_CHAR_L, 0xFF);
      Xil_Out32(SERDES_L0_BIST_MARKER_IDLE_H, 0x0);
      Xil_Out32(SERDES_L0_BIST_LOW_PULSE_TIME, 0x00);
      Xil_Out32(SERDES_L0_BIST_TOTAL_PULSE_TIME, 0x00);
      Xil_Out32(SERDES_L0_BIST_TEST_PAT_1, 0x4A);
      Xil_Out32(SERDES_L0_BIST_TEST_PAT_2, 0x4A);
      Xil_Out32(SERDES_L0_BIST_TEST_PAT_3, 0x4A);
      Xil_Out32(SERDES_L0_BIST_TEST_PAT_4, 0x4A);
      Xil_Out32(SERDES_L0_BIST_TEST_PAT_MSBS, 0x0);
      Xil_Out32(SERDES_L0_BIST_PKT_NUM, 0x14);
      Xil_Out32(SERDES_L0_BIST_FRM_IDLE_TIME,0x02);
      Xil_Out32(SERDES_L0_BIST_CTRL_1, (Xil_In32(SERDES_L0_BIST_CTRL_1) & 0xFFFFFF1F));
   }
   if (lane_active == 1)
   {
      Xil_Out32(SERDES_L1_BIST_CTRL_1, (Xil_In32(SERDES_L1_BIST_CTRL_1) & 0xFFFFFF1F));
      Xil_Out32(SERDES_L1_BIST_FILLER_OUT, 0x1 );
      Xil_Out32(SERDES_L1_BIST_FORCE_MK_RST, 0x1 );
      Xil_Out32(SERDES_L1_TM_DIG_22, 0x0020);
      Xil_Out32(SERDES_L1_BIST_CTRL_2, 0x0);
      Xil_Out32(SERDES_L1_BIST_RUN_LEN_L, 0xF4);
      Xil_Out32(SERDES_L1_BIST_ERR_INJ_POINT_L, 0x0);
      Xil_Out32(SERDES_L1_BIST_RUNLEN_ERR_INJ_H, 0x0);
      Xil_Out32(SERDES_L1_BIST_IDLE_TIME,0x00);
      Xil_Out32(SERDES_L1_BIST_MARKER_L, 0xFB);
      Xil_Out32(SERDES_L1_BIST_IDLE_CHAR_L, 0xFF);
      Xil_Out32(SERDES_L1_BIST_MARKER_IDLE_H, 0x0);
      Xil_Out32(SERDES_L1_BIST_LOW_PULSE_TIME, 0x00);
      Xil_Out32(SERDES_L1_BIST_TOTAL_PULSE_TIME, 0x00);
      Xil_Out32(SERDES_L1_BIST_TEST_PAT_1, 0x4A);
      Xil_Out32(SERDES_L1_BIST_TEST_PAT_2, 0x4A);
      Xil_Out32(SERDES_L1_BIST_TEST_PAT_3, 0x4A);
      Xil_Out32(SERDES_L1_BIST_TEST_PAT_4, 0x4A);
      Xil_Out32(SERDES_L1_BIST_TEST_PAT_MSBS, 0x0);
      Xil_Out32(SERDES_L1_BIST_PKT_NUM, 0x14);
      Xil_Out32(SERDES_L1_BIST_FRM_IDLE_TIME,0x02);
      Xil_Out32(SERDES_L1_BIST_CTRL_1, (Xil_In32(SERDES_L1_BIST_CTRL_1) & 0xFFFFFF1F));
   }

   if (lane_active == 2)
   {
      Xil_Out32(SERDES_L2_BIST_CTRL_1, (Xil_In32(SERDES_L2_BIST_CTRL_1) & 0xFFFFFF1F));
      Xil_Out32(SERDES_L2_BIST_FILLER_OUT, 0x1 );
      Xil_Out32(SERDES_L2_BIST_FORCE_MK_RST, 0x1 );
      Xil_Out32(SERDES_L2_TM_DIG_22, 0x0020);
      Xil_Out32(SERDES_L2_BIST_CTRL_2, 0x0);
      Xil_Out32(SERDES_L2_BIST_RUN_LEN_L, 0xF4);
      Xil_Out32(SERDES_L2_BIST_ERR_INJ_POINT_L, 0x0);
      Xil_Out32(SERDES_L2_BIST_RUNLEN_ERR_INJ_H, 0x0);
      Xil_Out32(SERDES_L2_BIST_IDLE_TIME,0x00);
      Xil_Out32(SERDES_L2_BIST_MARKER_L, 0xFB);
      Xil_Out32(SERDES_L2_BIST_IDLE_CHAR_L, 0xFF);
      Xil_Out32(SERDES_L2_BIST_MARKER_IDLE_H, 0x0);
      Xil_Out32(SERDES_L2_BIST_LOW_PULSE_TIME, 0x00);
      Xil_Out32(SERDES_L2_BIST_TOTAL_PULSE_TIME, 0x00);
      Xil_Out32(SERDES_L2_BIST_TEST_PAT_1, 0x4A);
      Xil_Out32(SERDES_L2_BIST_TEST_PAT_2, 0x4A);
      Xil_Out32(SERDES_L2_BIST_TEST_PAT_3, 0x4A);
      Xil_Out32(SERDES_L2_BIST_TEST_PAT_4, 0x4A);
      Xil_Out32(SERDES_L2_BIST_TEST_PAT_MSBS, 0x0);
      Xil_Out32(SERDES_L2_BIST_PKT_NUM, 0x14);
      Xil_Out32(SERDES_L2_BIST_FRM_IDLE_TIME,0x02);
      Xil_Out32(SERDES_L2_BIST_CTRL_1, (Xil_In32(SERDES_L2_BIST_CTRL_1) & 0xFFFFFF1F));
   }

   if (lane_active == 3)
   {
      Xil_Out32(SERDES_L3_BIST_CTRL_1, (Xil_In32(SERDES_L3_BIST_CTRL_1) & 0xFFFFFF1F));
      Xil_Out32(SERDES_L3_BIST_FILLER_OUT, 0x1 );
      Xil_Out32(SERDES_L3_BIST_FORCE_MK_RST, 0x1 );
      Xil_Out32(SERDES_L3_TM_DIG_22, 0x0020);
      Xil_Out32(SERDES_L3_BIST_CTRL_2, 0x0);
      Xil_Out32(SERDES_L3_BIST_RUN_LEN_L, 0xF4);
      Xil_Out32(SERDES_L3_BIST_ERR_INJ_POINT_L, 0x0);
      Xil_Out32(SERDES_L3_BIST_RUNLEN_ERR_INJ_H, 0x0);
      Xil_Out32(SERDES_L3_BIST_IDLE_TIME,0x00);
      Xil_Out32(SERDES_L3_BIST_MARKER_L, 0xFB);
      Xil_Out32(SERDES_L3_BIST_IDLE_CHAR_L, 0xFF);
      Xil_Out32(SERDES_L3_BIST_MARKER_IDLE_H, 0x0);
      Xil_Out32(SERDES_L3_BIST_LOW_PULSE_TIME, 0x00);
      Xil_Out32(SERDES_L3_BIST_TOTAL_PULSE_TIME, 0x00);
      Xil_Out32(SERDES_L3_BIST_TEST_PAT_1, 0x4A);
      Xil_Out32(SERDES_L3_BIST_TEST_PAT_2, 0x4A);
      Xil_Out32(SERDES_L3_BIST_TEST_PAT_3, 0x4A);
      Xil_Out32(SERDES_L3_BIST_TEST_PAT_4, 0x4A);
      Xil_Out32(SERDES_L3_BIST_TEST_PAT_MSBS, 0x0);
      Xil_Out32(SERDES_L3_BIST_PKT_NUM, 0x14);
      Xil_Out32(SERDES_L3_BIST_FRM_IDLE_TIME,0x02);
      Xil_Out32(SERDES_L3_BIST_CTRL_1, (Xil_In32(SERDES_L3_BIST_CTRL_1) & 0xFFFFFF1F));
   }
   return (1);
}

static int serdes_bist_run(u32 lane_active)
{
   if (lane_active == 0) {
     PSU_Mask_Write(SERDES_RX_PROT_BUS_WIDTH, 0x00000003U, 0x00000000U);
     PSU_Mask_Write(SERDES_TX_PROT_BUS_WIDTH, 0x00000003U, 0x00000000U);
     PSU_Mask_Write(SERDES_LPBK_CTRL0, 0x00000007U, 0x00000001U);
     Xil_Out32(SERDES_L0_TM_DIG_22, 0x0020);
     Xil_Out32(SERDES_L0_BIST_CTRL_1,(Xil_In32(SERDES_L0_BIST_CTRL_1) | 0x1));
   }
   if (lane_active == 1) {
     PSU_Mask_Write(SERDES_RX_PROT_BUS_WIDTH, 0x0000000CU, 0x00000000U);
     PSU_Mask_Write(SERDES_TX_PROT_BUS_WIDTH, 0x0000000CU, 0x00000000U);
     PSU_Mask_Write(SERDES_LPBK_CTRL0, 0x00000070U, 0x00000010U);
     Xil_Out32(SERDES_L1_TM_DIG_22, 0x0020);
     Xil_Out32(SERDES_L1_BIST_CTRL_1,(Xil_In32(SERDES_L1_BIST_CTRL_1) | 0x1));
   }
   if (lane_active == 2) {
     PSU_Mask_Write(SERDES_RX_PROT_BUS_WIDTH, 0x00000030U, 0x00000000U);
     PSU_Mask_Write(SERDES_TX_PROT_BUS_WIDTH, 0x00000030U, 0x00000000U);
     PSU_Mask_Write(SERDES_LPBK_CTRL1, 0x00000007U, 0x00000001U);
     Xil_Out32(SERDES_L2_TM_DIG_22, 0x0020);
     Xil_Out32(SERDES_L2_BIST_CTRL_1,(Xil_In32(SERDES_L2_BIST_CTRL_1) | 0x1));
   }
   if (lane_active == 3)  {
     PSU_Mask_Write(SERDES_TX_PROT_BUS_WIDTH, 0x000000C0U, 0x00000000U);
     PSU_Mask_Write(SERDES_RX_PROT_BUS_WIDTH, 0x000000C0U, 0x00000000U);
     PSU_Mask_Write(SERDES_LPBK_CTRL1, 0x00000070U, 0x00000010U);
     Xil_Out32(SERDES_L3_TM_DIG_22, 0x0020);
     Xil_Out32(SERDES_L3_BIST_CTRL_1,(Xil_In32(SERDES_L3_BIST_CTRL_1) | 0x1));
   }
   mask_delay(100);
   return (1);
}

static int serdes_bist_result(u32 lane_active)
{
   u32 pkt_cnt_l0, pkt_cnt_h0, err_cnt_l0, err_cnt_h0;
   if (lane_active == 0) {
      pkt_cnt_l0 = Xil_In32(SERDES_L0_BIST_PKT_CTR_L);
      pkt_cnt_h0 = Xil_In32(SERDES_L0_BIST_PKT_CTR_H);
      err_cnt_l0 = Xil_In32(SERDES_L0_BIST_ERR_CTR_L);
      err_cnt_h0 = Xil_In32(SERDES_L0_BIST_ERR_CTR_H);
   }
   if (lane_active == 1) {
      pkt_cnt_l0 = Xil_In32(SERDES_L1_BIST_PKT_CTR_L);
      pkt_cnt_h0 = Xil_In32(SERDES_L1_BIST_PKT_CTR_H);
      err_cnt_l0 = Xil_In32(SERDES_L1_BIST_ERR_CTR_L);
      err_cnt_h0 = Xil_In32(SERDES_L1_BIST_ERR_CTR_H);
   }
   if (lane_active == 2) {
      pkt_cnt_l0 = Xil_In32(SERDES_L2_BIST_PKT_CTR_L);
      pkt_cnt_h0 = Xil_In32(SERDES_L2_BIST_PKT_CTR_H);
      err_cnt_l0 = Xil_In32(SERDES_L2_BIST_ERR_CTR_L);
      err_cnt_h0 = Xil_In32(SERDES_L2_BIST_ERR_CTR_H);
   }
   if (lane_active == 3) {
      pkt_cnt_l0 = Xil_In32(SERDES_L3_BIST_PKT_CTR_L);
      pkt_cnt_h0 = Xil_In32(SERDES_L3_BIST_PKT_CTR_H);
      err_cnt_l0 = Xil_In32(SERDES_L3_BIST_ERR_CTR_L);
      err_cnt_h0 = Xil_In32(SERDES_L3_BIST_ERR_CTR_H);
   }
   if (lane_active == 0) Xil_Out32(SERDES_L0_BIST_CTRL_1,0x0);
   if (lane_active == 1) Xil_Out32(SERDES_L1_BIST_CTRL_1,0x0);
   if (lane_active == 2) Xil_Out32(SERDES_L2_BIST_CTRL_1,0x0);
   if (lane_active == 3) Xil_Out32(SERDES_L3_BIST_CTRL_1,0x0);
   if((err_cnt_l0 > 0) || (err_cnt_h0 > 0) || ((pkt_cnt_l0 == 0) && (pkt_cnt_h0 == 0)))
     return (0);
   return (1);
}

static int serdes_illcalib_pcie_gen1 (u32 pllsel, u32 lane3_protocol, u32 lane3_rate, u32 lane2_protocol, u32 lane2_rate, u32 lane1_protocol, u32 lane1_rate, u32 lane0_protocol, u32 lane0_rate, u32 gen2_calib)
{
	u64 tempbistresult;
	u32 currbistresult[4];
	u32 prevbistresult[4];
        u32 itercount = 0;
        u32 ill12_val[4], ill1_val[4];
        u32 loop=0;
        u32 iterresult[8];
        u32 meancount[4];
        u32 bistpasscount[4];
        u32 meancountalt[4];
        u32 meancountalt_bistpasscount[4];
        u32 lane0_active;
        u32 lane1_active;
        u32 lane2_active;
        u32 lane3_active;

        lane0_active = (lane0_protocol == 1);
        lane1_active = (lane1_protocol == 1);
        lane2_active = (lane2_protocol == 1);
        lane3_active = (lane3_protocol == 1);
        for (loop=0; loop<=3; loop++)
        {
          iterresult[loop] = 0;
          iterresult[loop+4] = 0;
          meancountalt[loop] = 0;
          meancountalt_bistpasscount[loop]=0;
          meancount[loop] = 0;
          prevbistresult[loop] = 0;
          bistpasscount[loop] = 0;
        }
        itercount = 0;
        if (lane0_active) serdes_bist_static_settings(0);
        if (lane1_active) serdes_bist_static_settings(1);
        if (lane2_active) serdes_bist_static_settings(2);
        if (lane3_active) serdes_bist_static_settings(3);
        do
        {
          if (gen2_calib != 1)
          {
            if (lane0_active == 1) ill1_val[0] =  ((0x04 + itercount*8) % 0x100);
            if (lane0_active == 1) ill12_val[0] = ((0x04 + itercount*8) >= 0x100) ? 0x10 : 0x00;
            if (lane1_active == 1) ill1_val[1] =  ((0x04 + itercount*8) % 0x100);
            if (lane1_active == 1) ill12_val[1] = ((0x04 + itercount*8) >= 0x100) ? 0x10 : 0x00;
            if (lane2_active == 1) ill1_val[2] =  ((0x04 + itercount*8) % 0x100);
            if (lane2_active == 1) ill12_val[2] = ((0x04 + itercount*8) >= 0x100) ? 0x10 : 0x00;
            if (lane3_active == 1) ill1_val[3] =  ((0x04 + itercount*8) % 0x100);
            if (lane3_active == 1) ill12_val[3] = ((0x04 + itercount*8) >= 0x100) ? 0x10 : 0x00;

            if (lane0_active == 1) Xil_Out32(SERDES_L0_TM_E_ILL1,ill1_val[0]);
            if (lane0_active == 1) PSU_Mask_Write(SERDES_L0_TM_ILL12, 0x000000F0U, ill12_val[0]);
            if (lane1_active == 1) Xil_Out32(SERDES_L1_TM_E_ILL1,ill1_val[1]);
            if (lane1_active == 1) PSU_Mask_Write(SERDES_L1_TM_ILL12, 0x000000F0U, ill12_val[1]);
            if (lane2_active == 1) Xil_Out32(SERDES_L2_TM_E_ILL1,ill1_val[2]);
            if (lane2_active == 1) PSU_Mask_Write(SERDES_L2_TM_ILL12, 0x000000F0U, ill12_val[2]);
            if (lane3_active == 1) Xil_Out32(SERDES_L3_TM_E_ILL1,ill1_val[3]);
            if (lane3_active == 1) PSU_Mask_Write(SERDES_L3_TM_ILL12, 0x000000F0U, ill12_val[3]);
          }
          if (gen2_calib == 1)
          {
            if (lane0_active == 1) ill1_val[0] = ((0x104 + itercount*8) % 0x100);
            if (lane0_active == 1) ill12_val[0] = ((0x104 + itercount*8) >= 0x200) ? 0x02 : 0x01;
            if (lane1_active == 1) ill1_val[1] = ((0x104 + itercount*8) % 0x100);
            if (lane1_active == 1) ill12_val[1] = ((0x104 + itercount*8) >= 0x200) ? 0x02 : 0x01;
            if (lane2_active == 1) ill1_val[2] = ((0x104 + itercount*8) % 0x100);
            if (lane2_active == 1) ill12_val[2] = ((0x104 + itercount*8) >= 0x200) ? 0x02 : 0x01;
            if (lane3_active == 1) ill1_val[3] = ((0x104 + itercount*8) % 0x100);
            if (lane3_active == 1) ill12_val[3] = ((0x104 + itercount*8) >= 0x200) ? 0x02 : 0x01;

            if (lane0_active == 1) Xil_Out32(SERDES_L0_TM_E_ILL2,ill1_val[0]);
            if (lane0_active == 1) PSU_Mask_Write(SERDES_L0_TM_ILL12, 0x0000000FU, ill12_val[0]);
            if (lane1_active == 1) Xil_Out32(SERDES_L1_TM_E_ILL2,ill1_val[1]);
            if (lane1_active == 1) PSU_Mask_Write(SERDES_L1_TM_ILL12, 0x0000000FU, ill12_val[1]);
            if (lane2_active == 1) Xil_Out32(SERDES_L2_TM_E_ILL2,ill1_val[2]);
            if (lane2_active == 1) PSU_Mask_Write(SERDES_L2_TM_ILL12, 0x0000000FU, ill12_val[2]);
            if (lane3_active == 1) Xil_Out32(SERDES_L3_TM_E_ILL2,ill1_val[3]);
            if (lane3_active == 1) PSU_Mask_Write(SERDES_L3_TM_ILL12, 0x0000000FU, ill12_val[3]);
          }

	  if (lane0_active == 1) PSU_Mask_Write(SERDES_L0_TM_ANA_BYP_7, 0x00000030U, 0x00000010U);
	  if (lane1_active == 1) PSU_Mask_Write(SERDES_L1_TM_ANA_BYP_7, 0x00000030U, 0x00000010U);
	  if (lane2_active == 1) PSU_Mask_Write(SERDES_L2_TM_ANA_BYP_7, 0x00000030U, 0x00000010U);
	  if (lane3_active == 1) PSU_Mask_Write(SERDES_L3_TM_ANA_BYP_7, 0x00000030U, 0x00000010U);
          if (lane0_active == 1) currbistresult[0] = 0;
          if (lane1_active == 1) currbistresult[1] = 0;
          if (lane2_active == 1) currbistresult[2] = 0;
          if (lane3_active == 1) currbistresult[3] = 0;
          serdes_rst_seq (pllsel, lane3_protocol, lane3_rate, lane2_protocol, lane2_rate, lane1_protocol, lane1_rate, lane0_protocol, lane0_rate);
          if (lane3_active == 1) serdes_bist_run(3);
          if (lane2_active == 1) serdes_bist_run(2);
          if (lane1_active == 1) serdes_bist_run(1);
          if (lane0_active == 1) serdes_bist_run(0);
          tempbistresult = 0;
          if (lane3_active == 1) tempbistresult = tempbistresult | serdes_bist_result(3);
          tempbistresult = tempbistresult << 1;
          if (lane2_active == 1) tempbistresult = tempbistresult | serdes_bist_result(2);
          tempbistresult = tempbistresult << 1;
          if (lane1_active == 1) tempbistresult = tempbistresult | serdes_bist_result(1);
          tempbistresult = tempbistresult << 1;
          if (lane0_active == 1) tempbistresult = tempbistresult | serdes_bist_result(0);
          Xil_Out32(SERDES_UPHY_SPARE0, 0x0);
          Xil_Out32(SERDES_UPHY_SPARE0, 0x2);

          if (itercount < 32) {
             iterresult[0] = ((iterresult[0]<<1) | ((tempbistresult&0x1)==0x1));
             iterresult[1] = ((iterresult[1]<<1) | ((tempbistresult&0x2)==0x2));
             iterresult[2] = ((iterresult[2]<<1) | ((tempbistresult&0x4)==0x4));
             iterresult[3] = ((iterresult[3]<<1) | ((tempbistresult&0x8)==0x8));
          } else {
             iterresult[4] = ((iterresult[4]<<1) | ((tempbistresult&0x1)==0x1));
             iterresult[5] = ((iterresult[5]<<1) | ((tempbistresult&0x2)==0x2));
             iterresult[6] = ((iterresult[6]<<1) | ((tempbistresult&0x4)==0x4));
             iterresult[7] = ((iterresult[7]<<1) | ((tempbistresult&0x8)==0x8));
          }
          currbistresult[0] = currbistresult[0] | ((tempbistresult&0x1)==1);
          currbistresult[1] = currbistresult[1] | ((tempbistresult&0x2)==0x2);
          currbistresult[2] = currbistresult[2] | ((tempbistresult&0x4)==0x4);
          currbistresult[3] = currbistresult[3] | ((tempbistresult&0x8)==0x8);

          for (loop=0; loop<=3; loop++)
          {
             if ((currbistresult[loop]==1) && (prevbistresult[loop]==1))
                bistpasscount[loop] = bistpasscount[loop]+1;
             if ((bistpasscount[loop]<4) && (currbistresult[loop]==0) && (itercount>2))
             {
                if (meancountalt_bistpasscount[loop] < bistpasscount[loop])
                {
                  meancountalt_bistpasscount[loop] = bistpasscount[loop];
                  meancountalt[loop] = ((itercount-1)-((bistpasscount[loop]+1)/2));
                }
                bistpasscount[loop] = 0;
             }
             if ((meancount[loop]==0) && (bistpasscount[loop]>=4) && ((currbistresult[loop]==0)||(itercount == 63)) && (prevbistresult[loop]==1))
                meancount[loop] = (itercount-1)-((bistpasscount[loop]+1)/2);
             prevbistresult[loop] = currbistresult[loop];
          }
        }while(++itercount<64);

        for (loop=0; loop<=3; loop++)
        {
          if ((lane0_active == 0) && (loop == 0))  continue;
          if ((lane1_active == 0) && (loop == 1))  continue;
          if ((lane2_active == 0) && (loop == 2))  continue;
          if ((lane3_active == 0) && (loop == 3))  continue;

          if (meancount[loop] == 0)
            meancount[loop] = meancountalt[loop];


          if (gen2_calib != 1)
          {
            ill1_val[loop] = ((0x04 + meancount[loop]*8) % 0x100);
            ill12_val[loop] = ((0x04 + meancount[loop]*8) >= 0x100) ? 0x10 : 0x00;
#ifdef XFSBL_DEBUG
			Xil_Out32(0xFFFE0000+loop*4,iterresult[loop]);
            Xil_Out32(0xFFFE0010+loop*4,iterresult[loop+4]);
            Xil_Out32(0xFFFE0020+loop*4,bistpasscount[loop]);
            Xil_Out32(0xFFFE0030+loop*4,meancount[loop]);
#endif
          }
          if (gen2_calib == 1)
          {
            ill1_val[loop] = ((0x104 + meancount[loop]*8) % 0x100);
            ill12_val[loop] = ((0x104 + meancount[loop]*8) >= 0x200) ? 0x02 : 0x01;
#ifdef XFSBL_DEBUG
			Xil_Out32(0xFFFE0040+loop*4,iterresult[loop]);
            Xil_Out32(0xFFFE0050+loop*4,iterresult[loop+4]);
            Xil_Out32(0xFFFE0060+loop*4,bistpasscount[loop]);
            Xil_Out32(0xFFFE0070+loop*4,meancount[loop]);
#endif
          }
        }
        if (gen2_calib != 1)
        {
           if (lane0_active == 1) Xil_Out32(SERDES_L0_TM_E_ILL1,ill1_val[0]);
           if (lane0_active == 1) PSU_Mask_Write(SERDES_L0_TM_ILL12, 0x000000F0U, ill12_val[0]);
           if (lane1_active == 1) Xil_Out32(SERDES_L1_TM_E_ILL1,ill1_val[1]);
           if (lane1_active == 1) PSU_Mask_Write(SERDES_L1_TM_ILL12, 0x000000F0U, ill12_val[1]);
           if (lane2_active == 1) Xil_Out32(SERDES_L2_TM_E_ILL1,ill1_val[2]);
           if (lane2_active == 1) PSU_Mask_Write(SERDES_L2_TM_ILL12, 0x000000F0U, ill12_val[2]);
           if (lane3_active == 1) Xil_Out32(SERDES_L3_TM_E_ILL1,ill1_val[3]);
           if (lane3_active == 1) PSU_Mask_Write(SERDES_L3_TM_ILL12, 0x000000F0U, ill12_val[3]);
        }
        if (gen2_calib == 1)
        {
           if (lane0_active == 1) Xil_Out32(SERDES_L0_TM_E_ILL2,ill1_val[0]);
           if (lane0_active == 1) PSU_Mask_Write(SERDES_L0_TM_ILL12, 0x0000000FU, ill12_val[0]);
           if (lane1_active == 1) Xil_Out32(SERDES_L1_TM_E_ILL2,ill1_val[1]);
           if (lane1_active == 1) PSU_Mask_Write(SERDES_L1_TM_ILL12, 0x0000000FU, ill12_val[1]);
           if (lane2_active == 1) Xil_Out32(SERDES_L2_TM_E_ILL2,ill1_val[2]);
           if (lane2_active == 1) PSU_Mask_Write(SERDES_L2_TM_ILL12, 0x0000000FU, ill12_val[2]);
           if (lane3_active == 1) Xil_Out32(SERDES_L3_TM_E_ILL2,ill1_val[3]);
           if (lane3_active == 1) PSU_Mask_Write(SERDES_L3_TM_ILL12, 0x0000000FU, ill12_val[3]);
        }


	if (lane0_active == 1) PSU_Mask_Write(SERDES_L0_TM_ANA_BYP_7, 0x00000030U, 0x00000000U);
	if (lane1_active == 1) PSU_Mask_Write(SERDES_L1_TM_ANA_BYP_7, 0x00000030U, 0x00000000U);
	if (lane2_active == 1) PSU_Mask_Write(SERDES_L2_TM_ANA_BYP_7, 0x00000030U, 0x00000000U);
	if (lane3_active == 1) PSU_Mask_Write(SERDES_L3_TM_ANA_BYP_7, 0x00000030U, 0x00000000U);

        Xil_Out32(SERDES_UPHY_SPARE0,0);
        if (lane0_active == 1)
        {
           Xil_Out32(SERDES_L0_BIST_CTRL_1,0);
           Xil_Out32(SERDES_L0_BIST_CTRL_2,0);
           Xil_Out32(SERDES_L0_BIST_RUN_LEN_L,0);
           Xil_Out32(SERDES_L0_BIST_ERR_INJ_POINT_L,0);
           Xil_Out32(SERDES_L0_BIST_RUNLEN_ERR_INJ_H,0);
           Xil_Out32(SERDES_L0_BIST_IDLE_TIME,0);
           Xil_Out32(SERDES_L0_BIST_MARKER_L,0);
           Xil_Out32(SERDES_L0_BIST_IDLE_CHAR_L,0);
           Xil_Out32(SERDES_L0_BIST_MARKER_IDLE_H,0);
           Xil_Out32(SERDES_L0_BIST_LOW_PULSE_TIME,0);
           Xil_Out32(SERDES_L0_BIST_TOTAL_PULSE_TIME,0);
           Xil_Out32(SERDES_L0_BIST_TEST_PAT_1,0);
           Xil_Out32(SERDES_L0_BIST_TEST_PAT_2,0);
           Xil_Out32(SERDES_L0_BIST_TEST_PAT_3,0);
           Xil_Out32(SERDES_L0_BIST_TEST_PAT_4,0);
           Xil_Out32(SERDES_L0_BIST_TEST_PAT_MSBS,0);
           Xil_Out32(SERDES_L0_BIST_PKT_NUM,0);
           Xil_Out32(SERDES_L0_BIST_FRM_IDLE_TIME,0);
           Xil_Out32(SERDES_L0_BIST_PKT_CTR_L,0);
           Xil_Out32(SERDES_L0_BIST_PKT_CTR_H,0);
           Xil_Out32(SERDES_L0_BIST_ERR_CTR_L,0);
           Xil_Out32(SERDES_L0_BIST_ERR_CTR_H,0);
           Xil_Out32(SERDES_L0_BIST_FILLER_OUT,1);
           Xil_Out32(SERDES_L0_BIST_FORCE_MK_RST,0);
           Xil_Out32(SERDES_L0_TM_DIG_22,0);
           PSU_Mask_Write(SERDES_RX_PROT_BUS_WIDTH, 0x00000003U, 0x00000001U);
           PSU_Mask_Write(SERDES_TX_PROT_BUS_WIDTH, 0x00000003U, 0x00000001U);
           PSU_Mask_Write(SERDES_LPBK_CTRL0, 0x00000007U, 0x00000000U);
        }
        if (lane1_active == 1)
        {
           Xil_Out32(SERDES_L1_BIST_CTRL_1,0);
           Xil_Out32(SERDES_L1_BIST_CTRL_2,0);
           Xil_Out32(SERDES_L1_BIST_RUN_LEN_L,0);
           Xil_Out32(SERDES_L1_BIST_ERR_INJ_POINT_L,0);
           Xil_Out32(SERDES_L1_BIST_RUNLEN_ERR_INJ_H,0);
           Xil_Out32(SERDES_L1_BIST_IDLE_TIME,0);
           Xil_Out32(SERDES_L1_BIST_MARKER_L,0);
           Xil_Out32(SERDES_L1_BIST_IDLE_CHAR_L,0);
           Xil_Out32(SERDES_L1_BIST_MARKER_IDLE_H,0);
           Xil_Out32(SERDES_L1_BIST_LOW_PULSE_TIME,0);
           Xil_Out32(SERDES_L1_BIST_TOTAL_PULSE_TIME,0);
           Xil_Out32(SERDES_L1_BIST_TEST_PAT_1,0);
           Xil_Out32(SERDES_L1_BIST_TEST_PAT_2,0);
           Xil_Out32(SERDES_L1_BIST_TEST_PAT_3,0);
           Xil_Out32(SERDES_L1_BIST_TEST_PAT_4,0);
           Xil_Out32(SERDES_L1_BIST_TEST_PAT_MSBS,0);
           Xil_Out32(SERDES_L1_BIST_PKT_NUM,0);
           Xil_Out32(SERDES_L1_BIST_FRM_IDLE_TIME,0);
           Xil_Out32(SERDES_L1_BIST_PKT_CTR_L,0);
           Xil_Out32(SERDES_L1_BIST_PKT_CTR_H,0);
           Xil_Out32(SERDES_L1_BIST_ERR_CTR_L,0);
           Xil_Out32(SERDES_L1_BIST_ERR_CTR_H,0);
           Xil_Out32(SERDES_L1_BIST_FILLER_OUT,1);
           Xil_Out32(SERDES_L1_BIST_FORCE_MK_RST,0);
           Xil_Out32(SERDES_L1_TM_DIG_22,0);
           PSU_Mask_Write(SERDES_RX_PROT_BUS_WIDTH, 0x0000000CU, 0x00000004U);
           PSU_Mask_Write(SERDES_TX_PROT_BUS_WIDTH, 0x0000000CU, 0x00000004U);
           PSU_Mask_Write(SERDES_LPBK_CTRL0, 0x00000070U, 0x00000000U);
        }
        if (lane2_active == 1)
        {
           Xil_Out32(SERDES_L2_BIST_CTRL_1,0);
           Xil_Out32(SERDES_L2_BIST_CTRL_2,0);
           Xil_Out32(SERDES_L2_BIST_RUN_LEN_L,0);
           Xil_Out32(SERDES_L2_BIST_ERR_INJ_POINT_L,0);
           Xil_Out32(SERDES_L2_BIST_RUNLEN_ERR_INJ_H,0);
           Xil_Out32(SERDES_L2_BIST_IDLE_TIME,0);
           Xil_Out32(SERDES_L2_BIST_MARKER_L,0);
           Xil_Out32(SERDES_L2_BIST_IDLE_CHAR_L,0);
           Xil_Out32(SERDES_L2_BIST_MARKER_IDLE_H,0);
           Xil_Out32(SERDES_L2_BIST_LOW_PULSE_TIME,0);
           Xil_Out32(SERDES_L2_BIST_TOTAL_PULSE_TIME,0);
           Xil_Out32(SERDES_L2_BIST_TEST_PAT_1,0);
           Xil_Out32(SERDES_L2_BIST_TEST_PAT_2,0);
           Xil_Out32(SERDES_L2_BIST_TEST_PAT_3,0);
           Xil_Out32(SERDES_L2_BIST_TEST_PAT_4,0);
           Xil_Out32(SERDES_L2_BIST_TEST_PAT_MSBS,0);
           Xil_Out32(SERDES_L2_BIST_PKT_NUM,0);
           Xil_Out32(SERDES_L2_BIST_FRM_IDLE_TIME,0);
           Xil_Out32(SERDES_L2_BIST_PKT_CTR_L,0);
           Xil_Out32(SERDES_L2_BIST_PKT_CTR_H,0);
           Xil_Out32(SERDES_L2_BIST_ERR_CTR_L,0);
           Xil_Out32(SERDES_L2_BIST_ERR_CTR_H,0);
           Xil_Out32(SERDES_L2_BIST_FILLER_OUT,1);
           Xil_Out32(SERDES_L2_BIST_FORCE_MK_RST,0);
           Xil_Out32(SERDES_L2_TM_DIG_22,0);
           PSU_Mask_Write(SERDES_RX_PROT_BUS_WIDTH, 0x00000030U, 0x00000010U);
           PSU_Mask_Write(SERDES_TX_PROT_BUS_WIDTH, 0x00000030U, 0x00000010U);
           PSU_Mask_Write(SERDES_LPBK_CTRL1, 0x00000007U, 0x00000000U);
        }
        if (lane3_active == 1)
        {
           Xil_Out32(SERDES_L3_BIST_CTRL_1,0);
           Xil_Out32(SERDES_L3_BIST_CTRL_2,0);
           Xil_Out32(SERDES_L3_BIST_RUN_LEN_L,0);
           Xil_Out32(SERDES_L3_BIST_ERR_INJ_POINT_L,0);
           Xil_Out32(SERDES_L3_BIST_RUNLEN_ERR_INJ_H,0);
           Xil_Out32(SERDES_L3_BIST_IDLE_TIME,0);
           Xil_Out32(SERDES_L3_BIST_MARKER_L,0);
           Xil_Out32(SERDES_L3_BIST_IDLE_CHAR_L,0);
           Xil_Out32(SERDES_L3_BIST_MARKER_IDLE_H,0);
           Xil_Out32(SERDES_L3_BIST_LOW_PULSE_TIME,0);
           Xil_Out32(SERDES_L3_BIST_TOTAL_PULSE_TIME,0);
           Xil_Out32(SERDES_L3_BIST_TEST_PAT_1,0);
           Xil_Out32(SERDES_L3_BIST_TEST_PAT_2,0);
           Xil_Out32(SERDES_L3_BIST_TEST_PAT_3,0);
           Xil_Out32(SERDES_L3_BIST_TEST_PAT_4,0);
           Xil_Out32(SERDES_L3_BIST_TEST_PAT_MSBS,0);
           Xil_Out32(SERDES_L3_BIST_PKT_NUM,0);
           Xil_Out32(SERDES_L3_BIST_FRM_IDLE_TIME,0);
           Xil_Out32(SERDES_L3_BIST_PKT_CTR_L,0);
           Xil_Out32(SERDES_L3_BIST_PKT_CTR_H,0);
           Xil_Out32(SERDES_L3_BIST_ERR_CTR_L,0);
           Xil_Out32(SERDES_L3_BIST_ERR_CTR_H,0);
           Xil_Out32(SERDES_L3_BIST_FILLER_OUT,1);
           Xil_Out32(SERDES_L3_BIST_FORCE_MK_RST,0);
           Xil_Out32(SERDES_L3_TM_DIG_22,0);
           PSU_Mask_Write(SERDES_RX_PROT_BUS_WIDTH, 0x000000C0U, 0x00000040U);
           PSU_Mask_Write(SERDES_TX_PROT_BUS_WIDTH, 0x000000C0U, 0x00000040U);
           PSU_Mask_Write(SERDES_LPBK_CTRL1, 0x00000070U, 0x00000000U);
        }
        return 1;
}

static int serdes_illcalib (u32 lane3_protocol, u32 lane3_rate, u32 lane2_protocol, u32 lane2_rate, u32 lane1_protocol, u32 lane1_rate, u32 lane0_protocol, u32 lane0_rate)
//Protocol values
//pcie = 1; sata = 2; usb = 3; dp = 4; sgmii = 5
//Rate values
//pcie_gen1 = 0; pcie_gen2 = 1;
//sata_gen1 = 1; sata_gen2 = 2; sata_gen3 = 3;
//usb = 0; sgmii = 0; DP = 0;
{
  unsigned int rdata=0;
  unsigned int sata_gen2=1;
  unsigned int temp_ill12=0;
  unsigned int temp_PLL_REF_SEL_OFFSET;
  unsigned int temp_TM_IQ_ILL1;
  unsigned int temp_TM_E_ILL1;
  unsigned int temp_tx_dig_tm_61;
  unsigned int temp_tm_dig_6;
  unsigned int temp_pll_fbdiv_frac_3_msb_offset;

  if ((lane0_protocol == 2)||(lane0_protocol == 1))
  {
    Xil_Out32(SERDES_L0_TM_IQ_ILL7, 0xF3);
    Xil_Out32(SERDES_L0_TM_E_ILL7, 0xF3);
    Xil_Out32(SERDES_L0_TM_IQ_ILL8,0xF3);
    Xil_Out32(SERDES_L0_TM_E_ILL8,0xF3);
  }
  if ((lane1_protocol == 2)||(lane1_protocol == 1))
  {
    Xil_Out32(SERDES_L1_TM_IQ_ILL7, 0xF3);
    Xil_Out32(SERDES_L1_TM_E_ILL7, 0xF3);
    Xil_Out32(SERDES_L1_TM_IQ_ILL8,0xF3);
    Xil_Out32(SERDES_L1_TM_E_ILL8,0xF3);
  }
  if ((lane2_protocol == 2)||(lane2_protocol == 1))
  {
    Xil_Out32(SERDES_L2_TM_IQ_ILL7, 0xF3);
    Xil_Out32(SERDES_L2_TM_E_ILL7, 0xF3);
    Xil_Out32(SERDES_L2_TM_IQ_ILL8,0xF3);
    Xil_Out32(SERDES_L2_TM_E_ILL8,0xF3);
  }
  if ((lane3_protocol == 2)||(lane3_protocol == 1))
  {
    Xil_Out32(SERDES_L3_TM_IQ_ILL7, 0xF3);
    Xil_Out32(SERDES_L3_TM_E_ILL7, 0xF3);
    Xil_Out32(SERDES_L3_TM_IQ_ILL8,0xF3);
    Xil_Out32(SERDES_L3_TM_E_ILL8,0xF3);
  }

  if (sata_gen2 == 1)
  {
    if (lane0_protocol == 2)
    {
      temp_pll_fbdiv_frac_3_msb_offset=Xil_In32(SERDES_L0_PLL_FBDIV_FRAC_3_MSB);
      Xil_Out32(SERDES_L0_PLL_FBDIV_FRAC_3_MSB,0x0);
      temp_PLL_REF_SEL_OFFSET = Xil_In32(SERDES_PLL_REF_SEL0_OFFSET);
      PSU_Mask_Write(SERDES_PLL_REF_SEL0_OFFSET, 0x0000001FU, 0x0000000DU);
      temp_TM_IQ_ILL1 = Xil_In32(SERDES_L0_TM_IQ_ILL1);
      temp_TM_E_ILL1 = Xil_In32(SERDES_L0_TM_E_ILL1);
      Xil_Out32(SERDES_L0_TM_IQ_ILL1,0x78);
      temp_tx_dig_tm_61 = Xil_In32(SERDES_L0_TX_DIG_TM_61);
      temp_tm_dig_6 = Xil_In32(SERDES_L0_TM_DIG_6);
      PSU_Mask_Write(SERDES_L0_TX_DIG_TM_61, 0x0000000BU, 0x00000000U);
      PSU_Mask_Write(SERDES_L0_TM_DIG_6, 0x0000000FU, 0x00000000U);
      temp_ill12 = Xil_In32(SERDES_L0_TM_ILL12) & 0xF0;

      serdes_illcalib_pcie_gen1 (0, 0, 0, 0, 0, 0, 0, 1, 0, 0);

      Xil_Out32(SERDES_L0_PLL_FBDIV_FRAC_3_MSB,temp_pll_fbdiv_frac_3_msb_offset);
      Xil_Out32(SERDES_PLL_REF_SEL0_OFFSET, temp_PLL_REF_SEL_OFFSET);
      Xil_Out32(SERDES_L0_TM_IQ_ILL1,temp_TM_IQ_ILL1);
      Xil_Out32(SERDES_L0_TX_DIG_TM_61, temp_tx_dig_tm_61);
      Xil_Out32(SERDES_L0_TM_DIG_6, temp_tm_dig_6);
      Xil_Out32(SERDES_L0_TM_E_ILL2, Xil_In32(SERDES_L0_TM_E_ILL1));
      temp_ill12 = temp_ill12 | (Xil_In32(SERDES_L0_TM_ILL12)>>4 & 0xF);
      Xil_Out32(SERDES_L0_TM_ILL12, temp_ill12);
      Xil_Out32(SERDES_L0_TM_E_ILL1, temp_TM_E_ILL1);
    }
    if (lane1_protocol == 2)
    {
      temp_pll_fbdiv_frac_3_msb_offset=Xil_In32(SERDES_L1_PLL_FBDIV_FRAC_3_MSB);
      Xil_Out32(SERDES_L1_PLL_FBDIV_FRAC_3_MSB,0x0);
      temp_PLL_REF_SEL_OFFSET = Xil_In32(SERDES_PLL_REF_SEL1_OFFSET);
      PSU_Mask_Write(SERDES_PLL_REF_SEL1_OFFSET, 0x0000001FU, 0x0000000DU);
      temp_TM_IQ_ILL1 = Xil_In32(SERDES_L1_TM_IQ_ILL1);
      temp_TM_E_ILL1 = Xil_In32(SERDES_L1_TM_E_ILL1);
      Xil_Out32(SERDES_L1_TM_IQ_ILL1,0x78);
      temp_tx_dig_tm_61 = Xil_In32(SERDES_L1_TX_DIG_TM_61);
      temp_tm_dig_6 = Xil_In32(SERDES_L1_TM_DIG_6);
      PSU_Mask_Write(SERDES_L1_TX_DIG_TM_61, 0x0000000BU, 0x00000000U);
      PSU_Mask_Write(SERDES_L1_TM_DIG_6, 0x0000000FU, 0x00000000U);
      temp_ill12 = Xil_In32(SERDES_L1_TM_ILL12) & 0xF0;

      serdes_illcalib_pcie_gen1 (1, 0, 0, 0, 0, 1, 0, 0, 0, 0);

      Xil_Out32(SERDES_L1_PLL_FBDIV_FRAC_3_MSB,temp_pll_fbdiv_frac_3_msb_offset);
      Xil_Out32(SERDES_PLL_REF_SEL1_OFFSET, temp_PLL_REF_SEL_OFFSET);
      Xil_Out32(SERDES_L1_TM_IQ_ILL1,temp_TM_IQ_ILL1);
      Xil_Out32(SERDES_L1_TX_DIG_TM_61, temp_tx_dig_tm_61);
      Xil_Out32(SERDES_L1_TM_DIG_6, temp_tm_dig_6);
      Xil_Out32(SERDES_L1_TM_E_ILL2, Xil_In32(SERDES_L1_TM_E_ILL1));
      temp_ill12 = temp_ill12 | (Xil_In32(SERDES_L1_TM_ILL12)>>4 & 0xF);
      Xil_Out32(SERDES_L1_TM_ILL12, temp_ill12);
      Xil_Out32(SERDES_L1_TM_E_ILL1, temp_TM_E_ILL1);
    }
    if (lane2_protocol == 2)
    {
      temp_pll_fbdiv_frac_3_msb_offset=Xil_In32(SERDES_L2_PLL_FBDIV_FRAC_3_MSB);
      Xil_Out32(SERDES_L2_PLL_FBDIV_FRAC_3_MSB,0x0);
      temp_PLL_REF_SEL_OFFSET = Xil_In32(SERDES_PLL_REF_SEL2_OFFSET);
      PSU_Mask_Write(SERDES_PLL_REF_SEL2_OFFSET, 0x0000001FU, 0x0000000DU);
      temp_TM_IQ_ILL1 = Xil_In32(SERDES_L2_TM_IQ_ILL1);
      temp_TM_E_ILL1 = Xil_In32(SERDES_L2_TM_E_ILL1);
      Xil_Out32(SERDES_L2_TM_IQ_ILL1,0x78);
      temp_tx_dig_tm_61 = Xil_In32(SERDES_L2_TX_DIG_TM_61);
      temp_tm_dig_6 = Xil_In32(SERDES_L2_TM_DIG_6);
      PSU_Mask_Write(SERDES_L2_TX_DIG_TM_61, 0x0000000BU, 0x00000000U);
      PSU_Mask_Write(SERDES_L2_TM_DIG_6, 0x0000000FU, 0x00000000U);
      temp_ill12 = Xil_In32(SERDES_L2_TM_ILL12) & 0xF0;

      serdes_illcalib_pcie_gen1 (2, 0, 0, 1, 0, 0, 0, 0, 0, 0);

      Xil_Out32(SERDES_L2_PLL_FBDIV_FRAC_3_MSB,temp_pll_fbdiv_frac_3_msb_offset);
      Xil_Out32(SERDES_PLL_REF_SEL2_OFFSET, temp_PLL_REF_SEL_OFFSET);
      Xil_Out32(SERDES_L2_TM_IQ_ILL1,temp_TM_IQ_ILL1);
      Xil_Out32(SERDES_L2_TX_DIG_TM_61, temp_tx_dig_tm_61);
      Xil_Out32(SERDES_L2_TM_DIG_6, temp_tm_dig_6);
      Xil_Out32(SERDES_L2_TM_E_ILL2, Xil_In32(SERDES_L2_TM_E_ILL1));
      temp_ill12 = temp_ill12 | (Xil_In32(SERDES_L2_TM_ILL12)>>4 & 0xF);
      Xil_Out32(SERDES_L2_TM_ILL12, temp_ill12);
      Xil_Out32(SERDES_L2_TM_E_ILL1, temp_TM_E_ILL1);
    }
    if (lane3_protocol == 2)
    {
      temp_pll_fbdiv_frac_3_msb_offset=Xil_In32(SERDES_L3_PLL_FBDIV_FRAC_3_MSB);
      Xil_Out32(SERDES_L3_PLL_FBDIV_FRAC_3_MSB,0x0);
      temp_PLL_REF_SEL_OFFSET = Xil_In32(SERDES_PLL_REF_SEL3_OFFSET);
      PSU_Mask_Write(SERDES_PLL_REF_SEL3_OFFSET, 0x0000001FU, 0x0000000DU);
      temp_TM_IQ_ILL1 = Xil_In32(SERDES_L3_TM_IQ_ILL1);
      temp_TM_E_ILL1 = Xil_In32(SERDES_L3_TM_E_ILL1);
      Xil_Out32(SERDES_L3_TM_IQ_ILL1,0x78);
      temp_tx_dig_tm_61 = Xil_In32(SERDES_L3_TX_DIG_TM_61);
      temp_tm_dig_6 = Xil_In32(SERDES_L3_TM_DIG_6);
      PSU_Mask_Write(SERDES_L3_TX_DIG_TM_61, 0x0000000BU, 0x00000000U);
      PSU_Mask_Write(SERDES_L3_TM_DIG_6, 0x0000000FU, 0x00000000U);
      temp_ill12 = Xil_In32(SERDES_L3_TM_ILL12) & 0xF0;

      serdes_illcalib_pcie_gen1 (3, 1, 0, 0, 0, 0, 0, 0, 0, 0);

      Xil_Out32(SERDES_L3_PLL_FBDIV_FRAC_3_MSB,temp_pll_fbdiv_frac_3_msb_offset);
      Xil_Out32(SERDES_PLL_REF_SEL3_OFFSET, temp_PLL_REF_SEL_OFFSET);
      Xil_Out32(SERDES_L3_TM_IQ_ILL1,temp_TM_IQ_ILL1);
      Xil_Out32(SERDES_L3_TX_DIG_TM_61, temp_tx_dig_tm_61);
      Xil_Out32(SERDES_L3_TM_DIG_6, temp_tm_dig_6);
      Xil_Out32(SERDES_L3_TM_E_ILL2, Xil_In32(SERDES_L3_TM_E_ILL1));
      temp_ill12 = temp_ill12 | (Xil_In32(SERDES_L3_TM_ILL12)>>4 & 0xF);
      Xil_Out32(SERDES_L3_TM_ILL12, temp_ill12);
      Xil_Out32(SERDES_L3_TM_E_ILL1, temp_TM_E_ILL1);
    }
    rdata  = Xil_In32(SERDES_UPHY_SPARE0);
    rdata  = (rdata & 0xDF);
    Xil_Out32(SERDES_UPHY_SPARE0,rdata);
  }

  if ((lane0_protocol == 2)&&(lane0_rate == 3))
  {
    PSU_Mask_Write(SERDES_L0_TM_ILL11, 0x000000F0U, 0x00000020U);
    PSU_Mask_Write(SERDES_L0_TM_E_ILL3, 0x000000FFU, 0x00000094U);
  }
  if ((lane1_protocol == 2)&&(lane1_rate == 3))
  {
    PSU_Mask_Write(SERDES_L1_TM_ILL11, 0x000000F0U, 0x00000020U);
    PSU_Mask_Write(SERDES_L1_TM_E_ILL3, 0x000000FFU, 0x00000094U);
  }
  if ((lane2_protocol == 2)&&(lane2_rate == 3))
  {
    PSU_Mask_Write(SERDES_L2_TM_ILL11, 0x000000F0U, 0x00000020U);
    PSU_Mask_Write(SERDES_L2_TM_E_ILL3, 0x000000FFU, 0x00000094U);
  }
  if ((lane3_protocol == 2)&&(lane3_rate == 3))
  {
    PSU_Mask_Write(SERDES_L3_TM_ILL11, 0x000000F0U, 0x00000020U);
    PSU_Mask_Write(SERDES_L3_TM_E_ILL3, 0x000000FFU, 0x00000094U);
  }

  //PCIe settings
  //If lane-0 is PCIe, we need to run pcie dynamic search on all active pcie lanes
  //and reset sequence on all active lanes
  if (lane0_protocol == 1)
  {
   if (lane0_rate == 0)
   {
     serdes_illcalib_pcie_gen1 (0, lane3_protocol, lane3_rate, lane2_protocol, lane2_rate, lane1_protocol, lane1_rate, lane0_protocol, 0, 0);
   }
   else
   {
     serdes_illcalib_pcie_gen1 (0, lane3_protocol, lane3_rate, lane2_protocol, lane2_rate, lane1_protocol, lane1_rate, lane0_protocol, 0, 0);
     serdes_illcalib_pcie_gen1 (0, lane3_protocol, lane3_rate, lane2_protocol, lane2_rate, lane1_protocol, lane1_rate, lane0_protocol, lane0_rate, 1);
   }
  }

  //USB3 settings
  if (lane0_protocol == 3) Xil_Out32(SERDES_L0_TM_IQ_ILL8,0xF3);
  if (lane0_protocol == 3) Xil_Out32(SERDES_L0_TM_E_ILL8,0xF3);
  if (lane0_protocol == 3) Xil_Out32(SERDES_L0_TM_ILL12,0x20);
  if (lane0_protocol == 3) Xil_Out32(SERDES_L0_TM_E_ILL1,0x37);

  if (lane1_protocol == 3) Xil_Out32(SERDES_L1_TM_IQ_ILL8,0xF3);
  if (lane1_protocol == 3) Xil_Out32(SERDES_L1_TM_E_ILL8,0xF3);
  if (lane1_protocol == 3) Xil_Out32(SERDES_L1_TM_ILL12,0x20);
  if (lane1_protocol == 3) Xil_Out32(SERDES_L1_TM_E_ILL1,0x37);

  if (lane2_protocol == 3) Xil_Out32(SERDES_L2_TM_IQ_ILL8,0xF3);
  if (lane2_protocol == 3) Xil_Out32(SERDES_L2_TM_E_ILL8,0xF3);
  if (lane2_protocol == 3) Xil_Out32(SERDES_L2_TM_ILL12,0x20);
  if (lane2_protocol == 3) Xil_Out32(SERDES_L2_TM_E_ILL1,0x37);

  if (lane3_protocol == 3) Xil_Out32(SERDES_L3_TM_IQ_ILL8,0xF3);
  if (lane3_protocol == 3) Xil_Out32(SERDES_L3_TM_E_ILL8,0xF3);
  if (lane3_protocol == 3) Xil_Out32(SERDES_L3_TM_ILL12,0x20);
  if (lane3_protocol == 3) Xil_Out32(SERDES_L3_TM_E_ILL1,0x37);

  return 1;
}


//Kishore -- ILL calibration code ends

/*Following SERDES programming sequences that a user need to follow to work
 * around the known limitation with SERDES. These sequences should done
 * before STEP 1 and STEP 2 as described in previous section. These
 * programming steps are *required for current silicon version and are
 * likely to undergo further changes with subsequent silicon versions.
 */


static int serdes_enb_coarse_saturation(void)
{
  /*Enable PLL Coarse Code saturation Logic*/
	Xil_Out32(0xFD402094, 0x00000010);
	Xil_Out32(0xFD406094, 0x00000010);
	Xil_Out32(0xFD40A094, 0x00000010);
	Xil_Out32(0xFD40E094, 0x00000010);
		return 1;
}

int serdes_fixcal_code(void)
{
	int MaskStatus = 1;

  unsigned int rdata = 0;

	/*The valid codes are from 0x26 to 0x3C.
	*There are 23 valid codes in total.
	*/
 /*Each element of array stands for count of occurence of valid code.*/
	unsigned int match_pmos_code[23];
	/*Each element of array stands for count of occurence of valid code.*/
	/*The valid codes are from 0xC to 0x12.
	*There are 7 valid codes in total.
	*/
	unsigned int match_nmos_code[23];
	/*Each element of array stands for count of occurence of valid code.*/
	/*The valid codes are from 0x6 to 0xC.
	* There are 7 valid codes in total.
	*/
	unsigned int match_ical_code[7];
	/*Each element of array stands for count of occurence of valid code.*/
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

  rdata = Xil_In32(0XFD40289C);
  rdata = rdata & ~0x03;
  rdata = rdata | 0x1;
  Xil_Out32(0XFD40289C, rdata);
  // check supply good status before starting AFE sequencing
  int count = 0;
  do
  {
    if (count == PSU_MASK_POLL_TIME)
      break;
    rdata = Xil_In32(0xFD402B1C);
    count++;
  }while((rdata&0x0000000E) !=0x0000000E);

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
		Xil_Out32(0xFD410010, 0x00000000);
		Xil_Out32(0xFD410014, 0x00000000);

	/*Set ICM_CFG value*/
	/*This will trigger recalibration of all stages*/
	Xil_Out32(0xFD410010, 0x00000001);
	Xil_Out32(0xFD410014, 0x00000000);

	/*is calibration done? polling on L3_CALIB_DONE_STATUS*/
	MaskStatus = mask_poll(0xFD40EF14, 0x2);
	if (MaskStatus == 0) {
		/*failure here is because of calibration done timeout*/
		xil_printf("#SERDES initialization timed out\n\r");
		return MaskStatus;
	}

	p_code = mask_read(0xFD40EF18, 0xFFFFFFFF);/*PMOS code*/
	n_code = mask_read(0xFD40EF1C, 0xFFFFFFFF);/*NMOS code*/
	/*m_code = mask_read(0xFD40EF20, 0xFFFFFFFF)*/;/*MPHY code*/
	i_code = mask_read(0xFD40EF24, 0xFFFFFFFF);/*ICAL code*/
	r_code = mask_read(0xFD40EF28, 0xFFFFFFFF);/*RX code*/
	/*u_code = mask_read(0xFD40EF2C, 0xFFFFFFFF)*/;/*USB2 code*/

	/*PMOS code in acceptable range*/
	if ((p_code >= 0x26) && (p_code <= 0x3C))
		match_pmos_code[p_code - 0x26] += 1;

	/*NMOS code in acceptable range*/
	if ((n_code >= 0x26) && (n_code <= 0x3C))
		match_nmos_code[n_code - 0x26] += 1;

	/*PMOS code in acceptable range*/
	if ((i_code >= 0xC) && (i_code <= 0x12))
		match_ical_code[i_code - 0xC] += 1;

	/*NMOS code in acceptable range*/
	if ((r_code >= 0x6) && (r_code <= 0xC))
		match_rcal_code[r_code - 0x6] += 1;


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
	L3_TM_CALIB_DIG20 = mask_read(0xFD40EC50, 0xFFFFFFF0);/*read DIG20*/
	L3_TM_CALIB_DIG20 = L3_TM_CALIB_DIG20 | 0x8 | ((p_code >> 2) & 0x7);


	/*L3_TM_CALIB_DIG19[7:6]	PSW Code [1:0]*/
	/*L3_TM_CALIB_DIG19[5]	PSW Override*/
	/*L3_TM_CALIB_DIG19[2]	NSW MSB Override*/
	/*L3_TM_CALIB_DIG19[1:0]	NSW Code [4:3]*/
	L3_TM_CALIB_DIG19 = mask_read(0xFD40EC4C, 0xFFFFFF18);/*read DIG19*/
	L3_TM_CALIB_DIG19 = L3_TM_CALIB_DIG19 | ((p_code & 0x3) << 6)
		| 0x20 | 0x4 | ((n_code >> 3) & 0x3);

	/*L3_TM_CALIB_DIG18[7:5]	NSW Code [2:0]*/
	/*L3_TM_CALIB_DIG18[4]	NSW Override*/
	L3_TM_CALIB_DIG18 = mask_read(0xFD40EC48, 0xFFFFFF0F);/*read DIG18*/
	L3_TM_CALIB_DIG18 = L3_TM_CALIB_DIG18 | ((n_code & 0x7) << 5) | 0x10;


	/*L3_TM_CALIB_DIG16[2:0]	RX Code [3:1]*/
	L3_TM_CALIB_DIG16 = mask_read(0xFD40EC40, 0xFFFFFFF8);/*read DIG16*/
	L3_TM_CALIB_DIG16 = L3_TM_CALIB_DIG16 | ((r_code >> 1) & 0x7);

	/*L3_TM_CALIB_DIG15[7]	RX Code [0]*/
	/*L3_TM_CALIB_DIG15[6]	RX CODE Override*/
	/*L3_TM_CALIB_DIG15[3]	ICAL MSB Override*/
	/*L3_TM_CALIB_DIG15[2:0]	ICAL Code [3:1]*/
	L3_TM_CALIB_DIG15 = mask_read(0xFD40EC3C, 0xFFFFFF30);/*read DIG15*/
	L3_TM_CALIB_DIG15 = L3_TM_CALIB_DIG15 | ((r_code & 0x1) << 7)
	| 0x40 | 0x8 | ((i_code >> 1) & 0x7);

	/*L3_TM_CALIB_DIG14[7]	ICAL Code [0]*/
	/*L3_TM_CALIB_DIG14[6]	ICAL Override*/
	L3_TM_CALIB_DIG14 = mask_read(0xFD40EC38, 0xFFFFFF3F);/*read DIG14*/
	L3_TM_CALIB_DIG14 = L3_TM_CALIB_DIG14 | ((i_code & 0x1) << 7) | 0x40;

	/*Forces the calibration values*/
	Xil_Out32(0xFD40EC50, L3_TM_CALIB_DIG20);
	Xil_Out32(0xFD40EC4C, L3_TM_CALIB_DIG19);
	Xil_Out32(0xFD40EC48, L3_TM_CALIB_DIG18);
	Xil_Out32(0xFD40EC40, L3_TM_CALIB_DIG16);
	Xil_Out32(0xFD40EC3C, L3_TM_CALIB_DIG15);
	Xil_Out32(0xFD40EC38, L3_TM_CALIB_DIG14);
	return MaskStatus;

}

static int init_serdes(void)
{
	int status = 1;

	status &=  psu_resetin_init_data();

	status &= serdes_fixcal_code();
	status &= serdes_enb_coarse_saturation();

	status &=  psu_serdes_init_data();
	status &=  psu_resetout_init_data();

	return status;
}

/* Sequence used to init SI5345 configuration process */
si5345_revd_register_t const si5345_start[] =
{
	{ 0x0B24, 0xC0 },
	{ 0x0B25, 0x00 },
	{ 0x0540, 0x01 },
};

/* SI5345 configuration registers generated by Clock Builder Pro on Windows 0S */
si5345_revd_register_t const si5345_config[] =
{
	/* Tool version*/
	{ 0x0006, 0x00 },
	{ 0x0007, 0x00 },
	{ 0x0008, 0x00 },

	/* I2C address */
	{ 0x000B, 0x68 },

	/* Unknown */
	{ 0x0016, 0x02 },
	{ 0x0017, 0xDC },
	{ 0x0018, 0xFF },
	{ 0x0019, 0xFF },
	{ 0x001A, 0xFF },

	/* SPI settings */
	{ 0x002B, 0x02 },

	/* LOS settings */
	{ 0x002C, 0x00 },
	{ 0x002D, 0x00 },
	{ 0x002E, 0x00 },
	{ 0x002F, 0x00 },
	{ 0x0030, 0x00 },
	{ 0x0031, 0x00 },
	{ 0x0032, 0x00 },
	{ 0x0033, 0x00 },
	{ 0x0034, 0x00 },
	{ 0x0035, 0x00 },
	{ 0x0036, 0x00 },
	{ 0x0037, 0x00 },
	{ 0x0038, 0x00 },
	{ 0x0039, 0x00 },
	{ 0x003A, 0x00 },
	{ 0x003B, 0x00 },
	{ 0x003C, 0x00 },
	{ 0x003D, 0x00 },

	/* OOF ? */
	{ 0x003F, 0x00 },
	{ 0x0040, 0x04 },
	{ 0x0041, 0x00 },
	{ 0x0042, 0x00 },
	{ 0x0043, 0x00 },
	{ 0x0044, 0x00 },
	{ 0x0045, 0x0C },
	{ 0x0046, 0x00 },
	{ 0x0047, 0x00 },
	{ 0x0048, 0x00 },
	{ 0x0049, 0x00 },
	{ 0x004A, 0x00 },
	{ 0x004B, 0x00 },
	{ 0x004C, 0x00 },
	{ 0x004D, 0x00 },
	{ 0x004E, 0x00 },
	{ 0x004F, 0x00 },
	{ 0x0050, 0x0F },
	{ 0x0051, 0x00 },
	{ 0x0052, 0x00 },
	{ 0x0053, 0x00 },
	{ 0x0054, 0x00 },
	{ 0x0055, 0x00 },
	{ 0x0056, 0x00 },
	{ 0x0057, 0x00 },
	{ 0x0058, 0x00 },
	{ 0x0059, 0x00 },
	{ 0x005A, 0x00 },
	{ 0x005B, 0x00 },
	{ 0x005C, 0x00 },
	{ 0x005D, 0x00 },
	{ 0x005E, 0x00 },
	{ 0x005F, 0x00 },
	{ 0x0060, 0x00 },
	{ 0x0061, 0x00 },
	{ 0x0062, 0x00 },
	{ 0x0063, 0x00 },
	{ 0x0064, 0x00 },
	{ 0x0065, 0x00 },
	{ 0x0066, 0x00 },
	{ 0x0067, 0x00 },
	{ 0x0068, 0x00 },
	{ 0x0069, 0x00 },
	{ 0x0092, 0x00 },
	{ 0x0093, 0x00 },
	{ 0x0095, 0x00 },
	{ 0x0096, 0x00 },
	{ 0x0098, 0x00 },
	{ 0x009A, 0x00 },
	{ 0x009B, 0x00 },
	{ 0x009D, 0x00 },
	{ 0x009E, 0x00 },
	{ 0x00A0, 0x00 },
	{ 0x00A2, 0x00 },
	{ 0x00A9, 0x00 },
	{ 0x00AA, 0x00 },
	{ 0x00AB, 0x00 },
	{ 0x00AC, 0x00 },

	/* Lack of NVM commands 0xE4 (write), 0xE5 (read) */

	/* Fast lock */
	{ 0x00E5, 0x01 },
	{ 0x00EA, 0x00 },
	{ 0x00EB, 0x00 },
	{ 0x00EC, 0x00 },
	{ 0x00ED, 0x00 },

	/* Global gating */
	{ 0x0102, 0x01 },

	/* Output 0 */
	{ 0x0108, 0x06 },
	{ 0x0109, 0x09 },
	{ 0x010A, 0x33 },
	{ 0x010B, 0x08 },

	/* Output 1 */
	{ 0x010D, 0x01 },
	{ 0x010E, 0x09 },
	{ 0x010F, 0x3B },
	{ 0x0110, 0x28 },

	/* Output 2 */
	{ 0x0112, 0x01 },
	{ 0x0113, 0x09 },
	{ 0x0114, 0x3B },
	{ 0x0115, 0x28 },

	/* Output 3 */
	{ 0x0117, 0x01 },
	{ 0x0118, 0x09 },
	{ 0x0119, 0x3B },
	{ 0x011A, 0x28 },

	/* Output 4 */
	{ 0x011C, 0x01 },
	{ 0x011D, 0x09 },
	{ 0x011E, 0x3B },
	{ 0x011F, 0x28 },

	/* Output 5 */
	{ 0x0121, 0x01 },
	{ 0x0122, 0x09 },
	{ 0x0123, 0x3B },
	{ 0x0124, 0x28 },

	/* Output 6 */
	{ 0x0126, 0x01 },
	{ 0x0127, 0x09 },
	{ 0x0128, 0x3B },
	{ 0x0129, 0x28 },

	/* Output 7 */
	{ 0x012B, 0x01 },
	{ 0x012C, 0x09 },
	{ 0x012D, 0x3B },
	{ 0x012E, 0x28 },

	/* Output 8 */
	{ 0x0130, 0x01 },
	{ 0x0131, 0x09 },
	{ 0x0132, 0x3B },
	{ 0x0133, 0x28 },

	/* Output 9 */
	{ 0x013A, 0x01 },
	{ 0x013B, 0x09 },
	{ 0x013C, 0x3B },
	{ 0x013D, 0x28 },

	/* ? */
	{ 0x013F, 0x00 },
	{ 0x0140, 0x00 },
	{ 0x0141, 0x40 },
	{ 0x0142, 0xFF },

	/* Prescale */
	{ 0x0206, 0x00 },
	{ 0x0208, 0x00 },
	{ 0x0209, 0x00 },
	{ 0x020A, 0x00 },
	{ 0x020B, 0x00 },
	{ 0x020C, 0x00 },
	{ 0x020D, 0x00 },
	{ 0x020E, 0x00 },
	{ 0x020F, 0x00 },
	{ 0x0210, 0x00 },
	{ 0x0211, 0x00 },
	{ 0x0212, 0x00 },
	{ 0x0213, 0x00 },
	{ 0x0214, 0x00 },
	{ 0x0215, 0x00 },
	{ 0x0216, 0x00 },
	{ 0x0217, 0x00 },
	{ 0x0218, 0x00 },
	{ 0x0219, 0x00 },
	{ 0x021A, 0x00 },
	{ 0x021B, 0x00 },
	{ 0x021C, 0x00 },
	{ 0x021D, 0x00 },
	{ 0x021E, 0x00 },
	{ 0x021F, 0x00 },
	{ 0x0220, 0x00 },
	{ 0x0221, 0x00 },
	{ 0x0222, 0x00 },
	{ 0x0223, 0x00 },
	{ 0x0224, 0x00 },
	{ 0x0225, 0x00 },
	{ 0x0226, 0x00 },
	{ 0x0227, 0x00 },
	{ 0x0228, 0x00 },
	{ 0x0229, 0x00 },
	{ 0x022A, 0x00 },
	{ 0x022B, 0x00 },
	{ 0x022C, 0x00 },
	{ 0x022D, 0x00 },
	{ 0x022E, 0x00 },
	{ 0x022F, 0x00 },
	{ 0x0231, 0x0B },
	{ 0x0232, 0x0B },
	{ 0x0233, 0x0B },
	{ 0x0234, 0x0B },
	{ 0x0235, 0x00 },
	{ 0x0236, 0x00 },
	{ 0x0237, 0x00 },
	{ 0x0238, 0x00 },
	{ 0x0239, 0x84 },
	{ 0x023A, 0x00 },
	{ 0x023B, 0x00 },
	{ 0x023C, 0x00 },
	{ 0x023D, 0x00 },
	{ 0x023E, 0x80 },
	{ 0x024A, 0x00 },
	{ 0x024B, 0x00 },
	{ 0x024C, 0x00 },
	{ 0x024D, 0x00 },
	{ 0x024E, 0x00 },
	{ 0x024F, 0x00 },
	{ 0x0250, 0x00 },
	{ 0x0251, 0x00 },
	{ 0x0252, 0x00 },
	{ 0x0253, 0x00 },
	{ 0x0254, 0x00 },
	{ 0x0255, 0x00 },
	{ 0x0256, 0x00 },
	{ 0x0257, 0x00 },
	{ 0x0258, 0x00 },
	{ 0x0259, 0x00 },
	{ 0x025A, 0x00 },
	{ 0x025B, 0x00 },
	{ 0x025C, 0x00 },
	{ 0x025D, 0x00 },
	{ 0x025E, 0x00 },
	{ 0x025F, 0x00 },
	{ 0x0260, 0x00 },
	{ 0x0261, 0x00 },
	{ 0x0262, 0x00 },
	{ 0x0263, 0x00 },
	{ 0x0264, 0x00 },
	{ 0x0268, 0x00 },
	{ 0x0269, 0x00 },
	{ 0x026A, 0x00 },
	{ 0x026B, 0x50 },
	{ 0x026C, 0x48 },
	{ 0x026D, 0x4F },
	{ 0x026E, 0x45 },
	{ 0x026F, 0x4E },
	{ 0x0270, 0x49 },
	{ 0x0271, 0x58 },
	{ 0x0272, 0x00 },
	{ 0x028A, 0x00 },
	{ 0x028B, 0x00 },
	{ 0x028C, 0x00 },
	{ 0x028D, 0x00 },
	{ 0x028E, 0x00 },
	{ 0x028F, 0x00 },
	{ 0x0290, 0x00 },
	{ 0x0291, 0x00 },
	{ 0x0294, 0x80 },
	{ 0x0296, 0x00 },
	{ 0x0297, 0x00 },
	{ 0x0299, 0x00 },
	{ 0x029D, 0x00 },
	{ 0x029E, 0x00 },
	{ 0x029F, 0x00 },
	{ 0x02A9, 0x00 },
	{ 0x02AA, 0x00 },
	{ 0x02AB, 0x00 },
	{ 0x02B7, 0xFF },

	{ 0x0302, 0x00 },
	{ 0x0303, 0x00 },
	{ 0x0304, 0x00 },
	{ 0x0305, 0x00 },
	{ 0x0306, 0x21 },
	{ 0x0307, 0x00 },
	{ 0x0308, 0x00 },
	{ 0x0309, 0x00 },
	{ 0x030A, 0x00 },
	{ 0x030B, 0x80 },
	{ 0x030C, 0x00 },
	{ 0x030D, 0x00 },
	{ 0x030E, 0x00 },
	{ 0x030F, 0x00 },
	{ 0x0310, 0x00 },
	{ 0x0311, 0x00 },
	{ 0x0312, 0x00 },
	{ 0x0313, 0x00 },
	{ 0x0314, 0x00 },
	{ 0x0315, 0x00 },
	{ 0x0316, 0x00 },
	{ 0x0317, 0x00 },
	{ 0x0318, 0x00 },
	{ 0x0319, 0x00 },
	{ 0x031A, 0x00 },
	{ 0x031B, 0x00 },
	{ 0x031C, 0x00 },
	{ 0x031D, 0x00 },
	{ 0x031E, 0x00 },
	{ 0x031F, 0x00 },
	{ 0x0320, 0x00 },
	{ 0x0321, 0x00 },
	{ 0x0322, 0x00 },
	{ 0x0323, 0x00 },
	{ 0x0324, 0x00 },
	{ 0x0325, 0x00 },
	{ 0x0326, 0x00 },
	{ 0x0327, 0x00 },
	{ 0x0328, 0x00 },
	{ 0x0329, 0x00 },
	{ 0x032A, 0x00 },
	{ 0x032B, 0x00 },
	{ 0x032C, 0x00 },
	{ 0x032D, 0x00 },
	{ 0x032E, 0x00 },
	{ 0x032F, 0x00 },
	{ 0x0330, 0x00 },
	{ 0x0331, 0x00 },
	{ 0x0332, 0x00 },
	{ 0x0333, 0x00 },
	{ 0x0334, 0x00 },
	{ 0x0335, 0x00 },
	{ 0x0336, 0x00 },
	{ 0x0337, 0x00 },
	{ 0x0338, 0x00 },
	{ 0x0339, 0x1F },
	{ 0x033B, 0x00 },
	{ 0x033C, 0x00 },
	{ 0x033D, 0x00 },
	{ 0x033E, 0x00 },
	{ 0x033F, 0x00 },
	{ 0x0340, 0x00 },
	{ 0x0341, 0x00 },
	{ 0x0342, 0x00 },
	{ 0x0343, 0x00 },
	{ 0x0344, 0x00 },
	{ 0x0345, 0x00 },
	{ 0x0346, 0x00 },
	{ 0x0347, 0x00 },
	{ 0x0348, 0x00 },
	{ 0x0349, 0x00 },
	{ 0x034A, 0x00 },
	{ 0x034B, 0x00 },
	{ 0x034C, 0x00 },
	{ 0x034D, 0x00 },
	{ 0x034E, 0x00 },
	{ 0x034F, 0x00 },
	{ 0x0350, 0x00 },
	{ 0x0351, 0x00 },
	{ 0x0352, 0x00 },
	{ 0x0353, 0x00 },
	{ 0x0354, 0x00 },
	{ 0x0355, 0x00 },
	{ 0x0356, 0x00 },
	{ 0x0357, 0x00 },
	{ 0x0358, 0x00 },
	{ 0x0359, 0x00 },
	{ 0x035A, 0x00 },
	{ 0x035B, 0x00 },
	{ 0x035C, 0x00 },
	{ 0x035D, 0x00 },
	{ 0x035E, 0x00 },
	{ 0x035F, 0x00 },
	{ 0x0360, 0x00 },
	{ 0x0361, 0x00 },
	{ 0x0362, 0x00 },

	{ 0x0487, 0x00 },

	{ 0x0508, 0x00 },
	{ 0x0509, 0x00 },
	{ 0x050A, 0x00 },
	{ 0x050B, 0x00 },
	{ 0x050C, 0x00 },
	{ 0x050D, 0x00 },
	{ 0x050E, 0x00 },
	{ 0x050F, 0x00 },
	{ 0x0510, 0x00 },
	{ 0x0511, 0x00 },
	{ 0x0512, 0x00 },
	{ 0x0513, 0x00 },
	{ 0x0515, 0x00 },
	{ 0x0516, 0x00 },
	{ 0x0517, 0x00 },
	{ 0x0518, 0x00 },
	{ 0x0519, 0x00 },
	{ 0x051A, 0x00 },
	{ 0x051B, 0x00 },
	{ 0x051C, 0x00 },
	{ 0x051D, 0x00 },
	{ 0x051E, 0x00 },
	{ 0x051F, 0x00 },
	{ 0x0521, 0x2B },
	{ 0x052A, 0x00 },
	{ 0x052B, 0x01 },
	{ 0x052C, 0x0F },
	{ 0x052D, 0x03 },
	{ 0x052E, 0x00 },
	{ 0x052F, 0x00 },
	{ 0x0531, 0x00 },
	{ 0x0532, 0x00 },
	{ 0x0533, 0x04 },
	{ 0x0534, 0x00 },
	{ 0x0535, 0x01 },
	{ 0x0536, 0x06 },
	{ 0x0537, 0x00 },
	{ 0x0538, 0x00 },
	{ 0x0539, 0x00 },
	{ 0x053D, 0x0A },
	{ 0x053E, 0x06 },
	{ 0x0589, 0x0C },
	{ 0x058A, 0x00 },
	{ 0x059B, 0x18 },
	{ 0x059D, 0x00 },
	{ 0x059E, 0x00 },
	{ 0x059F, 0x00 },
	{ 0x05A0, 0x00 },
	{ 0x05A1, 0x00 },
	{ 0x05A2, 0x00 },
	{ 0x05A6, 0x00 },

	/* ? */
	{ 0x0802, 0x35 },
	{ 0x0803, 0x05 },
	{ 0x0804, 0x01 },
	{ 0x0805, 0x00 },
	{ 0x0806, 0x00 },
	{ 0x0807, 0x00 },
	{ 0x0808, 0x00 },
	{ 0x0809, 0x00 },
	{ 0x080A, 0x00 },
	{ 0x080B, 0x00 },
	{ 0x080C, 0x00 },
	{ 0x080D, 0x00 },
	{ 0x080E, 0x00 },
	{ 0x080F, 0x00 },
	{ 0x0810, 0x00 },
	{ 0x0811, 0x00 },
	{ 0x0812, 0x00 },
	{ 0x0813, 0x00 },
	{ 0x0814, 0x00 },
	{ 0x0815, 0x00 },
	{ 0x0816, 0x00 },
	{ 0x0817, 0x00 },
	{ 0x0818, 0x00 },
	{ 0x0819, 0x00 },
	{ 0x081A, 0x00 },
	{ 0x081B, 0x00 },
	{ 0x081C, 0x00 },
	{ 0x081D, 0x00 },
	{ 0x081E, 0x00 },
	{ 0x081F, 0x00 },
	{ 0x0820, 0x00 },
	{ 0x0821, 0x00 },
	{ 0x0822, 0x00 },
	{ 0x0823, 0x00 },
	{ 0x0824, 0x00 },
	{ 0x0825, 0x00 },
	{ 0x0826, 0x00 },
	{ 0x0827, 0x00 },
	{ 0x0828, 0x00 },
	{ 0x0829, 0x00 },
	{ 0x082A, 0x00 },
	{ 0x082B, 0x00 },
	{ 0x082C, 0x00 },
	{ 0x082D, 0x00 },
	{ 0x082E, 0x00 },
	{ 0x082F, 0x00 },
	{ 0x0830, 0x00 },
	{ 0x0831, 0x00 },
	{ 0x0832, 0x00 },
	{ 0x0833, 0x00 },
	{ 0x0834, 0x00 },
	{ 0x0835, 0x00 },
	{ 0x0836, 0x00 },
	{ 0x0837, 0x00 },
	{ 0x0838, 0x00 },
	{ 0x0839, 0x00 },
	{ 0x083A, 0x00 },
	{ 0x083B, 0x00 },
	{ 0x083C, 0x00 },
	{ 0x083D, 0x00 },
	{ 0x083E, 0x00 },
	{ 0x083F, 0x00 },
	{ 0x0840, 0x00 },
	{ 0x0841, 0x00 },
	{ 0x0842, 0x00 },
	{ 0x0843, 0x00 },
	{ 0x0844, 0x00 },
	{ 0x0845, 0x00 },
	{ 0x0846, 0x00 },
	{ 0x0847, 0x00 },
	{ 0x0848, 0x00 },
	{ 0x0849, 0x00 },
	{ 0x084A, 0x00 },
	{ 0x084B, 0x00 },
	{ 0x084C, 0x00 },
	{ 0x084D, 0x00 },
	{ 0x084E, 0x00 },
	{ 0x084F, 0x00 },
	{ 0x0850, 0x00 },
	{ 0x0851, 0x00 },
	{ 0x0852, 0x00 },
	{ 0x0853, 0x00 },
	{ 0x0854, 0x00 },
	{ 0x0855, 0x00 },
	{ 0x0856, 0x00 },
	{ 0x0857, 0x00 },
	{ 0x0858, 0x00 },
	{ 0x0859, 0x00 },
	{ 0x085A, 0x00 },
	{ 0x085B, 0x00 },
	{ 0x085C, 0x00 },
	{ 0x085D, 0x00 },
	{ 0x085E, 0x00 },
	{ 0x085F, 0x00 },
	{ 0x0860, 0x00 },
	{ 0x0861, 0x00 },

	/* XAXB */
	{ 0x090E, 0x02 },
	{ 0x0943, 0x00 },
	{ 0x0949, 0x00 },
	{ 0x094A, 0x00 },
	{ 0x094E, 0x49 },
	{ 0x094F, 0x02 },
	{ 0x095E, 0x00 },

	{ 0x0A02, 0x00 },
	{ 0x0A03, 0x01 },
	{ 0x0A04, 0x01 },
	{ 0x0A05, 0x01 },
	{ 0x0A14, 0x00 },
	{ 0x0A1A, 0x00 },
	{ 0x0A20, 0x00 },
	{ 0x0A26, 0x00 },
	{ 0x0A2C, 0x00 },

	{ 0x0B44, 0x0F },
	{ 0x0B46, 0x00 },
	{ 0x0B47, 0x0F },
	{ 0x0B48, 0x0F },
	{ 0x0B4A, 0x1E },
	{ 0x0B57, 0x03 },
	{ 0x0B58, 0x01 },
};

/* Sequence used to end si5345 configuration */
si5345_revd_register_t const si5345_end[] =
{
	{ 0x0514, 0x01 },
	{ 0x001C, 0x01 },
	{ 0x0540, 0x00 },
	{ 0x0B24, 0xC3 },
	{ 0x0B25, 0x02 },
};

static int si5345_readPage(uint8_t *page)
{
    return i2c_regRead(SI5345_I2C_ADDRESS, SI5345_PAGE_ADDRESS, page, 1);
}

static int si5345_setPage(const uint8_t *page)
{
	uint8_t page_set_req[2] = {0x01, *page};
	return i2c_busWrite(SI5345_I2C_ADDRESS, page_set_req, 2);
}

static int si5345_setReg(const si5345_revd_register_t* const reg)
{
    /* Read current page */
    uint8_t current_page = 0;
    int ret = si5345_readPage(&current_page);
    if (ret != 0) {
        printf("fsbl2: fail to read i2c page address\n");
        return ret;
    }

	/* Make sure page is correct */
	const uint8_t desired_page = (uint8_t)((reg->address & 0x0F00) >> 8);
	if (current_page != desired_page) {
        ret = si5345_setPage(&desired_page);
        if (ret != 0) {
            printf("fsbl2: fail to write i2c page address\n");
            return ret;
        }
	}

	/* Write register address */
	const uint8_t u8_address = (reg->address & 0xFF);
	uint8_t register_write_data[2] = {u8_address, reg->value};
	ret = i2c_busWrite(SI5345_I2C_ADDRESS, register_write_data, 2);
	if (ret != 0) {
		printf("fsbl2: fail to write i2c reg value, error: %i\n", ret);
	}

    return ret;
}

static int si5345_upload_config(void)
{
    int ret = 0;

	printf("fsbl2: si5345 config start...\n");
	const uint32_t start_len = (sizeof(si5345_start) / sizeof(si5345_start[0]));
	for (uint32_t i = 0; i < start_len; i++) {
		ret = si5345_setReg(&si5345_start[i]);
        if (ret != 0) {
            printf("fsbl2: failed to upload i2c register\n");
            return ret;
        }
	}
	usleep(350 * 1000);

    printf("fsbl2: si5345 config upload...\n");
	const uint32_t config_len = (sizeof(si5345_config) / sizeof(si5345_config[0]));
	for (uint32_t i = 0; i < config_len; i++) {
		ret = si5345_setReg(&si5345_config[i]);
        if (ret != 0) {
            printf("fsbl2: failed to upload i2c register\n");
            return ret;
        }
	}

    printf("fsbl2: si5345 config finish...\n");
	const uint32_t end_len = (sizeof(si5345_end) / sizeof(si5345_end[0]));
	for (uint32_t i = 0; i < end_len; i++) {
		ret = si5345_setReg(&si5345_end[i]);
        if (ret != 0) {
            printf("fsbl2: failed to upload i2c register\n");
            return ret;
        }
	}

    return ret;
}

int si5345_initPcieClk(void)
{
    /* Init I2C 0 */
	i2c_init(0);

    /* Disable all U16 I2C multiplexer channels */
	uint8_t data = 0;
    int ret = i2c_busWrite(0x73, &data, 1);
    if (ret != 0) {
        printf("fsbl2: fail to config i2c multiplexer: %i\n", ret);
        return ret;
    }

	/* Enable only one channel of U27: PLL chip */
	data = (1 << 4);
	ret = i2c_busWrite(0x77, &data, 1);
    if (ret != 0) {
        printf("fsbl2: fail to config i2c multiplexer: %i\n", ret);
        return ret;
    }

    /* Read SI5345 whoami */
    uint8_t whoami_page = 0;
	(void)si5345_setPage(&whoami_page);
	uint8_t chip_id[4] = {0};
	ret = i2c_regRead(0x69, 0x02, chip_id, 4);
    if (ret != 0) {
        printf("fsbl2: fail to read si5345 whoami: %i\n", ret);
        return ret;
    }
    else {
        printf("fsbl2: si5345 chip id = 0x%x 0x%x 0x%x 0x%x \n",
            chip_id[0], chip_id[1], chip_id[2], chip_id[3]);
    }
    if ((chip_id[0] == 0x45) && (chip_id[1] == 0x53)) {
        printf("fsbl2: si5345 chip id valid\n");
    }
    else {
        printf("fsbl2: si5345 chip id invalid\n");
    }

    /* Configure SI5345 */
	return si5345_upload_config();
}

static int pcie_initClock(void)
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

static int pcie_deassertAllResets(void)
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
		printf("pcie: fail to deassert GT reset\n");
		return ret;
	}

	platformctl_t ctl2 = {
		.action = pctl_set,
		.type = pctl_devreset,
		.devreset.dev = pctl_devreset_fpd_pcie_ctrl,
		.devreset.state = 0,
	};
	ret = platformctl(&ctl2);
	if (ret != 0) {
		printf("pcie: fail to deassert PCIE CTRL reset\n");
		return ret;
	}

	platformctl_t ctl3 = {
		.action = pctl_set,
		.type = pctl_devreset,
		.devreset.dev = pctl_devreset_fpd_pcie_bridge,
		.devreset.state = 0,
	};
	ret = platformctl(&ctl3);
	if (ret != 0) {
		printf("pcie: fail to deassert PCIE BRIDGE reset\n");
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
		printf("pcie: fail to deassert PCIE CFG reset\n");
		return ret;
	}

	return ret;
}

static int pcie_assertReset(void)
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
		printf("pcie: fail to assert PCIE CTRL clock\n");
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
		printf("pcie: fail to assert PCIE BRIDGE clock\n");
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
		printf("pcie: fail to assert PCIE CFG clock\n");
		return ret;
	}

	return ret;
}

static uint32_t readReg(uint32_t *base, uint32_t offset)
{
	return *((volatile uint32_t *)((char *)base + offset));
}

static void writeReg(uint32_t *base, uint32_t offset, uint32_t value)
{
	*((volatile uint32_t *)((char *)base + offset)) = value;
}

static void writeRegMsk(uint32_t *base, uint32_t offset, uint32_t clr, uint32_t set)
{
	uint32_t value = readReg(base, offset);
	value &= ~clr;
	value |= set;
	writeReg(base, offset, value);
}

static int pcitest_configPsGtr(void)
{
	/* Map SERDES registers memory */
	uint32_t *serdes = mmap(NULL, SERDES_SIZE,
			PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
			-1,
			SERDES_ADDRESS);
	if (NULL == serdes) {
		printf("pcietest: fail to map SERDES registers memory\n");
		return -1;
	}

	/* "Enable coarse saturation" */
	writeReg(serdes, L0_TM_PLL_DIG_37, 0x10);

	/* Set external reference clock equal 100MHz */
	writeRegMsk(serdes, PLL_REF_SEL0, 0x1f, 0x0d);

	/* Set that clock is not shared among 4 transceivers */
	writeRegMsk(serdes, L0_L0_REF_CLK_SEL, 0x9f, (1 << 7));

	/* Continue configuring PLL  */
	uint32_t step_size = 87533;
	writeRegMsk(serdes, L0_PLL_SS_STEP_SIZE_0_LSB, 0xff, step_size & 0xff);
	step_size >>= 8;
	writeRegMsk(serdes, L0_PLL_SS_STEP_SIZE_1, 0xff, step_size & 0xff);
	step_size >>= 8;
	writeRegMsk(serdes, L0_PLL_SS_STEP_SIZE_2, 0xff, step_size & 0xff);
	uint32_t steps = 1058;
	writeRegMsk(serdes, L0_PLL_SS_STEPS_0_LSB, 0xff, steps & 0xff);
	writeRegMsk(serdes, L0_PLL_SS_STEPS_1_MSB, 0x07, (steps >> 8) & 0x07);
	step_size >>= 8;
	writeRegMsk(serdes, L0_PLL_SS_STEP_SIZE_3_MSB, 0x3, (step_size & 0x3) | 0x10 | 0x20);

	/* Set PCIe protocol for PS GTR lane 0 */
	writeRegMsk(serdes, ICM_CFG0, 0x07, 0x1);

	/* Wait for PLL lock */
	usleep(10 * 1000);

	/* Read PLL status */
	uint32_t pll_status = readReg(serdes, 0x23E4);
	if (pll_status & 0x10) {
		printf("pcietest: PLL locked, status 0x%x\n", pll_status);
	}
	else {
		printf("pcietest: fail to lock PLL, status 0x%x\n", pll_status);
	}

	/* Unmap SERDES memory */
	munmap((void *)serdes, SERDES_SIZE);

	return 0;
}

static int pcietest_pciIntxIrqHandler(unsigned int id, void *arg)
{
	return 0;
}

static int pcietest_initPcie(void)
{
	/* Map registers memory */
	pcie.breg = mmap(NULL, BREG_SIZE,
		PROT_WRITE | PROT_READ,
		MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
		-1,
		BREG_ADDRESS);
	if (NULL == pcie.breg) {
		printf("pcietest: fail to map AXI PCIE MAIN registers memory\n");
		return -1;
	}
	pcie.pcireg = mmap(NULL, PCIREG_SIZE,
		PROT_WRITE | PROT_READ,
		MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
		-1,
		PCIREG_ADDRESS);
	if (NULL == pcie.pcireg) {
		printf("pcietest: fail to map PCIE ATTRIB registers memory\n");
		return -1;
	}
	pcie.cfg = mmap(NULL, CFG_SIZE,
		PROT_WRITE | PROT_READ,
		MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
		-1,
		CFG_ADDRESS);
	if (NULL == pcie.pcireg) {
		printf("pcietest: fail to map PCIE CFG registers memory\n");
		return -1;
	}

	/* Initialize synchronization primitives */
	if (condCreate(&pcie.cond) != 0) {
		printf("pcietest: failed to create conditional variable\n");
		return -1;
	}
	if (mutexCreate(&pcie.lock) != 0) {
		printf("pcietest: failed to create mutex\n");
		return -1;
	}

	/* Register PCI INTx interrupt handler */
	if (interrupt(GIC_PCIE_INTX_IRQ, pcietest_pciIntxIrqHandler, &pcie, pcie.cond, &pcie.inth) != 0) {
		printf("pcietest: failed to register interrupt handler\n");
		return -1;
	}

	usleep(1000);

	/* Check if BREG is present */
	uint32_t breg_value = readReg(pcie.breg, E_BREG_CAPABILITIES);
	if (!(breg_value & 0x1)) {
		printf("pcietest: fail lack of Egress Bridge Register Translation\n");
		return -1;
	}

	uint32_t ltsm = (readReg(pcie.pcireg, 0x228) >> 3);
	printf("pcietest: ltsm 0x%x\n", ltsm);

	/* Map the bridge register aperture */
	writeReg(pcie.breg, E_BREG_BASE_LO, BREG_ADDRESS);
	writeReg(pcie.breg, E_BREG_BASE_HI, 0x0);

	/* Enable BREG */
	writeReg(pcie.breg, E_BREG_CONTROL, 0x1);

	ltsm = (readReg(pcie.pcireg, 0x228) >> 3);
	printf("pcietest: ltsm 0x%x\n", ltsm);

	/* Disable DMA */
	writeRegMsk(pcie.breg, BRCFG_PCIE_RX0, 0x7, 0x7);

	/* Enable Ingress subtractive decode translation */
	writeReg(pcie.breg, I_ISUB_CONTROL, 0x1);

	/* Enable msg filtering details */
	writeReg(pcie.breg, BRCFG_PCIE_RX_MSG_FILTER, ((1 << 1) | (1 << 2) | (1 << 3)));

	/* This routes the PCIe DMA traffic to go through CCI path (?) */
	writeRegMsk(pcie.breg, BRCFG_PCIE_RX1, 0xff, 0xff);

	/* Check if the phy link is up or not */
	bool phy_link_up = false;
	static const int LINK_WAIT_MAX_RETRIES = 10;
	for (int retries = 0; retries < LINK_WAIT_MAX_RETRIES; retries++) {
		uint32_t link_status = readReg(pcie.pcireg, PS_LINKUP_OFFSET);
		if (link_status & 0x2) {
			phy_link_up = true;
			break;
		}
		usleep(100000);
	}

	if (phy_link_up) {
		printf("pcietest: phy link up\n");
	}
	else {
		printf("pcietest: fail, phy link down\n");
		return -1;
	}

	uint32_t ecam_value = readReg(pcie.breg, E_ECAM_CAPABILITIES);
	if (ecam_value & 0x1) {
		printf("pcietest: ecam present\n");
	}
	else {
		printf("pcietest: fail ecam is not present\n");
		return -1;
	}

	/* Enable ECAM */
	writeRegMsk(pcie.breg, E_ECAM_CONTROL, 0x1, 0x1);
	/* Set size of translation window */
	writeRegMsk(pcie.breg, E_ECAM_CONTROL, (0x1f << 16), (16 << 16)); // 256 MB

	writeReg(pcie.breg, E_ECAM_BASE_LO, lower_32_bits(CFG_ADDRESS));
	writeReg(pcie.breg, E_ECAM_BASE_HI, upper_32_bits(CFG_ADDRESS));


	/* Check link status in loop */
    uint32_t ltsm_previous = 0;
    for(uint32_t i = 0; i < 100000; i++) {
        uint32_t ltsm = (readReg(pcie.pcireg, 0x228) >> 3);
        uint32_t pcie_link_status = readReg(pcie.pcireg, PS_LINKUP_OFFSET);
        if (ltsm != ltsm_previous) {
            printf("status: ltsm 0x%x link status 0x%x\n", ltsm, pcie_link_status);
        }
        ltsm_previous = ltsm;
        usleep(1 * 100);
    }



#if 0
	/* Check link status */
	bool pcie_link_up = false;
	for (int retries = 0; retries < LINK_WAIT_MAX_RETRIES; retries++) {
		uint32_t pcie_link_status = readReg(pcie.pcireg, PS_LINKUP_OFFSET);
		if (pcie_link_status & 0x1) {
			pcie_link_up = true;
			break;
		}
		usleep(100000);
	}

	ltsm = (readReg(pcie.pcireg, 0x228) >> 3);
	printf("pcietest: ltsm 0x%x\n", ltsm);

	if (pcie_link_up) {
		printf("pcietest: pcie link up\n");
	}
	else {
		printf("pcietest: fail pcie link down\n");
		return -1;
	}
#endif
	/* Unmap memory */
	munmap((void *)pcie.breg, BREG_SIZE);
	munmap((void *)pcie.pcireg, PCIREG_SIZE);

	return 0;
}

int main(int argc, char **argv)
{
    pcie_initClock();

	printf("fsbl2: deassert reset on all related peripherals\n");
	int ret = pcie_deassertAllResets();
	if (ret != 0) {
		printf("fsbl: fail to deassert reset on all related peripherals\n");
		return ret;
	}

	printf("fsbl2: assert reset on all 3 PCIE peripherals\n");
	ret = pcie_assertReset();
	if (ret != 0) {
		printf("fsbl: fail to assert reset on all 3 PCIE peripherals\n");
		return ret;
	}

    (void)fsbl2_integration_init();

    xil_printf("fsbl2: initialising PS GTR SERDES and PCIE modules\n");

    // PCIe reset
    u32 dataVal = 0;
    /* Set MIO31 direction as output */
    XFsbl_Out32(GPIO_DIRM_1, XFsbl_In32(GPIO_DIRM_1) | GPIO_MIO31_MASK);
    /* Set MIO31 output enable */
    XFsbl_Out32(GPIO_OEN_1, XFsbl_In32(GPIO_OEN_1) | GPIO_MIO31_MASK);
    /* Set MIO31 to LOW */
    dataVal = XFsbl_In32(GPIO_DATA_1) & ~(GPIO_MIO31_MASK);
    XFsbl_Out32(GPIO_DATA_1, dataVal);

	if (si5345_initPcieClk() != 0) {
		fprintf(stderr, "fsbl2: failed to configure\n");
		return EXIT_FAILURE;
	}
	else {
		printf("fsbl2: configured successfully\n");
	}

    /* Set MIO31 to HIGH */
    dataVal = XFsbl_In32(GPIO_DATA_1) | GPIO_MIO31_MASK;
    XFsbl_Out32(GPIO_DATA_1, dataVal);
    xil_printf("fsbl2: PCIe Reset Complete\n");

    /* wait after initialization*/
    (void)usleep(DELAY_5_US);

    (void)init_serdes();

	/* Configure PS GTR transceivers */
	printf("pcietest: configure PS GTR transceivers...\n");
	ret = pcitest_configPsGtr();
	if (ret != 0) {
		printf("pcietest: fail to configure PS GTR transceivers\n");
		return ret;
	}

	/* Finaly configure PCI peripheral */
	printf("pcietest: configure PCI Express peripheral...\n");
	ret = pcietest_initPcie();
	if (ret != 0) {
		printf("pcietest: fail to configure PCI Express peripheral\n");
		return ret;
	}

	return 0;
}
