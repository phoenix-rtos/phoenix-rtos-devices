/*
 * Phoenix-RTOS
 *
 * ADE9113 register and field definitions
 *
 * NOTE: This file was generated from datasheet using internal tools. See
 * (non-public) devtools repo for details. To be determined what is the
 * correct way to upstream those tools and source data into public repo.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jan Wiśniewski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


/* clang-format off */

#ifndef ADE9113_REGS_H
#define ADE9113_REGS_H

#include <stdint.h>

/* ------------------------------------------------------------------------ */
/* Register addresses                                                   */
/* ------------------------------------------------------------------------ */

#define ADE9113_SWRST              0x001  /* Software Reset. */
#define ADE9113_CONFIG0            0x002  /* ADC Configuration. */
#define ADE9113_CONFIG_FILT        0x003  /* Digital Filter Configuration. */
#define ADE9113_CONFIG_ISO_ACC     0x005  /* Enable Access to Isolated (ISO) Register Space. The ISO side is Pin 1 to Pin 14. */
#define ADE9113_CRC_RESULT_HI      0x006  /* Background Register Map CRC Most Significant Byte. */
#define ADE9113_CRC_RESULT_LO      0x007  /* Background Register Map CRC Least Significant Byte. */
#define ADE9113_EFUSE_REFRESH      0x008  /* eFuse Refresh. */
#define ADE9113_EMI_CONFIG         0x009  /* Configure Isolation Frequency Hopping Method. */
#define ADE9113_EMI_HI_MASK        0x00A  /* Emissions Mask, High Frequency Bounds. */
#define ADE9113_EMI_LO_MASK        0x00B  /* Emissions Mask, Low Frequency Bounds. */
#define ADE9113_EMI_HI_LIMIT       0x00C  /* Factory Stored High Frequency Limit. Corresponds to emissions at 420 MHz. */
#define ADE9113_EMI_MID_LIMIT      0x00D  /* Factory Stored Center Frequency Value. Corresponds to emissions at 360 MHz. */
#define ADE9113_EMI_LO_LIMIT       0x00E  /* Factory Stored Low Frequency Limit. Corresponds to emissions at 300 MHz. */
#define ADE9113_MASK0              0x00F  /* Interrupt Mask 0 Register. Mask register for STATUS0. */
#define ADE9113_MASK1              0x010  /* Interrupt Mask 1 Register. Mask register for STATUS1. */
#define ADE9113_MASK2              0x011  /* Interrupt Mask 2 Register. Mask register for STATUS2. */
#define ADE9113_CONFIG_ZX          0x012  /* Zero-Crossing Configuration. */
#define ADE9113_SCRATCH            0x013  /* Software Debug Register for Testing SPI R/W. */
#define ADE9113_SYNC_SNAP          0x014  /* ADC Synchronization Register. */
#define ADE9113_SNAPSHOT_COUNT_HI  0x017  /* System Timing Controller Counter Most Significant Byte. */
#define ADE9113_SNAPSHOT_COUNT_LO  0x018  /* System Timing Controller Counter Least Significant Byte. */
#define ADE9113_WR_LOCK            0x01F  /* Configuration Lock Register. */
#define ADE9113_STATUS0            0x020  /* Latched Status of High Priority Interrupts. */
#define ADE9113_STATUS1            0x021  /* Latched Status of Low Priority Interrupts. */
#define ADE9113_STATUS2            0x022  /* Latched Status of Isolated ADC Interrupts. */
#define ADE9113_COM_FLT_TYPE       0x023  /* ISO to NONISO Communications Fault Type. */
#define ADE9113_COM_FLT_COUNT      0x024  /* ISO to NONISO Communications Fault Count. */
#define ADE9113_CONFIG_CRC         0x025  /* Configuration of Background Register Map CRC. */
#define ADE9113_I_WAV_HI           0x026  /* Current Channel Waveform Data Most Significant Byte. */
#define ADE9113_I_WAV_MD           0x027  /* Current Channel Waveform Data Middle Byte. */
#define ADE9113_I_WAV_LO           0x028  /* Current Channel Waveform Data Least Significant Byte. */
#define ADE9113_V1_WAV_HI          0x029  /* V1 Channel Waveform Data Most Significant Byte. */
#define ADE9113_V1_WAV_MD          0x02A  /* V1 Channel Waveform Data Middle Byte. */
#define ADE9113_V1_WAV_LO          0x02B  /* V1 Channel Waveform Data Least Significant Byte. */
#define ADE9113_V2_WAV_HI          0x02C  /* V2 Channel Waveform Data Most Significant Byte. */
#define ADE9113_V2_WAV_MD          0x02D  /* V2 Channel Waveform Data Middle Byte. */
#define ADE9113_V2_WAV_LO          0x02E  /* V2 Channel Waveform Data Least Significant Byte. */
#define ADE9113_UNIQUE_PART_ID_5   0x075  /* Unique Part ID Most Significant Byte (Byte 5). */
#define ADE9113_UNIQUE_PART_ID_4   0x076  /* Unique Part ID Byte 4. */
#define ADE9113_UNIQUE_PART_ID_3   0x077  /* Unique Part ID Byte 3. */
#define ADE9113_UNIQUE_PART_ID_2   0x078  /* Unique Part ID Byte 2. */
#define ADE9113_UNIQUE_PART_ID_1   0x079  /* Unique Part ID Byte 1. */
#define ADE9113_UNIQUE_PART_ID_0   0x07A  /* Unique Part ID Least Significant Byte (Byte 0). */
#define ADE9113_SILICON_REVISION   0x07D  /* Revision Value of ISO and NONISO Silicon. */
#define ADE9113_VERSION_PRODUCT    0x07E  /* Product Version Identifier. */
#define ADE9113_DC_OFFSET_MODE     0x0CC  /* Enable the Current Channel Input Short. */

/* ------------------------------------------------------------------------ */
/* Field enumerations                                                   */
/* ------------------------------------------------------------------------ */

enum ade9113_swrst {
	ADE9113_SWRST__SOFTWARE_RESET_COMMAND = 0xD6,  /* Software Reset Command. The software resets all registers to their default values. Power cycling the device may be required to correct hardware functionality. */
};

enum ade9113_config0_stream_dbg {
	ADE9113_CONFIG0_STREAM_DBG__NORMAL_MODE = 0x00,  /* Normal Mode. The x_WAV_x registers contain the ADC results. */
	ADE9113_CONFIG0_STREAM_DBG__STATIC_MODE = 0x01,  /* Static Mode. The x_WAV_x registers are static and hold their value. The x_WAV_x registers can be written to change to a new value. The x_WAV_x registers become static and hold their value until a register write is performed to the x_WAV_x register with a new value. Programming must first be enabled with the count mode. setting before this static mode functions. */
	ADE9113_CONFIG0_STREAM_DBG__COUNT_MODE  = 0x02,  /* Count Mode. Data Increments at ADC Conversion Rate. Enables write access to x_WAV_x registers and increments the x_WAV_x register with every DREADY pulse. */
	ADE9113_CONFIG0_STREAM_DBG__RESERVED    = 0x03,  /* Reserved. Same operation as normal mode. */
};

enum ade9113_config_filt_lpf_bw {
	ADE9113_CONFIG_FILT_LPF_BW__BANDWIDTH_2K7 = 0x00,  /* Bandwidth of 2.7 kHz at 8 kSPS output data rate. */
	ADE9113_CONFIG_FILT_LPF_BW__BANDWIDTH_3K3 = 0x01,  /* Bandwidth of 3.3 kHz at 8 kSPS output data rate. */
};

enum ade9113_config_filt_datapath_config {
	ADE9113_CONFIG_FILT_DATAPATH_CONFIG__32_KHZ          = 0x00,  /* Sinc3, 32 kHz Sampling. */
	ADE9113_CONFIG_FILT_DATAPATH_CONFIG__LPF_32_KHZ      = 0x01,  /* Sinc3, LPF Enabled, 32 kHz Sampling. */
	ADE9113_CONFIG_FILT_DATAPATH_CONFIG__COMP_LPF_32_KHZ = 0x02,  /* Sinc3, Compensation Enabled, LPF Enabled, 32 kHz Sampling. */
	ADE9113_CONFIG_FILT_DATAPATH_CONFIG__LPF_8_KHZ       = 0x03,  /* Sinc3, LPF Enabled, 8 kHz Sampling. */
	ADE9113_CONFIG_FILT_DATAPATH_CONFIG__COMP_LPF_8_KHZ  = 0x04,  /* Sinc3, Compensation Enabled, LPF Enabled, 8 kHz Sampling. */
	ADE9113_CONFIG_FILT_DATAPATH_CONFIG__LPF_4_KHZ       = 0x05,  /* Sinc3, LPF Enabled, 4 kHz Sampling. */
	ADE9113_CONFIG_FILT_DATAPATH_CONFIG__LPF_2_KHZ       = 0x06,  /* Sinc3, LPF Enabled, 2 kHz Sampling. */
	ADE9113_CONFIG_FILT_DATAPATH_CONFIG__LPF_1_KHZ       = 0x07,  /* Sinc3, LPF Enabled, 1 kHz Sampling. */
};

enum ade9113_emi_config_emi_config {
	ADE9113_EMI_CONFIG_EMI_CONFIG__SAWTOOTH_FREQUENCY_RISING  = 0x00,  /* Sawtooth Frequency Rising. Frequency starts at frequency defined by EMI_LO_LIMIT and ramps to higher frequency as defined by EMI_HI_LIMIT and then returns to EMI_LO_LIMIT. */
	ADE9113_EMI_CONFIG_EMI_CONFIG__SAWTOOTH_FREQUENCY_FALLING = 0x01,  /* Sawtooth Frequency Falling. Frequency starts at frequency defined by EMI_HI_LIMIT and ramps to a lower frequency as defined by EMI_LO_LIMIT and then returns to EMI_LO_LIMIT. */
	ADE9113_EMI_CONFIG_EMI_CONFIG__RAMP                       = 0x02,  /* Ramp. Linear ramp up and down in frequency between the limits, EMI_LO_LIMIT and EMI_HI_LIMIT. */
	ADE9113_EMI_CONFIG_EMI_CONFIG__RANDOM_HOPPING_FREQUENCY   = 0x03,  /* Random Hopping Frequency. The isolated power oscillator frequency varies +/-63 trim codes around the calibrated center frequency of EMI_MID_LIMIT. */
};

enum ade9113_config_zx_zx_edge_sel {
	ADE9113_CONFIG_ZX_ZX_EDGE_SEL__PASS_SIGN        = 0x00,  /* ZX Pin Reflects the Sign of the Input Signal. The ZX pin goes high on negative to positive ZX and low on positive to negative ZX. */
	ADE9113_CONFIG_ZX_ZX_EDGE_SEL__PULSE_RAISING    = 0x01,  /* Detect Zero Crossings with Positive Slope. When zero crossing from negative to positive, a high pulse duration of 512 µs is generated. */
	ADE9113_CONFIG_ZX_ZX_EDGE_SEL__PULSE_FALLING    = 0x02,  /* Detect Zero Crossings with Negative Slope. When zero crossing from positive to negative, a high pulse duration of 512 µs is generated. */
	ADE9113_CONFIG_ZX_ZX_EDGE_SEL__PULSE_ZERO_CROSS = 0x03,  /* Detect Zero Crossings with Positive or Negative Slopes. Combines 01 and 10. */
};

enum ade9113_config_zx_zx_channel_config {
	ADE9113_CONFIG_ZX_ZX_CHANNEL_CONFIG__ZX_OUT_DISABLE = 0x00,  /* Disable Zero-Crossing Output. */
	ADE9113_CONFIG_ZX_ZX_CHANNEL_CONFIG__ZX_OUT_I       = 0x01,  /* Output Zero-Crossing Function from the Current Channel on the ZX Pin. */
	ADE9113_CONFIG_ZX_ZX_CHANNEL_CONFIG__ZX_OUT_V1      = 0x02,  /* Output Zero-Crossing Function from the V1 Channel on the ZX Pin. */
	ADE9113_CONFIG_ZX_ZX_CHANNEL_CONFIG__ZX_OUT_V2      = 0x03,  /* Output Zero-Crossing Function from the V2 Channel on the ZX Pin. */
};

enum ade9113_wr_lock {
	ADE9113_WR_LOCK__LOCK_KEY   = 0xD4,  /* Lock Key. If register map is locked, the lock key value can be read. */
	ADE9113_WR_LOCK__UNLOCK_KEY = 0x5E,  /* Unlock Key. if the register map is unlocked, any value can be written or read from this location. */
};

enum ade9113_version_product {
	ADE9113_VERSION_PRODUCT__ADE9113 = 0x00,  /* ADE9113. */
	ADE9113_VERSION_PRODUCT__ADE9112 = 0x01,  /* ADE9112. */
	ADE9113_VERSION_PRODUCT__ADE9103 = 0x03,  /* ADE9103. */
};

/* ------------------------------------------------------------------------ */
/* Register structs + encode / decode                                   */
/* ------------------------------------------------------------------------ */

/* ADC Configuration. */
struct ade9113_config0 {
	uint8_t clkout_en;
	uint8_t crc_en_spi_write;
	enum ade9113_config0_stream_dbg stream_dbg;
};

static inline uint8_t ade9113_config0_encode(struct ade9113_config0 v)
{
	return ((v.clkout_en & 0x01) << 0) | ((v.crc_en_spi_write & 0x01) << 1) | ((v.stream_dbg & 0x03) << 2);
}

#define ADE9113_CONFIG0_ENC(...) \
	ade9113_config0_encode((struct ade9113_config0){__VA_ARGS__})

static inline struct ade9113_config0 ade9113_config0_decode(uint8_t r)
{
	return (struct ade9113_config0){
		.clkout_en = (r >> 0) & 0x01,
		.crc_en_spi_write = (r >> 1) & 0x01,
		.stream_dbg = (r >> 2) & 0x03,
	};
}

/* Digital Filter Configuration. */
struct ade9113_config_filt {
	enum ade9113_config_filt_datapath_config datapath_config;
	enum ade9113_config_filt_lpf_bw lpf_bw;
	uint8_t i_adc_invert;
	uint8_t v1_adc_invert;
	uint8_t v2_adc_invert;
};

static inline uint8_t ade9113_config_filt_encode(struct ade9113_config_filt v)
{
	return ((v.datapath_config & 0x07) << 0) | ((v.lpf_bw & 0x01) << 3) | ((v.i_adc_invert & 0x01) << 4) | ((v.v1_adc_invert & 0x01) << 5) | ((v.v2_adc_invert & 0x01) << 6);
}

#define ADE9113_CONFIG_FILT_ENC(...) \
	ade9113_config_filt_encode((struct ade9113_config_filt){__VA_ARGS__})

static inline struct ade9113_config_filt ade9113_config_filt_decode(uint8_t r)
{
	return (struct ade9113_config_filt){
		.datapath_config = (r >> 0) & 0x07,
		.lpf_bw = (r >> 3) & 0x01,
		.i_adc_invert = (r >> 4) & 0x01,
		.v1_adc_invert = (r >> 5) & 0x01,
		.v2_adc_invert = (r >> 6) & 0x01,
	};
}

/* Enable Access to Isolated (ISO) Register Space. The ISO side is Pin 1 to Pin 14. */
struct ade9113_config_iso_acc {
	uint8_t iso_wr_acc_en;
};

static inline uint8_t ade9113_config_iso_acc_encode(struct ade9113_config_iso_acc v)
{
	return ((v.iso_wr_acc_en & 0x01) << 0);
}

#define ADE9113_CONFIG_ISO_ACC_ENC(...) \
	ade9113_config_iso_acc_encode((struct ade9113_config_iso_acc){__VA_ARGS__})

static inline struct ade9113_config_iso_acc ade9113_config_iso_acc_decode(uint8_t r)
{
	return (struct ade9113_config_iso_acc){
		.iso_wr_acc_en = (r >> 0) & 0x01,
	};
}

/* eFuse Refresh. */
struct ade9113_efuse_refresh {
	uint8_t efuse_refresh;
};

static inline uint8_t ade9113_efuse_refresh_encode(struct ade9113_efuse_refresh v)
{
	return ((v.efuse_refresh & 0x01) << 0);
}

#define ADE9113_EFUSE_REFRESH_ENC(...) \
	ade9113_efuse_refresh_encode((struct ade9113_efuse_refresh){__VA_ARGS__})

static inline struct ade9113_efuse_refresh ade9113_efuse_refresh_decode(uint8_t r)
{
	return (struct ade9113_efuse_refresh){
		.efuse_refresh = (r >> 0) & 0x01,
	};
}

/* Configure Isolation Frequency Hopping Method. */
struct ade9113_emi_config {
	enum ade9113_emi_config_emi_config emi_config;
};

static inline uint8_t ade9113_emi_config_encode(struct ade9113_emi_config v)
{
	return ((v.emi_config & 0x07) << 0);
}

#define ADE9113_EMI_CONFIG_ENC(...) \
	ade9113_emi_config_encode((struct ade9113_emi_config){__VA_ARGS__})

static inline struct ade9113_emi_config ade9113_emi_config_decode(uint8_t r)
{
	return (struct ade9113_emi_config){
		.emi_config = (r >> 0) & 0x07,
	};
}

/* Interrupt Mask 0 Register. Mask register for STATUS0. */
struct ade9113_mask0 {
	uint8_t comflt_err;
	uint8_t spi_crc_err;
	uint8_t crc_chg;
	uint8_t com_up;
	uint8_t status2x;
	uint8_t status1x;
};

static inline uint8_t ade9113_mask0_encode(struct ade9113_mask0 v)
{
	return ((v.comflt_err & 0x01) << 0) | ((v.spi_crc_err & 0x01) << 1) | ((v.crc_chg & 0x01) << 3) | ((v.com_up & 0x01) << 4) | ((v.status2x & 0x01) << 6) | ((v.status1x & 0x01) << 7);
}

#define ADE9113_MASK0_ENC(...) \
	ade9113_mask0_encode((struct ade9113_mask0){__VA_ARGS__})

static inline struct ade9113_mask0 ade9113_mask0_decode(uint8_t r)
{
	return (struct ade9113_mask0){
		.comflt_err = (r >> 0) & 0x01,
		.spi_crc_err = (r >> 1) & 0x01,
		.crc_chg = (r >> 3) & 0x01,
		.com_up = (r >> 4) & 0x01,
		.status2x = (r >> 6) & 0x01,
		.status1x = (r >> 7) & 0x01,
	};
}

/* Interrupt Mask 1 Register. Mask register for STATUS1. */
struct ade9113_mask1 {
	uint8_t adc_sync_done;
	uint8_t i_wav_ovrng;
	uint8_t v1_wav_ovrng;
	uint8_t v2_wav_ovrng;
};

static inline uint8_t ade9113_mask1_encode(struct ade9113_mask1 v)
{
	return ((v.adc_sync_done & 0x01) << 0) | ((v.i_wav_ovrng & 0x01) << 1) | ((v.v1_wav_ovrng & 0x01) << 2) | ((v.v2_wav_ovrng & 0x01) << 3);
}

#define ADE9113_MASK1_ENC(...) \
	ade9113_mask1_encode((struct ade9113_mask1){__VA_ARGS__})

static inline struct ade9113_mask1 ade9113_mask1_decode(uint8_t r)
{
	return (struct ade9113_mask1){
		.adc_sync_done = (r >> 0) & 0x01,
		.i_wav_ovrng = (r >> 1) & 0x01,
		.v1_wav_ovrng = (r >> 2) & 0x01,
		.v2_wav_ovrng = (r >> 3) & 0x01,
	};
}

/* Interrupt Mask 2 Register. Mask register for STATUS2. */
struct ade9113_mask2 {
	uint8_t iso_test_mmr_err;
	uint8_t iso_dig_mod_i_ovf;
	uint8_t iso_dig_mod_v1_ovf;
	uint8_t iso_dig_mod_v2_ovf;
	uint8_t iso_efuse_mem_err;
	uint8_t iso_phy_crc_err;
	uint8_t iso_clk_stbl_err;
};

static inline uint8_t ade9113_mask2_encode(struct ade9113_mask2 v)
{
	return ((v.iso_test_mmr_err & 0x01) << 0) | ((v.iso_dig_mod_i_ovf & 0x01) << 1) | ((v.iso_dig_mod_v1_ovf & 0x01) << 2) | ((v.iso_dig_mod_v2_ovf & 0x01) << 3) | ((v.iso_efuse_mem_err & 0x01) << 4) | ((v.iso_phy_crc_err & 0x01) << 5) | ((v.iso_clk_stbl_err & 0x01) << 6);
}

#define ADE9113_MASK2_ENC(...) \
	ade9113_mask2_encode((struct ade9113_mask2){__VA_ARGS__})

static inline struct ade9113_mask2 ade9113_mask2_decode(uint8_t r)
{
	return (struct ade9113_mask2){
		.iso_test_mmr_err = (r >> 0) & 0x01,
		.iso_dig_mod_i_ovf = (r >> 1) & 0x01,
		.iso_dig_mod_v1_ovf = (r >> 2) & 0x01,
		.iso_dig_mod_v2_ovf = (r >> 3) & 0x01,
		.iso_efuse_mem_err = (r >> 4) & 0x01,
		.iso_phy_crc_err = (r >> 5) & 0x01,
		.iso_clk_stbl_err = (r >> 6) & 0x01,
	};
}

/* Zero-Crossing Configuration. */
struct ade9113_config_zx {
	enum ade9113_config_zx_zx_channel_config zx_channel_config;
	enum ade9113_config_zx_zx_edge_sel zx_edge_sel;
};

static inline uint8_t ade9113_config_zx_encode(struct ade9113_config_zx v)
{
	return ((v.zx_channel_config & 0x03) << 0) | ((v.zx_edge_sel & 0x03) << 2);
}

#define ADE9113_CONFIG_ZX_ENC(...) \
	ade9113_config_zx_encode((struct ade9113_config_zx){__VA_ARGS__})

static inline struct ade9113_config_zx ade9113_config_zx_decode(uint8_t r)
{
	return (struct ade9113_config_zx){
		.zx_channel_config = (r >> 0) & 0x03,
		.zx_edge_sel = (r >> 2) & 0x03,
	};
}

/* ADC Synchronization Register. */
struct ade9113_sync_snap {
	uint8_t snapshot;
	uint8_t align;
	uint8_t prep_broadcast;
};

static inline uint8_t ade9113_sync_snap_encode(struct ade9113_sync_snap v)
{
	return ((v.snapshot & 0x01) << 0) | ((v.align & 0x01) << 1) | ((v.prep_broadcast & 0x01) << 2);
}

#define ADE9113_SYNC_SNAP_ENC(...) \
	ade9113_sync_snap_encode((struct ade9113_sync_snap){__VA_ARGS__})

static inline struct ade9113_sync_snap ade9113_sync_snap_decode(uint8_t r)
{
	return (struct ade9113_sync_snap){
		.snapshot = (r >> 0) & 0x01,
		.align = (r >> 1) & 0x01,
		.prep_broadcast = (r >> 2) & 0x01,
	};
}

/* System Timing Controller Counter Most Significant Byte. */
struct ade9113_snapshot_count_hi {
	uint8_t snapshot_count;
};

static inline uint8_t ade9113_snapshot_count_hi_encode(struct ade9113_snapshot_count_hi v)
{
	return ((v.snapshot_count & 0x3F) << 0);
}

#define ADE9113_SNAPSHOT_COUNT_HI_ENC(...) \
	ade9113_snapshot_count_hi_encode((struct ade9113_snapshot_count_hi){__VA_ARGS__})

static inline struct ade9113_snapshot_count_hi ade9113_snapshot_count_hi_decode(uint8_t r)
{
	return (struct ade9113_snapshot_count_hi){
		.snapshot_count = (r >> 0) & 0x3F,
	};
}

/* Latched Status of High Priority Interrupts. */
struct ade9113_status0 {
	uint8_t comflt_err;
	uint8_t spi_crc_err;
	uint8_t efuse_mem_err;
	uint8_t crc_chg;
	uint8_t com_up;
	uint8_t reset_done;
	uint8_t status2x;
	uint8_t status1x;
};

static inline uint8_t ade9113_status0_encode(struct ade9113_status0 v)
{
	return ((v.comflt_err & 0x01) << 0) | ((v.spi_crc_err & 0x01) << 1) | ((v.efuse_mem_err & 0x01) << 2) | ((v.crc_chg & 0x01) << 3) | ((v.com_up & 0x01) << 4) | ((v.reset_done & 0x01) << 5) | ((v.status2x & 0x01) << 6) | ((v.status1x & 0x01) << 7);
}

#define ADE9113_STATUS0_ENC(...) \
	ade9113_status0_encode((struct ade9113_status0){__VA_ARGS__})

static inline struct ade9113_status0 ade9113_status0_decode(uint8_t r)
{
	return (struct ade9113_status0){
		.comflt_err = (r >> 0) & 0x01,
		.spi_crc_err = (r >> 1) & 0x01,
		.efuse_mem_err = (r >> 2) & 0x01,
		.crc_chg = (r >> 3) & 0x01,
		.com_up = (r >> 4) & 0x01,
		.reset_done = (r >> 5) & 0x01,
		.status2x = (r >> 6) & 0x01,
		.status1x = (r >> 7) & 0x01,
	};
}

/* Latched Status of Low Priority Interrupts. */
struct ade9113_status1 {
	uint8_t adc_sync_done;
	uint8_t i_wav_ovrng;
	uint8_t v1_wav_ovrng;
	uint8_t v2_wav_ovrng;
};

static inline uint8_t ade9113_status1_encode(struct ade9113_status1 v)
{
	return ((v.adc_sync_done & 0x01) << 0) | ((v.i_wav_ovrng & 0x01) << 1) | ((v.v1_wav_ovrng & 0x01) << 2) | ((v.v2_wav_ovrng & 0x01) << 3);
}

#define ADE9113_STATUS1_ENC(...) \
	ade9113_status1_encode((struct ade9113_status1){__VA_ARGS__})

static inline struct ade9113_status1 ade9113_status1_decode(uint8_t r)
{
	return (struct ade9113_status1){
		.adc_sync_done = (r >> 0) & 0x01,
		.i_wav_ovrng = (r >> 1) & 0x01,
		.v1_wav_ovrng = (r >> 2) & 0x01,
		.v2_wav_ovrng = (r >> 3) & 0x01,
	};
}

/* Latched Status of Isolated ADC Interrupts. */
struct ade9113_status2 {
	uint8_t iso_test_mmr_err;
	uint8_t iso_dig_mod_i_ovf;
	uint8_t iso_dig_mod_v1_ovf;
	uint8_t iso_dig_mod_v2_ovf;
	uint8_t iso_efuse_mem_err;
	uint8_t iso_phy_crc_err;
	uint8_t iso_clk_stbl_err;
};

static inline uint8_t ade9113_status2_encode(struct ade9113_status2 v)
{
	return ((v.iso_test_mmr_err & 0x01) << 0) | ((v.iso_dig_mod_i_ovf & 0x01) << 1) | ((v.iso_dig_mod_v1_ovf & 0x01) << 2) | ((v.iso_dig_mod_v2_ovf & 0x01) << 3) | ((v.iso_efuse_mem_err & 0x01) << 4) | ((v.iso_phy_crc_err & 0x01) << 5) | ((v.iso_clk_stbl_err & 0x01) << 6);
}

#define ADE9113_STATUS2_ENC(...) \
	ade9113_status2_encode((struct ade9113_status2){__VA_ARGS__})

static inline struct ade9113_status2 ade9113_status2_decode(uint8_t r)
{
	return (struct ade9113_status2){
		.iso_test_mmr_err = (r >> 0) & 0x01,
		.iso_dig_mod_i_ovf = (r >> 1) & 0x01,
		.iso_dig_mod_v1_ovf = (r >> 2) & 0x01,
		.iso_dig_mod_v2_ovf = (r >> 3) & 0x01,
		.iso_efuse_mem_err = (r >> 4) & 0x01,
		.iso_phy_crc_err = (r >> 5) & 0x01,
		.iso_clk_stbl_err = (r >> 6) & 0x01,
	};
}

/* ISO to NONISO Communications Fault Type. */
struct ade9113_com_flt_type {
	uint8_t iso_ecc_err;
	uint8_t iso_phy_err;
	uint8_t iso_status_rd_ecc_e_rr;
};

static inline uint8_t ade9113_com_flt_type_encode(struct ade9113_com_flt_type v)
{
	return ((v.iso_ecc_err & 0x01) << 0) | ((v.iso_phy_err & 0x01) << 1) | ((v.iso_status_rd_ecc_e_rr & 0x01) << 2);
}

#define ADE9113_COM_FLT_TYPE_ENC(...) \
	ade9113_com_flt_type_encode((struct ade9113_com_flt_type){__VA_ARGS__})

static inline struct ade9113_com_flt_type ade9113_com_flt_type_decode(uint8_t r)
{
	return (struct ade9113_com_flt_type){
		.iso_ecc_err = (r >> 0) & 0x01,
		.iso_phy_err = (r >> 1) & 0x01,
		.iso_status_rd_ecc_e_rr = (r >> 2) & 0x01,
	};
}

/* Configuration of Background Register Map CRC. */
struct ade9113_config_crc {
	uint8_t crc_force;
	uint8_t crc_done;
};

static inline uint8_t ade9113_config_crc_encode(struct ade9113_config_crc v)
{
	return ((v.crc_force & 0x01) << 0) | ((v.crc_done & 0x01) << 1);
}

#define ADE9113_CONFIG_CRC_ENC(...) \
	ade9113_config_crc_encode((struct ade9113_config_crc){__VA_ARGS__})

static inline struct ade9113_config_crc ade9113_config_crc_decode(uint8_t r)
{
	return (struct ade9113_config_crc){
		.crc_force = (r >> 0) & 0x01,
		.crc_done = (r >> 1) & 0x01,
	};
}

/* Revision Value of ISO and NONISO Silicon. */
struct ade9113_silicon_revision {
	uint8_t iso_chip_rev;
	uint8_t noniso_chip_rev;
};

static inline uint8_t ade9113_silicon_revision_encode(struct ade9113_silicon_revision v)
{
	return ((v.iso_chip_rev & 0x0F) << 0) | ((v.noniso_chip_rev & 0x0F) << 4);
}

#define ADE9113_SILICON_REVISION_ENC(...) \
	ade9113_silicon_revision_encode((struct ade9113_silicon_revision){__VA_ARGS__})

static inline struct ade9113_silicon_revision ade9113_silicon_revision_decode(uint8_t r)
{
	return (struct ade9113_silicon_revision){
		.iso_chip_rev = (r >> 0) & 0x0F,
		.noniso_chip_rev = (r >> 4) & 0x0F,
	};
}

#endif /* ADE9113_REGS_H */

/* clang-format: on */
