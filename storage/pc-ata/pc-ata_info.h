/* 
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * Generic ata devices controller
 *
 * Copyright 2012-2015 Phoenix Systems
 * Author: Marcin Stragowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _DEV_ATAINFO_H_
#define _DEV_ATAINFO_H_

#include <stdint.h>

typedef union _atainfo_t {

	/* Detailed information about these fields can be found in t13 d1699 ATA8 ACS document */
	struct {
		uint16_t general_config;
		uint16_t obsolete_1;
		uint16_t specific_config;
		uint16_t obsolete_2;
		uint32_t retired_1;
		uint16_t obsolete_3;
		uint32_t compactflash_1;
		uint16_t retired_2;
		uint8_t serial_number[20];
		uint32_t retired_3;
		uint16_t obsolete_4;
		uint8_t firmware_rev[8];
		uint8_t model[40];
		uint8_t control80h;
		uint8_t max_drq_multiple;
		uint16_t trusted_computing;
		uint16_t capabilities_1;
		uint16_t capabilities_2;
		uint32_t obsolete_5;
		uint16_t freefall_control;
		uint8_t obsolete_6[10];
		uint8_t reserved_1;
		uint8_t logicalsect_drq;
		uint32_t lba28_totalsectors;
		uint16_t obsolete_7;
		uint16_t mdma_support;
		uint8_t reserved_2;
		uint8_t pio_support;
		uint16_t min_mdma_cycle;
		uint16_t rec_mdmda_cycle;
		uint16_t min_pio_cycle;
		uint16_t min_pioirq_cycle;
		uint32_t reserved_3;
		uint8_t idf_packet_dev[8];
		uint16_t queue_depth;
		uint16_t sata_capabilities;
		uint16_t sata_reserved;
		uint16_t sata_features_sup;
		uint16_t sata_features_on;
		uint16_t ver_major;
		uint16_t ver_minor;
		uint16_t commands1_sup;
		uint16_t commands2_sup;
		uint16_t commands3_sup;
		uint16_t commands4_sup;
		uint16_t commands5_sup;
		uint16_t commands6_sup;
		uint16_t udma_modes;
		uint8_t reserved_4;
		uint8_t norm_secure_erase_time;
		uint8_t reserved_5;
		uint8_t ext_secure_erase_time;
		uint16_t current_apm;
		uint16_t master_password;
		uint16_t hwreset_result;
		uint8_t recommended_aam;
		uint8_t current_aam;
		uint16_t stream_minimum_req;
		uint16_t dma_stream_trans_time;
		uint16_t dmapio_stream_acces_lat;
		uint32_t perf_grain;
		uint64_t lba48_totalsectors;
		uint16_t pio_streaming_time;
		uint16_t reserved_6;
		uint16_t physlog_sector_size;
		uint16_t interseek_delay;
		uint8_t world_wide_name[8];
		uint8_t reserved_7[8];
		uint16_t reserved_8;
		uint32_t log_sector_size;
		uint16_t commands7_sup;
		uint16_t commands8_sup;
		uint8_t reserved_9[12];
		uint16_t obsolete_8;
		uint16_t security_status;
		uint8_t vendor_specific[62];
		uint16_t cfa_power_mode;
		uint8_t compactflash_2[14];
		uint8_t reserved_10;
		uint8_t nominal_formfactor;
		uint8_t reserved_11[14];
		uint8_t curr_media_serial[60];
		uint16_t sct_command_trans;
		uint32_t ce_ata_reserved_1;
		uint16_t loginphy_alignment;
		uint32_t wrv_sectorcountmode3;
		uint32_t wrv_sectorcountmode2;
		uint16_t nv_cache_cap;
		uint32_t nv_cache_size;
		uint16_t rotation_rate;
		uint16_t reserved_12;
		uint16_t nv_cache_opts;
		uint8_t reserved_13;
		uint8_t wrv_feature_set;
		uint16_t reserved_14;
		uint16_t transport_major_ver;
		uint16_t transport_minor_ver;
		uint8_t ce_ata_reserved_2[20];
		uint16_t min_db_microcode_dl;
		uint16_t max_db_microcode_dl;
		uint8_t reserved_15[38];
		uint8_t checksum;
		uint8_t checksum_valid;
	} __attribute__((packed));


	struct {
		uint8_t buff[512];
	} __attribute__((packed));

} atainfo_t;

#endif
