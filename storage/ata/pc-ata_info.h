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


typedef union _atainfo_t {

	/* Detailed information about these fields can be found in t13 d1699 ATA8 ACS document */
	struct {
		u16 general_config;
		u16 obsolete_1;
		u16 specific_config;
		u16 obsolete_2;
		u32 retired_1;
		u16 obsolete_3;
		u32 compactflash_1;
		u16 retired_2;
		u8 serial_number[20];
		u32 retired_3;
		u16 obsolete_4;
		u8 firmware_rev[8];
		u8 model[40];
		u8 control80h;
		u8 max_drq_multiple;
		u16 trusted_computing;
		u16 capabilities_1;
		u16 capabilities_2;
		u32 obsolete_5;
		u16 freefall_control;
		u8 obsolete_6[10];
		u8 reserved_1;
		u8 logicalsect_drq;
		u32 lba28_totalsectors;
		u16 obsolete_7;
		u16 mdma_support;
		u8 reserved_2;
		u8 pio_support;
		u16 min_mdma_cycle;
		u16 rec_mdmda_cycle;
		u16 min_pio_cycle;
		u16 min_pioirq_cycle;
		u32 reserved_3;
		u8 idf_packet_dev[8];
		u16 queue_depth;
		u16 sata_capabilities;
		u16 sata_reserved;
		u16 sata_features_sup;
		u16 sata_features_on;
		u16 ver_major;
		u16 ver_minor;
		u16 commands1_sup;
		u16 commands2_sup;
		u16 commands3_sup;
		u16 commands4_sup;
		u16 commands5_sup;
		u16 commands6_sup;
		u16 udma_modes;
		u8 reserved_4;
		u8 norm_secure_erase_time;
		u8 reserved_5;
		u8 ext_secure_erase_time;
		u16 current_apm;
		u16 master_password;
		u16 hwreset_result;
		u8 recommended_aam;
		u8 current_aam;
		u16 stream_minimum_req;
		u16 dma_stream_trans_time;
		u16 dmapio_stream_acces_lat;
		u32 perf_grain;
		u64 lba48_totalsectors;
		u16 pio_streaming_time;
		u16 reserved_6;
		u16 physlog_sector_size;
		u16 interseek_delay;
		u8 world_wide_name[8];
		u8 reserved_7[8];
		u16 reserved_8;
		u32 log_sector_size;
		u16 commands7_sup;
		u16 commands8_sup;
		u8 reserved_9[12];
		u16 obsolete_8;
		u16 security_status;
		u8 vendor_specific[62];
		u16 cfa_power_mode;
		u8 compactflash_2[14];
		u8 reserved_10;
		u8 nominal_formfactor;
		u8 reserved_11[14];
		u8 curr_media_serial[60];
		u16 sct_command_trans;
		u32 ce_ata_reserved_1;
		u16 loginphy_alignment;
		u32 wrv_sectorcountmode3;
		u32 wrv_sectorcountmode2;
		u16 nv_cache_cap;
		u32 nv_cache_size;
		u16 rotation_rate;
		u16 reserved_12;
		u16 nv_cache_opts;
		u8 reserved_13;
		u8 wrv_feature_set;
		u16 reserved_14;
		u16 transport_major_ver;
		u16 transport_minor_ver;
		u8 ce_ata_reserved_2[20];
		u16 min_db_microcode_dl;
		u16 max_db_microcode_dl;
		u8 reserved_15[38];
		u8 checksum;
		u8 checksum_valid;
	} __attribute__((packed));


	struct {
		u8 buff[512];
	} __attribute__((packed));

} atainfo_t;

#endif
