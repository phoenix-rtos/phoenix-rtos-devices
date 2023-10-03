/*
 * Phoenix-RTOS
 *
 * Definitions of registers, data structures and commands for SD Host Controllers.
 * Compatible with SD Specifications Part A2: SD Host Controller Simplified Specification Version 2.00
 *
 * Copyright 2023 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _SDHOST_DEFS_H_
#define _SDHOST_DEFS_H_

#include <stdint.h>

/* SD memory card / SDIO standard commands */
enum SDIO_CMD {
	SDIO_CMD0_GO_IDLE_STATE = 0,          /* set the card into idle state */
	SDIO_CMD2_ALL_SEND_CID = 2,           /* all cards send their CID */
	SDIO_CMD3_RELATIVE_ADDR = 3,          /* set card's relative address */
	SDIO_CMD6_SWITCH_FUNC = 6,            /* check card's functions or switch function */
	SDIO_CMD7_SELECT_CARD = 7,            /* select a card by address */
	SDIO_CMD8_SEND_IF_COND = 8,           /* inquire about operating conditions */
	SDIO_CMD9_SEND_CSD = 9,               /* get CSD of addressed card */
	SDIO_CMD10_SEND_CID = 10,             /* get CID of addressed card */
	SDIO_CMD13_SEND_STATUS = 13,          /* get status of addressed card */
	SDIO_CMD15_STATE_INACTIVE = 15,       /* set the card into inactive state */
	SDIO_CMD16_SET_BLOCKLEN = 16,         /* set block length for data transfers (non-SDHC cards only) */
	SDIO_CMD17_READ_SINGLE_BLOCK = 17,    /* read single block */
	SDIO_CMD18_READ_MULTIPLE_BLOCK = 18,  /* read multiple blocks */
	SDIO_CMD24_WRITE_SINGLE_BLOCK = 24,   /* write single block */
	SDIO_CMD25_WRITE_MULTIPLE_BLOCK = 25, /* write multiple blocks */
	SDIO_CMD32_ERASE_WR_BLK_START = 32,   /* set start address for erase operation */
	SDIO_CMD33_ERASE_WR_BLK_END = 33,     /* set end address for erase operation */
	SDIO_CMD38_ERASE = 38,                /* start erase */
	SDIO_CMD52_RW_DIRECT = 52,            /* direct register I/O operation */
	SDIO_CMD53_RW_EXTENDED = 53,          /* bulk register I/O operation */
	SDIO_CMD55_APP_CMD = 55,              /* "prefix" for application-specific commands */
};

#define SDIO_ACMD(x)  ((1 << 6) | (x))
#define SDIO_ACMD_BIT SDIO_ACMD(0)
/* SD card application specific commands */
#define SDIO_ACMD6_SET_BUS_WIDTH    SDIO_ACMD(6)
#define SDIO_ACMD13_SD_STATUS       SDIO_ACMD(13) /* send SD Status register */
#define SDIO_ACMD41_SD_SEND_OP_COND SDIO_ACMD(41) /* inquire about operating conditions, negotiate voltage, initialize card */
#define SDIO_ACMD51_SEND_SCR        SDIO_ACMD(51) /* send SD Card Configuration Register */

#define RESPONSE_TYPE_BITS(x)     ((x) << 0)
#define CRC_CHECK_ENABLE_BIT(x)   ((x) << 3)
#define INDEX_CHECK_ENABLE_BIT(x) ((x) << 4)
#define DATA_PRESENT_BIT(x)       ((x) << 5)
#define COMMAND_TYPE_BIT(x)       ((x) << 6)

typedef union {
	struct {
		uint32_t dmaEnable : 1;
		uint32_t blockCountEnable : 1;
		uint32_t autoCmd12Enable : 1;
		uint32_t reserved1 : 1;
		uint32_t directionRead : 1;
		uint32_t multiBlock : 1;
		uint32_t reserved2 : 10;
		uint32_t responseType : 2;
		uint32_t reserved3 : 1;
		uint32_t crcCheckEnable : 1;
		uint32_t indexCheckEnable : 1;
		uint32_t dataPresent : 1;
		uint32_t commandType : 2;
		uint32_t commandIdx : 6;
		uint32_t reserved4 : 2;
	};
	struct {
		uint32_t transferModeRegister : 16;
		uint32_t commandMeta : 8;
	};
	uint32_t raw;
} sdhost_command_reg_t;


/* Enum of all registers defined by the standard. In comments are the byte offsets
 * that are used in documentation.
 */
enum SDHOST_REG {
	SDHOST_REG_SDMA_ADDRESS = 0x00 / 4,           /* Offset 000h */
	SDHOST_REG_TRANSFER_BLOCK = 0x04 / 4,         /* Offset 004h, 006h */
	SDHOST_REG_CMD_ARGUMENT = 0x08 / 4,           /* Offset 008h */
	SDHOST_REG_CMD = 0x0c / 4,                    /* Offset 00Ch, 00Eh */
	SDHOST_REG_RESPONSE_0 = 0x10 / 4,             /* Offset 010h */
	SDHOST_REG_RESPONSE_1 = 0x14 / 4,             /* Offset 014h */
	SDHOST_REG_RESPONSE_2 = 0x18 / 4,             /* Offset 018h */
	SDHOST_REG_RESPONSE_3 = 0x1c / 4,             /* Offset 01Ch */
	SDHOST_REG_BUFFER_DATA = 0x20 / 4,            /* Offset 020h */
	SDHOST_REG_PRES_STATE = 0x24 / 4,             /* Offset 024h */
	SDHOST_REG_HOST_CONTROL = 0x28 / 4,           /* Offset 028h, 029h, 02Ah, 02Bh */
	SDHOST_REG_CLOCK_CONTROL = 0x2c / 4,          /* Offset 02Ch, 02Eh, 02Fh */
	SDHOST_REG_INTR_STATUS = 0x30 / 4,            /* Offset 030h, 032h */
	SDHOST_REG_INTR_STATUS_ENABLE = 0x34 / 4,     /* Offset 034h, 036h */
	SDHOST_REG_INTR_SIGNAL_ENABLE = 0x38 / 4,     /* Offset 038h, 03Ah */
	SDHOST_REG_AUTOCMD12_ERROR_STATUS = 0x3c / 4, /* Offset 03Ch */
	SDHOST_REG_CAPABILITIES = 0x40 / 4,           /* Offset 040h */
	SDHOST_REG_CAPABILITIES_RESERVED = 0x44 / 4,  /* Offset 044h */
	SDHOST_REG_CURRENT_CAP = 0x48 / 4,            /* Offset 048h */
	SDHOST_REG_CURRENT_CAP_RESERVED = 0x4c / 4,   /* Offset 04Ch */
	SDHOST_REG_FORCE_EVENT = 0x50 / 4,            /* Offset 050h, 052h */
	SDHOST_REG_ADMA_ERROR_STATUS = 0x54 / 4,      /* Offset 054h */
	SDHOST_REG_ADMA_ADDR_1 = 0x58 / 4,            /* Offset 058h */
	SDHOST_REG_ADMA_ADDR_2 = 0x5c / 4,            /* Offset 05Ch */
	SDHOST_REG_VERSION = 0xfc / 4,                /* Offset 0FCh, 0FEh */
};

enum REG_STATE {
	PRES_STATE_CMD_BUSY = (1UL << 0),
	PRES_STATE_DAT_BUSY = (1UL << 1),
	PRES_STATE_CARD_INSERTED = (1UL << 16),
	PRES_STATE_CARD_DET_STABLE = (1UL << 17),
	PRES_STATE_CARD_DET_PIN = (1UL << 18),
	PRES_STATE_WRITE_PROT_PIN = (1UL << 19),
};

#define PRES_STATE_BUSY_FLAGS (PRES_STATE_DAT_BUSY | PRES_STATE_CMD_BUSY)

enum TRANSFER_BLOCK {
	TRANSFER_BLOCK_SDMA_BOUNDARY_4K = 0b000UL << 12,   /* 4K bytes (Detects A11 carry out)*/
	TRANSFER_BLOCK_SDMA_BOUNDARY_8K = 0b001UL << 12,   /* 8K bytes (Detects A12 carry out)*/
	TRANSFER_BLOCK_SDMA_BOUNDARY_16K = 0b010UL << 12,  /* 16K bytes (Detects A13 carry out)*/
	TRANSFER_BLOCK_SDMA_BOUNDARY_32K = 0b011UL << 12,  /* 32K bytes (Detects A14 carry out)*/
	TRANSFER_BLOCK_SDMA_BOUNDARY_64K = 0b100UL << 12,  /* 64K bytes (Detects A15 carry out)*/
	TRANSFER_BLOCK_SDMA_BOUNDARY_128K = 0b101UL << 12, /* 128K bytes (Detects A16 carry out)*/
	TRANSFER_BLOCK_SDMA_BOUNDARY_256K = 0b110UL << 12, /* 256K bytes (Detects A17 carry out)*/
	TRANSFER_BLOCK_SDMA_BOUNDARY_512K = 0b111UL << 12, /* 512K bytes (Detects A18 carry out)*/
};

enum HOST_CONTROL {
	HOST_CONTROL_LED_ON = 1UL << 0,
	HOST_CONTROL_4_BIT_MODE = 1UL << 1,
	HOST_CONTROL_HIGH_SPEED = 1UL << 2,

	HOST_CONTROL_DMA_SELECT_SDMA = 0b00UL << 3,
	HOST_CONTROL_DMA_SELECT_ADMA32 = 0b10UL << 3,
	HOST_CONTROL_DMA_SELECT_ADMA64 = 0b11UL << 3,

	HOST_CONTROL_CARD_DET_TEST = 1UL << 6,
	HOST_CONTROL_CARD_DET_TEST_ENABLE = 1UL << 7,
	HOST_CONTROL_BUS_POWER = 1UL << 8,

	HOST_CONTROL_BUS_VOLTAGE_1V8 = 0b101UL << 9,
	HOST_CONTROL_BUS_VOLTAGE_3V0 = 0b110UL << 9,
	HOST_CONTROL_BUS_VOLTAGE_3V3 = 0b111UL << 9,

};

enum CLOCK_CONTROL {
	CLOCK_CONTROL_START_INTERNAL_CLOCK = 1UL << 0,  /* Flag to start internal clock for SD host */
	CLOCK_CONTROL_INTERNAL_CLOCK_STABLE = 1UL << 1, /* Flag to check if the internal clock is stable and can be used */
	CLOCK_CONTROL_START_SD_CLOCK = 1UL << 2,        /* Flag to start output of clock on CLK pin */

	/* NOTE: I know it looks like (clockDivider / 2) << 8, but according to docs only these particular settings are supported. */
	CLOCK_CONTROL_DIV_256 = 0x80 << 8,  /* Divide refclk by 256 to get SD clock */
	CLOCK_CONTROL_DIV_128 = 0x40 << 8,  /* Divide refclk by 128 to get SD clock */
	CLOCK_CONTROL_DIV_64 = 0x20 << 8,   /* Divide refclk by 64 to get SD clock */
	CLOCK_CONTROL_DIV_32 = 0x10 << 8,   /* Divide refclk by 32 to get SD clock */
	CLOCK_CONTROL_DIV_16 = 0x08 << 8,   /* Divide refclk by 16 to get SD clock */
	CLOCK_CONTROL_DIV_8 = 0x04 << 8,    /* Divide refclk by 8 to get SD clock */
	CLOCK_CONTROL_DIV_4 = 0x02 << 8,    /* Divide refclk by 4 to get SD clock */
	CLOCK_CONTROL_DIV_2 = 0x01 << 8,    /* Divide refclk by 2 to get SD clock */
	CLOCK_CONTROL_DIV_1 = 0x00 << 8,    /* Divide refclk by 1 to get SD clock (use it directly) */
	CLOCK_CONTROL_DIV_MASK = 0xff << 8, /* Mask of bits for clock divider */

	CLOCK_CONTROL_RESET_ALL = 1UL << 24, /* Reset everything in the host (including clocks) */
	CLOCK_CONTROL_RESET_CMD = 1UL << 25, /* Reset parts of host related to command issuing */
	CLOCK_CONTROL_RESET_DAT = 1UL << 26, /* Reset parts of host related to data transfers */
};

#define CLOCK_CONTROL_DATA_TIMEOUT_VALUE(x) (((x)&0xf) << 16)

/* SDIO event status bits */
enum SDHOST_INTR {
	SDHOST_INTR_CMD_DONE = 1UL << 0,           /* command complete         */
	SDHOST_INTR_TRANSFER_DONE = 1UL << 1,      /* transfer complete        */
	SDHOST_INTR_BLOCK_GAP = 1UL << 2,          /* transfer was stopped     */
	SDHOST_INTR_SDMA_BOUNDARY = 1UL << 3,      /* DMA boundary reached     */
	SDHOST_INTR_RW_WRITE_READY = 1UL << 4,     /* write buffer ready       */
	SDHOST_INTR_RW_READ_READY = 1UL << 5,      /* read buffer ready        */
	SDHOST_INTR_CARD_IN = 1UL << 6,            /* card inserted            */
	SDHOST_INTR_CARD_OUT = 1UL << 7,           /* card removed             */
	SDHOST_INTR_CARD_IRQ = 1UL << 8,           /* card requested interrupt */
	SDHOST_INTR_TUNING_RETUNE = 1UL << 12,     /* retuning required soon   */
	SDHOST_INTR_CMD_ERROR = 1UL << 15,         /* command error            */
	SDHOST_INTR_CMD_TIMEOUT = 1UL << 16,       /* command timed out        */
	SDHOST_INTR_CMD_CRC = 1UL << 17,           /* command CRC value error  */
	SDHOST_INTR_CMD_ENDBIT = 1UL << 18,        /* command end bit error    */
	SDHOST_INTR_CMD_INDEX = 1UL << 19,         /* command index error      */
	SDHOST_INTR_DATA_TIMEOUT = 1UL << 20,      /* data timed out           */
	SDHOST_INTR_DATA_CRC = 1UL << 21,          /* data CRC value error     */
	SDHOST_INTR_DATA_ENDBIT = 1UL << 22,       /* data end bit error       */
	SDHOST_INTR_OVERCURRENT_ERROR = 1UL << 23, /* data end bit error       */
	SDHOST_INTR_AUTO_CMD12_ERROR = 1UL << 24,  /* data end bit error       */
	SDHOST_INTR_ADMA_ERROR = 1UL << 25,        /* tuning error occurred    */
	SDHOST_INTR_DMA_ERROR = 1UL << 28,         /* DMA transfer failed      */
};

#define SDHOST_INTR_CMD_STATUS ( \
	SDHOST_INTR_CMD_DONE | \
	SDHOST_INTR_TRANSFER_DONE | \
	SDHOST_INTR_BLOCK_GAP | \
	SDHOST_INTR_SDMA_BOUNDARY)

#define SDHOST_INTR_CMD_ERRORS ( \
	SDHOST_INTR_CMD_TIMEOUT | \
	SDHOST_INTR_CMD_CRC | \
	SDHOST_INTR_CMD_ENDBIT | \
	SDHOST_INTR_CMD_INDEX)

#define SDHOST_INTR_DAT_ERRORS ( \
	SDHOST_INTR_DATA_TIMEOUT | \
	SDHOST_INTR_DATA_CRC | \
	SDHOST_INTR_DATA_ENDBIT)

typedef enum {
	CMD_INVALID = 0,
	CMD_NO_DATA,      /* Command does not have data transfer */
	CMD_NO_DATA_WAIT, /* Command does not have data transfer, but needs us to wait for transferComplete bit */
	CMD_WRITE,        /* Command has data transfer, it's a 1 block write to card */
	CMD_WRITE_MULTI,  /* Command has data transfer, it's a multi-block write to card */
	CMD_READ,         /* Command has data transfer, it's a 1 block read from card */
	CMD_READ_MULTI,   /* Command has data transfer, it's a multi-block read from card */
	CMD_READ8,        /* Command has data transfer, it's an 8 byte read */
	CMD_READ64,       /* Command has data transfer, it's a 64 byte read */
} sdhost_command_data_t;

typedef struct {
	uint8_t bitsWhenSending;
	uint8_t dataType;
} cmd_metadata_t;

#define RESPONSE_METADATA_NONE \
	{ \
		.bitsWhenSending = 0, \
		.dataType = CMD_NO_DATA, \
	}

#define RESPONSE_METADATA_R1(data_type_arg) \
	{ \
		.bitsWhenSending = (RESPONSE_TYPE_BITS(2UL) | INDEX_CHECK_ENABLE_BIT(1UL) | CRC_CHECK_ENABLE_BIT(1UL)), \
		.dataType = data_type_arg, \
	}

#define RESPONSE_METADATA_R1b \
	{ \
		.bitsWhenSending = (RESPONSE_TYPE_BITS(3UL) | INDEX_CHECK_ENABLE_BIT(1UL) | CRC_CHECK_ENABLE_BIT(1UL)), \
		.dataType = CMD_NO_DATA_WAIT, \
	}

#define RESPONSE_METADATA_R2 \
	{ \
		.bitsWhenSending = (RESPONSE_TYPE_BITS(1UL) | CRC_CHECK_ENABLE_BIT(1UL)), \
		.dataType = CMD_NO_DATA, \
	}

#define RESPONSE_METADATA_R3 \
	{ \
		.bitsWhenSending = (RESPONSE_TYPE_BITS(2UL)), \
		.dataType = CMD_NO_DATA, \
	}

#define RESPONSE_METADATA_R4 \
	{ \
		.bitsWhenSending = (RESPONSE_TYPE_BITS(2UL)), \
		.dataType = CMD_NO_DATA, \
	}

#define RESPONSE_METADATA_R5 \
	{ \
		.bitsWhenSending = (RESPONSE_TYPE_BITS(2UL) | INDEX_CHECK_ENABLE_BIT(1UL) | CRC_CHECK_ENABLE_BIT(1UL)), \
		.dataType = CMD_NO_DATA, \
	}

#define RESPONSE_METADATA_R5b \
	{ \
		.bitsWhenSending = (RESPONSE_TYPE_BITS(3UL) | INDEX_CHECK_ENABLE_BIT(1UL) | CRC_CHECK_ENABLE_BIT(1UL)), \
		.dataType = CMD_NO_DATA_WAIT, \
	}

#define RESPONSE_METADATA_R6 \
	{ \
		.bitsWhenSending = (RESPONSE_TYPE_BITS(2UL) | INDEX_CHECK_ENABLE_BIT(1UL) | CRC_CHECK_ENABLE_BIT(1UL)), \
		.dataType = CMD_NO_DATA, \
	}

#define RESPONSE_METADATA_R7 \
	{ \
		.bitsWhenSending = (RESPONSE_TYPE_BITS(2UL) | INDEX_CHECK_ENABLE_BIT(1UL) | CRC_CHECK_ENABLE_BIT(1UL)), \
		.dataType = CMD_NO_DATA, \
	}

#define MAX_SD_COMMANDS 128

static const cmd_metadata_t sdCmdMetadata[MAX_SD_COMMANDS] = {
	[SDIO_CMD0_GO_IDLE_STATE] = RESPONSE_METADATA_NONE,
	[SDIO_CMD2_ALL_SEND_CID] = RESPONSE_METADATA_R2,
	[SDIO_CMD3_RELATIVE_ADDR] = RESPONSE_METADATA_R6,
	[SDIO_CMD6_SWITCH_FUNC] = RESPONSE_METADATA_R1(CMD_READ64),
	[SDIO_CMD7_SELECT_CARD] = RESPONSE_METADATA_R1b,
	[SDIO_CMD8_SEND_IF_COND] = RESPONSE_METADATA_R7,
	[SDIO_CMD9_SEND_CSD] = RESPONSE_METADATA_R2,
	[SDIO_CMD10_SEND_CID] = RESPONSE_METADATA_R2,
	[SDIO_CMD13_SEND_STATUS] = RESPONSE_METADATA_R1(CMD_NO_DATA),
	[SDIO_CMD16_SET_BLOCKLEN] = RESPONSE_METADATA_R1(CMD_NO_DATA),
	[SDIO_CMD17_READ_SINGLE_BLOCK] = RESPONSE_METADATA_R1(CMD_READ),
	[SDIO_CMD18_READ_MULTIPLE_BLOCK] = RESPONSE_METADATA_R1(CMD_READ_MULTI),
	[SDIO_CMD24_WRITE_SINGLE_BLOCK] = RESPONSE_METADATA_R1(CMD_WRITE),
	[SDIO_CMD25_WRITE_MULTIPLE_BLOCK] = RESPONSE_METADATA_R1(CMD_WRITE_MULTI),
	[SDIO_CMD32_ERASE_WR_BLK_START] = RESPONSE_METADATA_R1(CMD_NO_DATA),
	[SDIO_CMD33_ERASE_WR_BLK_END] = RESPONSE_METADATA_R1(CMD_NO_DATA),
	[SDIO_CMD38_ERASE] = RESPONSE_METADATA_R1b,
	[SDIO_CMD55_APP_CMD] = RESPONSE_METADATA_R1(CMD_NO_DATA),
	[SDIO_ACMD6_SET_BUS_WIDTH] = RESPONSE_METADATA_R1(CMD_NO_DATA),
	[SDIO_ACMD13_SD_STATUS] = RESPONSE_METADATA_R1(CMD_READ64),
	[SDIO_ACMD41_SD_SEND_OP_COND] = RESPONSE_METADATA_R3,
	[SDIO_ACMD51_SEND_SCR] = RESPONSE_METADATA_R1(CMD_READ8),
	{ 0 },
};


#define IF_COND_ECHO_PATTERN  (0xaa)    /* Can be anything, but 0xaa is recommended */
#define IF_COND_3V3_SUPPORTED (1 << 8)  /* If 1, 3.3V voltage is supported by card */
#define IF_COND_READY         (1 << 31) /* If 1, card has finished initializing */

#define CSD_VERSION(r32)             ((r32[3] >> 22) & 0x3)
#define CSDV1_READ_BL_LEN(r32)       ((r32[2] >> 8) & 0xf)
#define CSDV1_C_SIZE(r32)            (((r32[2] & 0x3) << 10) | (r32[1] >> 22))
#define CSDV1_C_SIZE_MULT(r32)       ((r32[1] >> 7) & 0x7)
#define CSDV1_WRITE_BL_LEN(r32)      ((r32[0] >> 14) & 0xf)
#define CSDV1_ERASE_BLK_EN(r32)      ((r32[1] >> 6) & 1)
#define CSDV1_ERASE_SECTOR_SIZE(r32) (((r32[1] & 0x3f) << 1) | (r32[0] >> 31))

#define CSDV2_C_SIZE(r32) ((r32[1] >> 8) & 0x3fffff)

#define SCR_DATA_STAT_AFTER_ERASE(x) (x[1] >> 7)

#define SCR_BUS_WIDTHS(x)    (x[1] & 0xf)
#define SCR_BUS_WIDTHS_1_BIT (1 << 0)
#define SCR_BUS_WIDTHS_4_BIT (1 << 2)

#define SCR_SD_SPEC(x)    (x[0] & 0xf)
#define SCR_SD_SPEC_V1_01 (0)
#define SCR_SD_SPEC_V1_10 (1)
#define SCR_SD_SPEC_V2_00 (2)

#define SD_STATUS_DAT_BUS_WIDTH(x)    (((x[0]) >> 6) & 0x3)
#define SD_STATUS_DAT_BUS_WIDTH_1_BIT (0)
#define SD_STATUS_DAT_BUS_WIDTH_4_BIT (2)

#define SDIO_SWITCH_FUNC_GET        (0 << 31)
#define SDIO_SWITCH_FUNC_SET        (1 << 31)
#define SDIO_SWITCH_FUNC_HIGH_SPEED (1 << 0)

enum CARD_STATUS {
	CARD_STATUS_APP_CMD = (1 << 5),
	CARD_STATUS_READY_FOR_DATA = (1 << 8),
	CARD_STATUS_ERASE_RESET = (1 << 13),
	CARD_STATUS_ECC_DISABLED = (1 << 14),
	CARD_STATUS_WP_ERASE_SKIP = (1 << 15),
	CARD_STATUS_CSD_OVERWRITE = (1 << 16),
	CARD_STATUS_ERROR = (1 << 19),
	CARD_ERROR_CC_ERROR = (1 << 20),
	CARD_ERROR_CARD_ECC_FAILED = (1 << 21),
	CARD_ERROR_ILLEGAL_COMMAND = (1 << 22),
	CARD_ERROR_COM_CRC_ERROR = (1 << 23),
	CARD_ERROR_LOCK_UNLOCK_FAILED = (1 << 24),
	CARD_ERROR_WP_VIOLATION = (1 << 26),
	CARD_ERROR_ERASE_PARAM = (1 << 27),
	CARD_ERROR_ERASE_SEQ_ERROR = (1 << 28),
	CARD_ERROR_BLOCK_LEN_ERROR = (1 << 29),
	CARD_ERROR_ADDRESS_ERROR = (1 << 30),
	CARD_ERROR_OUT_OF_RANGE = (1 << 31),
};

enum CARD_STATUS_CURRENT_STATES {
	CARD_STATUS_CURRENT_STATE_IDLE = 0,
	CARD_STATUS_CURRENT_STATE_READY = 1,
	CARD_STATUS_CURRENT_STATE_IDENT = 2,
	CARD_STATUS_CURRENT_STATE_STBY = 3,
	CARD_STATUS_CURRENT_STATE_TRAN = 4,
	CARD_STATUS_CURRENT_STATE_DATA = 5,
	CARD_STATUS_CURRENT_STATE_RCV = 6,
	CARD_STATUS_CURRENT_STATE_PRG = 7,
	CARD_STATUS_CURRENT_STATE_DIS = 8,
};

#define CARD_STATUS_CURRENT_STATE(x) ((x >> 9) & 0xf)
#define CARD_STATUS_ERRORS           ( \
	CARD_STATUS_ERROR | \
	CARD_ERROR_CC_ERROR | \
	CARD_ERROR_CARD_ECC_FAILED | \
	CARD_ERROR_ILLEGAL_COMMAND | \
	CARD_ERROR_COM_CRC_ERROR | \
	CARD_ERROR_LOCK_UNLOCK_FAILED | \
	CARD_ERROR_WP_VIOLATION | \
	CARD_ERROR_ERASE_PARAM | \
	CARD_ERROR_ERASE_SEQ_ERROR | \
	CARD_ERROR_BLOCK_LEN_ERROR | \
	CARD_ERROR_ADDRESS_ERROR | \
	CARD_ERROR_OUT_OF_RANGE)

#define SWITCH_FUNC_REG_MAX_CURRENT(x)               (((uint16_t)x[0] << 8) | x[1])
#define SDCARD_FUNCTION_GROUP_ACCESS_MODE            1
#define SDCARD_FUNCTION_GROUP_ACCESS_MODE_HIGH_SPEED (1 << 1)
#define SDCARD_FUNCTION_GROUP_COMMAND_SYSTEM         2

#endif /* _SDHOST_DEFS_H_ */
