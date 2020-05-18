/*
 * Phoenix-RTOS
 *
 * i.MX RT ROM API driver for FlexSPI
 *
 * Copyright 2019, 2020 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLEXSPI_ROM_API_H_
#define _FLEXSPI_ROM_API_H_

#include <stdio.h>


typedef struct _serial_norConfigOption_t {
	uint32_t option0;
	uint32_t option1;
} serial_norConfigOption_t;


typedef enum _flexspi_operation_t {
	kFlexSpiOperation_Command,                 /* FlexSPI operation: Only command, both TX and RX buffer are ignored. */
	kFlexSpiOperation_Config,                  /* FlexSPI operation: Configure device mode, the TX FIFO size is fixed in LUT. */
	kFlexSpiOperation_Write,                   /* FlexSPI operation: Write, only TX buffer is effective */
	kFlexSpiOperation_Read,                    /* FlexSPI operation: Read, only Rx Buffer is effective. */
	kFlexSpiOperation_End = kFlexSpiOperation_Read,
} flexspi_operation_t;


typedef struct _flexspi_xfer_t {
	flexspi_operation_t operation;             /* FlexSPI operation */
	uint32_t baseAddress;                      /* FlexSPI operation base address */
	uint32_t seqId;                            /* Sequence Id */
	uint32_t seqNum;                           /* Sequence Number */
	uint8_t isParallelModeEnable;              /* Is a parallel transfer */
	uint32_t *txBuffer;                        /* Tx buffer */
	uint32_t txSize;                           /* Tx size in bytes */
	uint32_t *rxBuffer;                        /* Rx buffer */
	uint32_t rxSize;                           /* Rx size in bytes */
} flexspi_xfer_t;


typedef struct _flexspi_lutSeq_t {
	uint8_t seqNum;                             /* Sequence Number, valid number: 1-16 */
	uint8_t seqId;                              /* Sequence Index, valid number: 0-15 */
	uint16_t reserved;
} flexspi_lutSeq_t;


typedef struct _flexspi_memConfig_t {
	uint32_t tag;                               /* Tag, fixed value 0x42464346UL */
	uint32_t version;                           /* Version,[31:24] -'V', [23:16] - Major, [15:8] - Minor, [7:0] - bugfix */
	uint32_t reserved0;                         /* Reserved for future use */
	uint8_t readSampleClkSrc;                   /* Read Sample Clock Source, valid value: 0/1/3 */
	uint8_t csHoldTime;                         /* CS hold time, default value: 3 */
	uint8_t csSetupTime;                        /* CS setup time, default value: 3 */
	uint8_t columnAddressWidth;                 /* Column Address with, for HyperBus protocol, it is fixed to 3, For */
	uint8_t deviceModeCfgEnable;                /* Device Mode Configure enable flag, 1 - Enable, 0 - Disable */
	uint8_t deviceModeType;                     /* Specify the configuration command type:Quad Enable, DPI/QPI/OPI switch, */
	uint16_t waitTimeCfgCommands;               /* Wait time for all configuration commands, unit: 100us, Used for */
	flexspi_lutSeq_t deviceModeSeq;             /* Device mode sequence info, [7:0] - LUT sequence id, [15:8] - LUt */
	uint32_t deviceModeArg;                     /* Argument/Parameter for device configuration */
	uint8_t configCmdEnable;                    /* Configure command Enable Flag, 1 - Enable, 0 - Disable */
	uint8_t configModeType[3];                  /* Configure Mode Type, similar as deviceModeTpe */
	flexspi_lutSeq_t configCmdSeqs[3];          /* Sequence info for Device Configuration command, similar as deviceModeSeq */
	uint32_t reserved1;                         /* Reserved for future use */
	uint32_t configCmdArgs[3];                  /* Arguments/Parameters for device Configuration commands */
	uint32_t reserved2;                         /* Reserved for future use */
	uint32_t controllerMiscOption;              /* Controller Misc Options, see Misc feature bit definitions for more */
	uint8_t deviceType;                         /* Device Type:  See Flash Type Definition for more details */
	uint8_t sflashPadType;                      /* Serial Flash Pad Type: 1 - Single, 2 - Dual, 4 - Quad, 8 - Octal */
	uint8_t serialClkFreq;                      /* Serial Flash Frequencey, device specific definitions, See System Boot */
	uint8_t lutCustomSeqEnable;                 /* LUT customization Enable, it is required if the program/erase cannot */
	uint32_t reserved3[2];                      /* Reserved for future use */
	uint32_t sflashA1Size;                      /* Size of Flash connected to A1 */
	uint32_t sflashA2Size;                      /* Size of Flash connected to A2 */
	uint32_t sflashB1Size;                      /* Size of Flash connected to B1 */
	uint32_t sflashB2Size;                      /* Size of Flash connected to B2 */
	uint32_t csPadSettingOverride;              /* CS pad setting override value */
	uint32_t sclkPadSettingOverride;            /* SCK pad setting override value */
	uint32_t dataPadSettingOverride;            /* data pad setting override value */
	uint32_t dqsPadSettingOverride;             /* DQS pad setting override value */
	uint32_t timeoutInMs;                       /* Timeout threshold for read status command */
	uint32_t commandInterval;                   /* CS deselect interval between two commands */
	uint16_t dataValidTime[2];                  /* CLK edge to data valid time for PORT A and PORT B, in terms of 0.1ns */
	uint16_t busyOffset;                        /* Busy offset, valid value: 0-31 */
	uint16_t busyBitPolarity;                   /* Busy flag polarity, 0 - busy flag is 1 when flash device is busy, 1 - */
	uint32_t lookupTable[64];                   /* Lookup table holds Flash command sequences */
	flexspi_lutSeq_t lutCustomSeq[12];          /* Customizable LUT Sequences */
	uint32_t reserved4[4];                      /* Reserved for future use */
} flexspi_memConfig_t;


typedef struct _flexspi_norConfig_t {
	flexspi_memConfig_t memConfig;              /* Common memory configuration info via FlexSPI */
	uint32_t pageSize;                          /* Page size of Serial NOR */
	uint32_t sectorSize;                        /* Sector size of Serial NOR */
	uint8_t ipcmdSerialClkFreq;                 /* Clock frequency for IP command */
	uint8_t isUniformBlockSize;                 /* Sector/Block size is the same */
	uint8_t reserved0[2];                       /* Reserved for future use */
	uint8_t serialNorType;                      /* Serial NOR Flash type: 0/1/2/3 */
	uint8_t needExitNoCmdMode;                  /* Need to exit NoCmd mode before other IP command */
	uint8_t halfClkForNonReadCmd;               /* Half the Serial Clock for non-read command: true/false */
	uint8_t needRestoreNoCmdMode;               /* Need to Restore NoCmd mode after IP commmand execution */
	uint32_t blockSize;                         /* Block size */
	uint32_t reserved2[11];                     /* Reserved for future use */
} flexspi_norConfig_t;


int flexspi_norFlashInit(uint32_t instance, flexspi_norConfig_t *config);


int flexspi_norFlashPageProgram(uint32_t instance, flexspi_norConfig_t *config, uint32_t dstAddr, const uint32_t *src);


int flexspi_norFlashEraseAll(uint32_t instance, flexspi_norConfig_t *config);


int flexspi_norGetConfig(uint32_t instance, flexspi_norConfig_t *config, serial_norConfigOption_t *option);


int flexspi_norFlashErase(uint32_t instance, flexspi_norConfig_t *config, uint32_t start, uint32_t length);


int flexspi_norFlashRead(uint32_t instance, flexspi_norConfig_t *config, uint32_t *dst, uint32_t start, uint32_t bytes);


int flexspi_norFlashExecuteSeq(uint32_t instance, flexspi_xfer_t *xfer);


int flexspi_norFlashUpdateLUT(uint32_t instance, uint32_t seqIndex, const uint32_t *lutBase, uint32_t seqNumber);


#endif
