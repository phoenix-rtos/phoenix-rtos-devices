/*
 * Phoenix-RTOS
 *
 * i.MX RT117x ROM API driver for FlexSPI
 *
 * Copyright 2019-2021 Phoenix Systems
 * Author: Hubert Buczynski, Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include "rom_api.h"


#define FLEXSPI_DRIVER_API_ADDRESS 0x021001c


typedef struct {
	uint32_t version;
	int32_t (*init)(uint32_t instance, flexspi_norConfig_t *config);
	int32_t (*page_program)(uint32_t instance, flexspi_norConfig_t *config, uint32_t dstAddr, const uint32_t *src);
	int32_t (*erase_all)(uint32_t instance, flexspi_norConfig_t *config);
	int32_t (*erase)(uint32_t instance, flexspi_norConfig_t *config, uint32_t start, uint32_t length);
	int32_t (*read)(uint32_t instance, flexspi_norConfig_t *config, uint32_t *dst, uint32_t start, uint32_t bytes);
	void (*clear_cache)(uint32_t instance);
	int32_t (*xfer)(uint32_t instance, flexspi_xfer_t *xfer);
	int32_t (*update_lut)(uint32_t instance, uint32_t seqIndex, const uint32_t *lutBase, uint32_t numberOfSeq);
	int32_t (*get_config)(uint32_t instance, flexspi_norConfig_t *config, serial_norConfigOption_t *option);
	int32_t (*erase_sector)(uint32_t instance, flexspi_norConfig_t *config, uint32_t address);
	int32_t (*erase_block)(uint32_t instance, flexspi_norConfig_t *config, uint32_t address);
	void (*hw_reset)(uint32_t instance, uint32_t resetLogic);
	int32_t (*wait_busy)(uint32_t instance, flexspi_norConfig_t *config, char isParallelMode, uint32_t address);
	int32_t (*set_clock_source)(uint32_t instance, uint32_t clockSrc);
	void (*config_clock)(uint32_t instance, uint32_t freqOption, uint32_t sampleClkMode);
} flexspi_nor_flash_driver_t;


typedef struct BootloaderTree {
	void (*runBootloader)(void *arg);
	int version;
	const char *copyright;
	const flexspi_nor_flash_driver_t *flexspiNorDriver;
} bootloader_tree_t;


static const bootloader_tree_t **volatile const bootloader_tree = (void *)FLEXSPI_DRIVER_API_ADDRESS;


int flexspi_norFlashInit(uint32_t instance, flexspi_norConfig_t *config)
{
	return (*bootloader_tree)->flexspiNorDriver->init(instance, config);
}


int flexspi_norFlashPageProgram(uint32_t instance, flexspi_norConfig_t *config, uint32_t dstAddr, const uint32_t *src)
{
	return (*bootloader_tree)->flexspiNorDriver->page_program(instance, config, dstAddr, src);
}


int flexspi_norFlashEraseAll(uint32_t instance, flexspi_norConfig_t *config)
{
	return (*bootloader_tree)->flexspiNorDriver->erase_all(instance, config);
}


int flexspi_norGetConfig(uint32_t instance, flexspi_norConfig_t *config, serial_norConfigOption_t *option)
{
	return (*bootloader_tree)->flexspiNorDriver->get_config(instance, config, option);
}


int flexspi_norFlashErase(uint32_t instance, flexspi_norConfig_t *config, uint32_t start, uint32_t length)
{
	return (*bootloader_tree)->flexspiNorDriver->erase(instance, config, start, length);
}


int flexspi_norFlashRead(uint32_t instance, flexspi_norConfig_t *config, char *dst, uint32_t start, uint32_t bytes)
{
	return (*bootloader_tree)->flexspiNorDriver->read(instance, config, (uint32_t *)dst, start, bytes);
}


int flexspi_norFlashUpdateLUT(uint32_t instance, uint32_t seqIndex, const uint32_t *lutBase, uint32_t seqNumber)
{
	return (*bootloader_tree)->flexspiNorDriver->update_lut(instance, seqIndex, lutBase, seqNumber);
}


int flexspi_norFlashExecuteSeq(uint32_t instance, flexspi_xfer_t *xfer)
{
	return (*bootloader_tree)->flexspiNorDriver->xfer(instance, xfer);
}
