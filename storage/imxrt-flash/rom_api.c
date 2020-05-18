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


#include <errno.h>
#include "rom_api.h"


#define FLEXSPI_DRIVER_API_ADDRESS 0x00201a60

#define FLEX_SPI_LUT_INSTR(opcode1, pads1, operand1, opcode0, pads0, operand0) \
	(((opcode1 & 0x3f) << 26) | ((pads1 & 0x3) << 24) | (operand1 & 0xff) << 16) \
	| (((opcode0 & 0x3f) << 10) | ((pads0 & 0x3) << 8) | (operand0 & 0xff))


typedef struct {
	uint32_t version;
	int32_t (*init)(uint32_t instance, flexspi_norConfig_t *config);
	int32_t (*program)(uint32_t instance, flexspi_norConfig_t *config, uint32_t dstAddr, const uint32_t *src);
	int32_t (*erase_all)(uint32_t instance, flexspi_norConfig_t *config);
	int32_t (*erase)(uint32_t instance, flexspi_norConfig_t *config, uint32_t start, uint32_t lengthInBytes);
	int32_t (*read)(uint32_t instance, flexspi_norConfig_t *config, uint32_t *dst, uint32_t addr, uint32_t lengthInBytes);
	void (*clear_cache)(uint32_t instance);
	int32_t (*xfer)(uint32_t instance, flexspi_xfer_t *xfer);
	int32_t (*update_lut)(uint32_t instance, uint32_t seqIndex, const uint32_t *lutBase, uint32_t seqNumber);
	int32_t (*get_config)(uint32_t instance, flexspi_norConfig_t *config, serial_norConfigOption_t *option);
} flexspi_norDriverInterface_t;


static volatile flexspi_norDriverInterface_t *flexspi_norApi = (void *)FLEXSPI_DRIVER_API_ADDRESS;


int flexspi_norFlashInit(uint32_t instance, flexspi_norConfig_t *config)
{
	return flexspi_norApi->init(instance, config);
}


int flexspi_norFlashPageProgram(uint32_t instance, flexspi_norConfig_t *config, uint32_t dstAddr, const uint32_t *src)
{
	return flexspi_norApi->program(instance, config, dstAddr, src);
}


int flexspi_norFlashEraseAll(uint32_t instance, flexspi_norConfig_t *config)
{
	return flexspi_norApi->erase_all(instance, config);
}


int flexspi_norGetConfig(uint32_t instance, flexspi_norConfig_t *config, serial_norConfigOption_t *option)
{
	return flexspi_norApi->get_config(instance, config, option);
}


int flexspi_norFlashErase(uint32_t instance, flexspi_norConfig_t *config, uint32_t start, uint32_t length)
{
	return flexspi_norApi->erase(instance, config, start, length);
}


int flexspi_norFlashRead(uint32_t instance, flexspi_norConfig_t *config, uint32_t *dst, uint32_t start, uint32_t bytes)
{
	return flexspi_norApi->read(instance, config, dst, start, bytes);
}


int flexspi_norFlashUpdateLUT(uint32_t instance, uint32_t seqIndex, const uint32_t *lutBase, uint32_t seqNumber)
{
	return flexspi_norApi->update_lut(instance, seqIndex, lutBase, seqNumber);
}


int flexspi_norFlashExecuteSeq(uint32_t instance, flexspi_xfer_t *xfer)
{
	return flexspi_norApi->xfer(instance, xfer);
}
