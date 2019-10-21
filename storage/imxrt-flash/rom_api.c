/*
 * Phoenix-RTOS
 *
 * i.MX RT ROM API driver for FlexSPI
 *
 * Copyright 2019 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include "rom_api.h"

#define FLEXSPI_DRIVER_API_ADDRESS 0x00201a60

typedef struct {
	 uint32_t version;
	 int32_t (*init)(uint32_t instance, flexspi_nor_config_t *config);
	 int32_t (*program)(uint32_t instance, flexspi_nor_config_t *config, uint32_t dst_addr, const uint32_t *src);
	 int32_t (*erase_all)(uint32_t instance, flexspi_nor_config_t *config);
	 int32_t (*erase)(uint32_t instance, flexspi_nor_config_t *config, uint32_t start, uint32_t lengthInBytes);
	 int32_t (*read)(uint32_t instance, flexspi_nor_config_t *config, uint32_t *dst, uint32_t addr, uint32_t lengthInBytes);
	 void (*clear_cache)(uint32_t instance);
	 int32_t (*xfer)(uint32_t instance, flexspi_xfer_t *xfer);
	 int32_t (*update_lut)(uint32_t instance, uint32_t seqIndex, const uint32_t *lutBase, uint32_t seqNumber);
	 int32_t (*get_config)(uint32_t instance, flexspi_nor_config_t *config, serial_nor_config_option_t *option);
} flexspi_nor_driver_interface_t;


static volatile flexspi_nor_driver_interface_t* flexspi_nor_api = (void *)FLEXSPI_DRIVER_API_ADDRESS;


int flexspi_nor_flash_init(uint32_t instance, flexspi_nor_config_t* config)
{
	return flexspi_nor_api->init(instance, config);
}


int flexspi_nor_flash_page_program(uint32_t instance, flexspi_nor_config_t* config, uint32_t dstAddr, const uint32_t* src)
{
	return flexspi_nor_api->program(instance, config, dstAddr, src);
}


int flexspi_nor_flash_erase_all(uint32_t instance, flexspi_nor_config_t* config)
{
	return flexspi_nor_api->erase_all(instance, config);
}


int flexspi_nor_get_config(uint32_t instance, flexspi_nor_config_t* config, serial_nor_config_option_t* option)
{
	return flexspi_nor_api->get_config(instance, config, option);
}


int flexspi_nor_flash_erase(uint32_t instance, flexspi_nor_config_t* config, uint32_t start, uint32_t length)
{
	return flexspi_nor_api->erase(instance, config, start, length);
}


int flexspi_nor_flash_read(uint32_t instance, flexspi_nor_config_t* config, uint32_t* dst, uint32_t start, uint32_t bytes)
{
	return flexspi_nor_api->read(instance, config, dst, start, bytes);
}
