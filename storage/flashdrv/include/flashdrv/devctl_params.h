/*
 * Phoenix-RTOS
 *
 * Devctl params
 *
 * Copyright 2026 Phoenix Systems
 * Author: Amelia Waszkowska
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _DEVCTL_PARAMS_H_
#define _DEVCTL_PARAMS_H_


enum {
	flashdrv_devctl_Erase,
	flashdrv_devctl_SPIMode,
	flashdrv_devctl_MAX
};


typedef enum {
	flashdrv_devctl_eraseSector,
    flashdrv_devctl_erasePartition,
	flashdrv_devctl_eraseChip,
	Erase_MAX
} EraseType_t;


typedef enum {
	BSPI, /* Basic SPI */
	DSPI, /* Dual SPI */
	QOUT, /* Quad Output SPI */
	QSPI, /* Quad SPI */
	SPI_MAX
} SPIMode_t;


#endif /* _DEVCTL_PARAMS_H_ */