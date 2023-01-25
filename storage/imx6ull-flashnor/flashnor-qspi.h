/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL QSPI NOR flash driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Hubert Badocha
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASHNOR_QSPI_H_
#define _FLASHNOR_QSPI_H_

#include <unistd.h>
#include <qspi.h>

#include "storage.h"


ssize_t flashnor_qspiRead(qspi_dev_t dev, unsigned int addr, void *buff, size_t bufflen);


ssize_t flashnor_qspiWrite(qspi_dev_t dev, unsigned int addr, const void *buff, size_t bufflen);


int flashnor_qspiEraseSector(qspi_dev_t dev, unsigned int addr);


int _flashnor_qspiInit(qspi_dev_t dev, storage_t *storage_dev);


#endif
