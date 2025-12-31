#ifndef _LIB_SPI_H_
#define _LIB_SPI_H_

#include <stddef.h>
#include <stdint.h>


int spi_configure(uint32_t spi, unsigned char cs, uint32_t bdiv, uint32_t prescaler, uint32_t endian, uint32_t mode);


int spi_transaction(uint32_t spi, unsigned char cs, const uint8_t *txBuff, uint8_t *rxBuff, size_t len);


#endif
