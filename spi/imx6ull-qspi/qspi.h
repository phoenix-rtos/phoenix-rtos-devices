/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL QSPI lib API
 *
 * Copyright 2023 Phoenix Systems
 * Author: Hubert Badocha
 *
 * %LICENSE%
 */

#ifndef IMX6ULL_QSPI_API_H
#define IMX6ULL_QSPI_API_H

#include <stddef.h>
#include <stdint.h>

#include <sys/threads.h>
#include <phoenix/arch/imx6ull.h>


#define MAX_READ_LEN  0xFFFF
#define MAX_WRITE_LEN 0xFFFF

typedef enum {
	lut_stop,
	lut_cmd,
	lut_addr,
	lut_dummy,
	lut_mode,
	lut_mode2,
	lut_mode4,
	lut_read,
	lut_write,
	lut_jmp_on_cs,
	lut_addr_ddr,
	lut_mode_ddr,
	lut_mode2_ddr,
	lut_mode4_ddr,
	lut_read_ddr,
	lut_write_ddr,
	lut_data_learn
} lut_code_t;

#define LUT_INSTR(code, pad, operand) ((((code)&0x3F) << 10) | (((pad)&0x03) << 8) | ((operand)&0xFF))

typedef struct {
	uint16_t instrs[8];
} lut_seq_t;

typedef enum {
	qspi_flash_a,
	qspi_flash_b,
} qspi_dev_t;

int _qspi_init(qspi_dev_t dev);

int qspi_setLutSeq(const lut_seq_t *lut, unsigned int lut_seq);

// int qspi_setClockDiv(qspi_ctx_t* ctx, uint8_t pre, uint8_t post);
void qspi_setTCSH(uint8_t cycles);
void qspi_setTCSS(uint8_t cycles);

/* TODO non busy read. */
int _qspi_read(qspi_dev_t dev, unsigned int lut_seq, uint32_t addr, void *buf, size_t size);
int _qspi_write(qspi_dev_t dev, unsigned int lut_seq, uint32_t addr, const void *data, size_t size);


#endif /* IMX6ULL_QSPI_API_H */
