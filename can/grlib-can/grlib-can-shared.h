/*
 * Phoenix-RTOS
 *
 * GRCANFD driver
 *
 * GRLIB CANFD driver file
 *
 * Copyright 2025 Phoenix Systems
 * Author: Mikolaj Matalowski
 *
 * %LICENSE%
 */

#ifndef GRLIB_CAN_SHARED_H
#define GRLIB_CAN_SHARED_H

#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/msg.h>

typedef struct
{
	union {
		struct {
			uint32_t head; /* Head contains CAN packet mode and IDs */
			uint32_t stat;

			uint8_t payload[8]; /* Payload */
		} frame;

		uint8_t payload[16];
	};
} grlibCan_msg_t;

typedef struct
{
	/* Base configuration */
	uint32_t conf;
	uint32_t syncMask;
	uint32_t syncCode;

	uint32_t nomBdRate;    /* Nominal baud-rate */
	uint32_t dataBdRate;   /* Data transfer baud-rate */
	uint32_t transDelComp; /* Tranmission delay compensation */

	/* CANopen currently omitted */

	/* TX configuration */
	uint32_t txCtrlReg;

	/* RX configuration */
	uint32_t rxCtrlReg;
	uint32_t rxAccMask;
	uint32_t rxAccCode;
} grlibCan_config_t;

typedef struct {
	enum { can_setConfig = 0,
		can_getConfig,
		can_getStatus,
		can_reset,
		can_writeSync,
		can_readSync,
		can_writeAsync,
		can_readAsync } type;
} grlibCan_devCtrl_t;

#endif
