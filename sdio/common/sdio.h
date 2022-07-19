/*
 * Phoenix-RTOS
 *
 * SDIO driver header
 *
 * Copyright 2022 Phoenix Systems
 * Author: Artur Miller
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _PHOENIX_SDIO_H
#define _PHOENIX_SDIO_H

#include <stdint.h>
#include <stddef.h>


/* SDIO interrupt events */
#define SDIO_EVENT_CARD_IN  0 /* card inserted            */
#define SDIO_EVENT_CARD_OUT 1 /* card removed             */
#define SDIO_EVENT_CARD_IRQ 2 /* card requested interrupt */

typedef enum {
	sdio_read,
	sdio_write
} sdio_dir_t;


/* interrupt event handler pointer */
typedef void (*sdio_event_handler_t)(void *arg);


/* Enables SDIO module */
extern int sdio_init(void);


/* Configures hardware module */
extern int sdio_config(uint32_t freq, uint16_t blocksz);


/* Disables SDIO module */
extern void sdio_free(void);


/* Transfers a single byte of data */
extern int sdio_transferDirect(sdio_dir_t dir, uint32_t address, uint8_t area, uint8_t *data);


/* Transfers to/from device synchronously */
extern int sdio_transferBulk(sdio_dir_t dir, int blockMode, uint32_t address, uint8_t area,
	uint8_t *data, size_t len);


/* Registers an interrupt event handler */
extern int sdio_eventRegister(uint8_t event, sdio_event_handler_t handler, void *arg);


/* Sets interrupt event detector state */
extern int sdio_eventEnable(uint8_t event, int enabled);


#endif /* _PHOENIX_SDIO_H */
