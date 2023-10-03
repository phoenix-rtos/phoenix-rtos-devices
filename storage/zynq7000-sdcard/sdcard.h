/*
 * Phoenix-RTOS
 *
 * SD card driver header file
 *
 * Copyright 2022, 2023 Phoenix Systems
 * Author: Artur Miller, Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _SDCARD_H_
#define _SDCARD_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define SDCARD_MAX_TRANSFER 4096 /* Maximum size of a single transfer in bytes */
#define SDCARD_BLOCKLEN     512  /* Block size in bytes used for sdcard_transferBlocks */

typedef enum {
	sdio_read,
	sdio_write
} sdio_dir_t;

typedef enum SDCARD_INSERTION {
	SDCARD_INSERTION_OUT = -1,
	SDCARD_INSERTION_UNSTABLE = 0,
	SDCARD_INSERTION_IN = 1,
} sdcard_insertion_t;

typedef int (*sdcard_event_handler_t)(unsigned int);


/* Returns > 0 if there are remaining slots to initialize, 0 if all slots were initialized, < 0 on failure */
extern int sdcard_initHost(unsigned int slot);


/* If fallbackMode == true, the card will be left in very low speed mode after initialization */
extern int sdcard_initCard(unsigned int slot, bool fallbackMode);


extern void sdcard_free(unsigned int slot);


extern int sdcard_transferBlocks(unsigned int slot, sdio_dir_t dir, uint32_t blockOffset, void *data, size_t len);


extern uint32_t sdcard_getSizeBlocks(unsigned int slot);


/* Returns the minimum number of blocks to be erased at a time */
extern uint32_t sdcard_getEraseSizeBlocks(unsigned int slot);


/* On SD cards erase is not necessary to write blocks.
 * State after erase is dependent on implementation of the SD card.
 */
extern int sdcard_eraseBlocks(unsigned int slot, uint32_t blockOffset, uint32_t nBlocks);


/* Sets all bytes in selected range of blocks to a value of 0xFF.
 * This is meant for emulating erase behavior of NOR flash.
 */
extern int sdcard_writeFF(unsigned int slot, uint32_t blockOffset, uint32_t nBlocks);


/* Handle card presence state (card is in our out). Should be called once at startup to handle initial state. */
extern void sdcard_handlePresence(sdcard_event_handler_t onInsert, sdcard_event_handler_t onRemove);


/* Handle changes in card presence state (card is inserted or removed) in an infinite loop */
extern void sdcard_presenceThread(sdcard_event_handler_t onInsert, sdcard_event_handler_t onRemove);


sdcard_insertion_t sdcard_isInserted(unsigned int slot);

#endif /* _SDCARD_H_ */
