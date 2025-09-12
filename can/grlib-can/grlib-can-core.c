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

/* System includes */
#include <errno.h>
#include <paths.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libklog.h>
#include <float.h>

#include <sys/debug.h>
#include <sys/file.h>
#include <sys/msg.h>
#include <sys/interrupt.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/threads.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <posix/utils.h>

/* Platform specific includes */
#include <board_config.h>
#include <sys/platform.h>
#include <phoenix/gaisler/ambapp.h>
#include <phoenix/arch/riscv64/riscv64.h>

/* Local includes */
#include "grlib-can-core.h"

/* Interrupt handler */
int __attribute__((section(".interrupt"), aligned(0x1000))) grlibCan_irqHandler(unsigned int irqNum, void *arg)
{
	grlibCan_dev_t *dev = (grlibCan_dev_t *)arg;
	grlibCan_hwDev_t *device = dev->device;

	int ret = 0;

	/* Handlers for SYNC TX/RX */
	if ((device->penIsrMsk & GRLIB_CAN_RX_IRQ) != 0) {
		dev->rxBufStatus = rxBufReady;
		dev->device->irqMskSet &= ~(GRLIB_CAN_RX_IRQ);
		ret = 1;
	}

	if ((device->penIsrMsk & GRLIB_CAN_TX_IRQ) != 0) {
		dev->txBufStatus = txBufReady;
		dev->device->irqMskSet &= ~GRLIB_CAN_TX_IRQ;
		ret = 1;
	}

	if ((device->penIsrMsk & (GRLIB_CAN_BMRDERR_IRQ | GRLIB_CAN_BMWRERR_IRQ | GRLIB_CAN_OR_IRQ | GRLIB_CAN_BUSSOFF_IRQ | GRLIB_CAN_PASS_IRQ)) != 0) {
		/* Error on the bus occurred */
		/* Turn off codec and set device state to error */
		dev->device->ctrlReg &= ~1;
		dev->deviceStatus = deviceError;
	}

	/* Clear interrupt register */
	device->irqClrReg = ~(0);

	return ret;
}

/* Pop one frame from RX channel */
int grlibCan_popFrame(grlibCan_dev_t *dev, grlibCan_msg_t *msg)
{
	uint32_t rdPtr = (dev->device->rxRdPtr) >> 4;
	(void)memcpy((void *)msg, (void *)(&dev->rxBufAdd[rdPtr]), sizeof(grlibCan_msg_t));
	dev->device->rxRdPtr = (((rdPtr + 1) % dev->rxBufSz) << 4);
	return 0;
}

/* Push one frame into TX channel */
int grlibCan_pushFrame(grlibCan_dev_t *dev, const grlibCan_msg_t *msg)
{
	uint32_t wrPtr = (dev->device->txWrtPtr) >> 4;
	(void)memcpy((void *)(&dev->txBufAdd[wrPtr]), (void *)msg, sizeof(grlibCan_msg_t));
	dev->device->txWrtPtr = (((wrPtr + 1) % dev->txBufSz) << 4);
	return 0;
}

static inline double getBd_nom(uint32_t reg)
{
	reg = reg >> 5;
	uint32_t PS2 = reg & ((~(uint32_t)0) >> 27);
	reg = reg >> 5;
	uint32_t PS1 = reg & ((~(uint32_t)0) >> 26);
	reg = reg >> 6;
	uint32_t SC = reg & ((~(uint32_t)0) >> 24);
	uint32_t a = (SC + 1) * (PS1 + PS2 + 1);
	return (double)(SYSCLK_FREQ) / ((double)a);
}

static inline double getBd_data(uint32_t reg)
{
	reg = reg >> 5;
	uint32_t PS2 = reg & ((~(uint32_t)0) >> 28);
	reg = reg >> 5;
	uint32_t PS1 = reg & ((~(uint32_t)0) >> 28);
	reg = reg >> 6;
	uint32_t SC = reg & ((~(uint32_t)0) >> 24);
	uint32_t a = (SC + 1) * (PS1 + PS2 + 1);
	return (double)(SYSCLK_FREQ) / ((double)a);
}

void grlibCan_copyConfig(grlibCan_dev_t *device, grlibCan_config_t *config)
{
	config->conf = device->device->confReg;
	config->syncMask = device->device->syncMask;
	config->syncCode = device->device->syncCode;

	config->nomBdRate = (uint32_t)getBd_nom(device->device->nomTimConf);
	config->dataBdRate = (uint32_t)getBd_data(device->device->dataTimConf);

	config->txCtrlReg = device->device->txCtrlReg;
	config->rxCtrlReg = device->device->rxCtrlReg;
	config->rxAccMask = device->device->rxAccMask;
	config->rxAccCode = device->device->rxAccCode;
}

int grlibCan_resetDevice(grlibCan_dev_t *dev)
{
	dev->device->ctrlReg |= 1 << 1;
	while ((dev->device->ctrlReg & (1 << 1)) != 0) { }
	dev->device->ctrlReg |= 1;
	grlibCan_applyDefConf(dev);
	dev->deviceStatus = deviceReady;
	return 0;
}

/* Apply default config */
int grlibCan_applyDefConf(grlibCan_dev_t *dev)
{
	volatile grlibCan_hwDev_t *device = dev->device;

	mutexLock(dev->ctrlLock);

	/* Reset device and turn off codec */
	device->ctrlReg &= ~1;
	device->ctrlReg = (1 << 1);

	/* Sleeping until reset finished */
	while ((device->ctrlReg & (1u << 1)) != 0) { }

	/* Configure ids to listen to */
	device->confReg = 0;
	device->confReg |= (1 << 2) | (1 << 1) | (1 << 6);

	/* Do no compare on any bits - listen to all */
	device->syncMask = (0u);
	device->syncCode = ~(0u);

	/* Set baud-rate for both nominal and data */
	device->nomTimConf = grlibCan_setBdRateNom(125000.0, device->nomTimConf);
	device->dataTimConf = grlibCan_setBdRateData(125000.0, device->dataTimConf);
	device->transDelCompReg = 0;

	/* Turn off CANopen */
	device->copReg &= ~1;

	/* Configure interrupts */
	volatile uint32_t pending = device->penIrq;
	(void)pending;

	/* Enable error interrupts */
	device->irqMskSet = GRLIB_CAN_BMRDERR_IRQ | GRLIB_CAN_BMWRERR_IRQ |
			GRLIB_CAN_OR_IRQ | GRLIB_CAN_BUSSOFF_IRQ | GRLIB_CAN_PASS_IRQ;

	/* Configure TX channel */
	device->txCtrlReg &= ~1;
	while ((device->txCtrlReg & (1 << 1)) != 0) { }


	device->txBufAdd = (uint32_t)va2pa((void *)dev->txBufAdd);
	device->txBufSz = (uint32_t)((dev->txBufSz / 4) << 6);
	device->txWrtPtr = 0;
	device->txRdPtr = 0;

	device->txCtrlReg &= ~(1 << 2);
	device->txCtrlReg |= 1;

	/* Configure RX channel */
	device->rxCtrlReg &= ~1;
	while ((device->rxCtrlReg & (1 << 1)) != 0) { }

	device->rxBufAdd = (uint32_t)va2pa((void *)dev->rxBufAdd);
	device->rxBufSz = (dev->rxBufSz / 4) << 6;
	device->rxWrtPtr = 0;
	device->rxRdPtr = 0;
	/* Put all frames that we got from the bus into the buffer */
	device->rxAccCode = (0u);
	device->rxAccMask = (0u);

	device->rxCtrlReg &= ~(1 << 3);
	device->rxCtrlReg |= 1;

	/* Turn codec back on */
	device->ctrlReg |= 1;

	mutexUnlock(dev->ctrlLock);
	return 0;
}

/* Apply user provided config */
int grlibCan_applyConfig(grlibCan_dev_t *dev, grlibCan_config_t *config)
{
	grlibCan_hwDev_t *device = dev->device;

	mutexLock(dev->ctrlLock);
	mutexLock(dev->txLock);
	mutexLock(dev->rxLock);

	/* Turn off codec */
	device->ctrlReg &= ~(1);
	while ((device->ctrlReg & (1u << 1)) != 0) { }

	/* Apply user defined configuration */
	device->confReg = config->conf;

	device->syncMask = config->syncMask;
	device->syncCode = config->syncCode;

	/* Set timing configuration */
	device->nomTimConf = grlibCan_setBdRateNom((double)config->nomBdRate, device->nomTimConf);
	device->dataTimConf = grlibCan_setBdRateData((double)config->dataBdRate, device->dataTimConf);
	device->transDelCompReg = config->transDelComp;

	/* Set TX and RX config */
	device->txCtrlReg = config->txCtrlReg;
	device->rxCtrlReg = config->rxCtrlReg;
	device->rxAccCode = config->rxAccCode;
	device->rxAccMask = config->rxAccMask;

	/* Turn codec back on */
	device->ctrlReg |= 1;

	mutexUnlock(dev->ctrlLock);
	mutexUnlock(dev->txLock);
	mutexUnlock(dev->rxLock);
	return 0;
}

/* Transmit buffer of frames in blocking mode */
int grlibCan_transmitSync(grlibCan_dev_t *dev, const grlibCan_msg_t *buffer,
		const uint32_t length)
{
	if (dev->deviceStatus == deviceError) {
		return 0;
	}
	uint32_t i = 0;

	(void)mutexLock(dev->txLock);

	dev->txStatus = txOngoing;

	while (i < length) {
		/* Check for bus status, if error return how many frames sent */
		if (dev->deviceStatus == deviceError) {
			(void)mutexUnlock(dev->txLock);
			return -i;
		}

		uint32_t wrPtr = dev->device->txWrtPtr >> 4;
		uint32_t rdPtr = dev->device->txRdPtr >> 4;

		/* While there is space in transmit buffer push frames */
		while (wrPtr != (rdPtr > 0 ? (rdPtr - 1) : (dev->txBufSz - 1))) {
			if (i == length) {
				(void)mutexUnlock(dev->txLock);
				return length;
			}

			(void)grlibCan_pushFrame(dev, &buffer[i]);
			i++;

			wrPtr = dev->device->txWrtPtr >> 4;
			rdPtr = dev->device->txRdPtr >> 4;
		}

		/* TX buffer has been filled */
		dev->txBufStatus = txBufFull;
		dev->device->irqMskSet |= GRLIB_CAN_TX_IRQ;

		while (dev->txBufStatus == txBufFull) {
			if (i == length) {
				break;
			}
			/* Wait for interrupt to clear TX_FULL */
			(void)condWait(dev->cond, dev->txLock, 1000000);
		}
	}

	dev->txStatus = txDone;

	(void)mutexUnlock(dev->txLock);
	return i;
}

/* Transmit buffer of frames in non-blocking mode */
int grlibCan_transmitAsync(grlibCan_dev_t *dev, const grlibCan_msg_t *buffer,
		const uint32_t length)
{
	if (dev->deviceStatus == deviceError) {
		return 0;
	}
	uint32_t i = 0;

	(void)mutexLock(dev->txLock);

	uint32_t wrPtr = dev->device->txWrtPtr >> 4;
	uint32_t rdPtr = dev->device->txRdPtr >> 4;

	/* While there is space in transmit buffer push frames */
	while (wrPtr != (rdPtr > 0 ? (rdPtr - 1) : (dev->txBufSz - 1))) {
		if (i == length) {
			break;
		}
		(void)grlibCan_pushFrame(dev, &buffer[i]);
		i++;

		wrPtr = dev->device->txWrtPtr >> 4;
		rdPtr = dev->device->txRdPtr >> 4;
	}

	(void)mutexUnlock(dev->txLock);
	return i;
}

/* Receive length frames in blocking mode */
int grlibCan_recvSync(grlibCan_dev_t *dev, grlibCan_msg_t *buffer,
		uint32_t length, uint32_t *pending)
{
	uint32_t i = 0;
	*pending = 0;

	(void)mutexLock(dev->rxLock);

	while (i < length) {

		/* Check for bus status, if error return how many frames read */
		if (dev->deviceStatus == deviceError) {
			(void)mutexUnlock(dev->rxLock);
			return -i;
		}

		/* While there are frames to read, process them */
		while (dev->device->rxRdPtr != dev->device->rxWrtPtr) {
			if (i == length) {
				(void)mutexUnlock(dev->rxLock);
				return length;
			}
			/* Check if whole CAN frame can be processed */
			uint32_t frame_len = ((dev->rxBufAdd[dev->device->rxRdPtr >> 4]).frame.stat >> 28);
			if (frame_len > 8) {
				/* Whole frame cannot be consumed */
				if ((frame_len - 8) / sizeof(grlibCan_msg_t) + 1 >= length - i) {
					(void)mutexUnlock(dev->rxLock);
					*pending = (frame_len - 8) / sizeof(grlibCan_msg_t) + 1;
					return i;
				}
				else {
					/* Read whole packet and continue*/
					for (int j = 0; j < (frame_len - 8) / sizeof(grlibCan_msg_t) + 1; j++) {
						(void)grlibCan_popFrame(dev, &buffer[i]);
						i++;
						continue;
					}
				}
			}
			/* Read one packet */
			(void)grlibCan_popFrame(dev, &buffer[i]);
			i++;
		}

		/* RX buffer is empty */
		dev->rxBufStatus = rxBufEmpty;
		dev->device->irqMskSet |= GRLIB_CAN_RX_IRQ;

		while (dev->rxBufStatus == rxBufEmpty) {
			if (i == length) {
				break;
			}
			(void)condWait(dev->cond, dev->rxLock, 100000);
		}
	}

	(void)mutexUnlock(dev->rxLock);
	return i;
}

/* Receive at most length frames in non-blocking mode */
int grlibCan_recvAsync(grlibCan_dev_t *dev, grlibCan_msg_t *buffer,
		const uint32_t length)
{
	if (dev->deviceStatus == deviceError) {
		return 0;
	}
	uint32_t i = 0;

	(void)mutexLock(dev->rxLock);

	/* While there are frames to read, process them */
	while (dev->device->rxRdPtr != dev->device->rxWrtPtr && i < length) {
		/* Check if whole CAN frame can be processed */
		uint32_t frame_len = ((dev->rxBufAdd[dev->device->rxRdPtr >> 4]).frame.stat >> 28);
		if (frame_len > 8) {
			/* Whole frame cannot be consumed */
			if ((frame_len - 8) / sizeof(grlibCan_msg_t) + 1 >= length - i) {
				(void)mutexUnlock(dev->rxLock);
				return i;
			}
			else {
				/* Read whole packet and continue*/
				for (int j = 0; j < (frame_len - 8) / sizeof(grlibCan_msg_t) + 1; j++) {
					(void)grlibCan_popFrame(dev, &buffer[i]);
					i++;
					continue;
				}
			}
		}
		/* Read one packet */
		(void)grlibCan_popFrame(dev, &buffer[i]);
		i++;
	}

	(void)mutexUnlock(dev->rxLock);
	return i;
}

/* Allocate TX/RX circular buffers */
int grlibCan_allocateBuffers(grlibCan_dev_t *dev)
{
	dev->txBufAdd = mmap(NULL, _PAGE_SIZE,
			PROT_WRITE | PROT_READ, MAP_UNCACHED | MAP_ANONYMOUS, -1, 0);
	if (dev->txBufAdd == MAP_FAILED) {
		return -ENOMEM;
	}

	dev->txBufSz = (_PAGE_SIZE / (sizeof(grlibCan_msg_t) * 4)) * 4;

	dev->rxBufAdd = mmap(NULL, _PAGE_SIZE,
			PROT_WRITE | PROT_READ, MAP_UNCACHED | MAP_ANONYMOUS, -1, 0);
	if (dev->rxBufAdd == MAP_FAILED) {
		(void)munmap((void *)dev->txBufAdd, _PAGE_SIZE);
		dev->txBufSz = 0;
		return -ENOMEM;
	}

	dev->rxBufSz = (_PAGE_SIZE / (sizeof(grlibCan_msg_t) * 4)) * 4;
	return 0;
}

/* Allocate resources for the devices */
int grlibCan_allocateResources(grlibCan_dev_t *dev, uintptr_t base, int id)
{
	dev->ownerPid = 0;
	dev->ctrlLock = (handle_t)-1;
	dev->cond = (handle_t)-1;
	dev->txLock = (handle_t)-1;
	dev->rxLock = (handle_t)-1;
	dev->txBufAdd = MAP_FAILED;
	dev->rxBufAdd = MAP_FAILED;
	dev->txBufSz = 0;
	dev->rxBufSz = 0;

	uintptr_t offset = (base & ~(_PAGE_SIZE - 1));
	dev->device = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)offset);

	if (dev->device == MAP_FAILED) {
		debug("grlib-can: Failed to map device physical address\n");
		return -1;
	}

	if (grlibCan_allocateBuffers(dev) < 0) {
		debug("grlib-can: Failed to allocate circular buffers\n");
		return -1;
	}

	if (mutexCreate(&dev->ctrlLock) < 0) {
		debug("grlib-can: Failed to create lock for config\n");
		return -1;
	}

	if (mutexCreate(&dev->txLock) < 0) {
		debug("grlib-can: Failed to create lock for config\n");
		return -1;
	}

	if (mutexCreate(&dev->rxLock) < 0) {
		debug("grlib-can: Failed to create lock for config\n");
		return -1;
	}

	if (condCreate(&dev->cond) < 0) {
		debug("grlib-can: Failed to create conditional\n");
		return -1;
	}

#ifdef VERBOSE
	debug("grlib-can: Managed to allocate resources\n");
#endif

	return 0;
}

/* Cleanup resources */
void grlibCan_cleanupResources(grlibCan_dev_t *dev)
{
	if (dev->device != MAP_FAILED) {
		(void)munmap((void *)dev->device, _PAGE_SIZE);
	}

	if (dev->txBufAdd != MAP_FAILED) {
		(void)munmap((void *)dev->txBufAdd, _PAGE_SIZE);
	}

	if (dev->rxBufAdd != MAP_FAILED) {
		(void)munmap((void *)dev->rxBufAdd, _PAGE_SIZE);
	}

	if (dev->ctrlLock != (handle_t)-1) {
		resourceDestroy(dev->ctrlLock);
	}

	if (dev->rxLock != (handle_t)-1) {
		resourceDestroy(dev->rxLock);
	}

	if (dev->txLock != (handle_t)-1) {
		resourceDestroy(dev->txLock);
	}

	if (dev->cond != (handle_t)-1) {
		resourceDestroy(dev->cond);
	}
}

/* Initialise devices */
int grlibCan_initDevices(grlibCan_dev_t *devices, int num)
{
	for (int i = 0; i < num; i++) {
		if (devices[i].canId == -1)
			continue;

		if (grlibCan_allocateResources(&devices[i], (uintptr_t)devices[i].device, 0) < 0) {
			grlibCan_cleanupResources(&devices[i]);
			return -1;
		}

		if (interrupt(devices[i].irqNum, grlibCan_irqHandler,
					(void *)&devices[i], devices[i].cond, NULL) < 0) {
			debug("grlib-can: Failed to set up the interrupt\n");
		}

#ifdef VERBOSE
		debug("grlib-can: Managed to register callback to device interrupt\n");
#endif

		/* Now having memory for all needed operations we can configure the device */
		grlibCan_applyDefConf(&devices[i]);

#ifdef VERBOSE
		debug("grlib-can: Applied default config to the device\n");
#endif
	}
	return 0;
}

/* Register devices in the file system */
int grlibCan_registerDevices(oid_t *oid, grlibCan_dev_t *devices, uint32_t length)
{
	oid_t dir;

	if (lookup("/dev", &dir, NULL) < 0) {
		debug("grlib-can: Failed to create devices\n");
		return -1;
	}

	for (int i = 0; i < length; i++) {
		char buf[24];
		if (snprintf(buf, sizeof(buf), "can%d", i) >= sizeof(buf)) {
			debug("grlib-can: Failed to create devices\n");
			return -1;
		}
		oid->id = i;

		if (create_dev(oid, buf) < 0) {
			debug("grlib-can: Failed to create devices\n");
			return -1;
		}
	}

	return 0;
}

/* Unregister devices from file system */
int grlibCan_unregisterDevices(oid_t *port, grlibCan_dev_t *devices, uint32_t length)
{
	oid_t dir;

	while (lookup("/dev", NULL, &dir) < 0) {
		usleep(100000);
	}

	for (int i = 0; i < length; i++) {
		if (devices[i].canId == -1)
			continue;
		char buf[24];
		if (snprintf(buf, sizeof(buf), "/dev/can%d", i) >= sizeof(buf)) {
			printf("grlib-can: Failed to remove can%d\n", i);
			continue;
		}
		port->id = i;
		if (unlink(buf) < 0) {
			printf("grlib-can: Failed to remove can%d\n", i);
		}
	}

	return 0;
}

static inline double getBd(uint16_t SC, uint16_t PS1, uint16_t PS2)
{
	uint32_t a = (SC + 1) * (PS1 + PS2 + 1);
	return (double)(SYSCLK_FREQ) / ((double)a);
}
/* Search for most optimal configuration to set given baud rate */
uint32_t grlibCan_setBdRateData(double dataBdRate, uint32_t prevSetting)
{
	uint16_t optSC = 0;
	uint16_t optPS1 = 0;
	uint16_t optPS2 = 0;
	double optDiff = DBL_MAX;

	for (uint16_t SC = 0; SC <= 255; SC++) {
		for (uint16_t PS1 = 1; PS1 <= 15; PS1++) {
			for (uint16_t PS2 = 2; PS2 <= 8; PS2++) {
				double diff = dataBdRate - getBd(SC, PS1, PS2);
				if (diff < 0) {
					diff = -diff;
				}
				if (diff < optDiff) {
					optSC = SC;
					optPS1 = PS1;
					optPS2 = PS2;
					optDiff = diff;
				}
				if (diff <= DBL_EPSILON) {
					break;
				}
			}
		}
	}

	if (optDiff >= 0.2 * dataBdRate) {
		return prevSetting;
	}

	return (optSC << 16) | (optPS1 << 10) | (optPS2 << 5) | (optPS1 < optPS2 ? optPS1 : optPS2);
}

uint32_t grlibCan_setBdRateNom(double nomBdRate, uint32_t prevSetting)
{
	uint16_t optSC = 0;
	uint16_t optPS1 = 0;
	uint16_t optPS2 = 0;
	double optDiff = DBL_MAX;

	for (int SC = 0; SC <= 255; SC++) {
		for (int PS1 = 2; PS1 <= 63; PS1++) {
			for (int PS2 = 2; PS2 <= 16; PS2++) {
				double diff = nomBdRate - getBd(SC, PS1, PS2);
				if (diff < 0) {
					diff = -diff;
				}
				if (diff < optDiff) {
					optSC = SC;
					optPS1 = PS1;
					optPS2 = PS2;
					optDiff = diff;
				}
				if (diff <= DBL_EPSILON) {
					break;
				}
			}
		}
	}

	if (optDiff >= 0.2 * nomBdRate) {
		return prevSetting;
	}

	return (optSC << 16) | (optPS1 << 10) | (optPS2 << 5) | (optPS1 < optPS2 ? optPS1 : optPS2 - 1);
}

int grlibCan_queryForDevices(grlibCan_dev_t *dev)
{
	int detectedDevices = 0;
	/* System query for number of GRCAN controllers and their properties */
	for (unsigned int i = 0; i < GRLIB_MAX_CAN_DEVICES; i++) {
		unsigned int id = i;

		ambapp_dev_t device = { .devId = CORE_ID_GRCANFD };
		platformctl_t ctl = {
			.action = pctl_get,
			.type = pctl_ambapp,
			.task.ambapp.dev = &device,
			.task.ambapp.instance = &id
		};

		if (platformctl(&ctl) < 0) {
			break;
		}

		detectedDevices++;

		if (device.bus == BUS_AMBA_AHB) {
			dev[i].canId = -1;
			dev[i].irqNum = 0;
			dev[i].device = 0;
			/* GRCANFD should be on APB bus */
			continue;
		}
		else if (device.bus == BUS_AMBA_APB) {
			dev[i].canId = device.devId;
			dev[i].irqNum = device.irqn;
			dev[i].device = (grlibCan_hwDev_t *)device.info.apb.base;
		}
	}
	return detectedDevices;
}
