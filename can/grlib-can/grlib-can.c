/*
 * Phoenix-RTOS
 *
 * GRCANFD driver
 *
 * GRLIB CANFD driver source file
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

#include <sys/debug.h>
#include <sys/file.h>
#include <sys/msg.h>
#include <sys/interrupt.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/threads.h>
#include <posix/utils.h>
#include <sys/time.h>
#include <sys/mman.h>

/* Local includes */
#include "grlib-can.h"

#pragma GCC optimize("O0")

void sendingThread(void *args)
{
	grlibCan_dev_t *dev = (grlibCan_dev_t *)args;
	grlibCan_msg_t frame[GRLIB_CAN_DEF_CIRCBUFSZ * 2];

	frame->frame.head = (1 << 18);
	frame->frame.stat = 0x10u << 24u;
	frame->frame.payload[0] = 0x0F;

	for (int i = 1; i < GRLIB_CAN_DEF_CIRCBUFSZ * 2; i++) {
		memcpy((void *)&frame[i], (void *)frame, sizeof(grlibCan_msg_t));
		frame[i].frame.payload[0] = 0x0F + i;
	}

	usleep(1000000);
	printf("Sending messages\n");

	for (int i = 0; i < 10; i++) {
		grlibCan_transmitSync(dev, frame, GRLIB_CAN_DEF_CIRCBUFSZ);
	}

	printf("Bursting finished\n");

	for (;;) {
		sleep(100000);
	}
}

uint8_t stack[2048];

static void __attribute__((used)) debug_routine(void *args)
{
	debug("Started debug routine\n");
	usleep(1000000);
	debug("debug routine\n");
	grlibCan_dev_t *dev = (grlibCan_dev_t *)args;

	grlibCan_msg_t frames[GRLIB_CAN_DEF_CIRCBUFSZ * 20];

	beginthread(sendingThread, 4, stack, 2048, dev);

	debug("Receiving frames\n");
	int recved = grlibCan_recvSync(dev, frames, 160);

	printf("Received: %d\n", recved);
	//printf("Number of interrupt handler calls: %d, rxIRQ, %d\n", dev->counter, dev->recv);

	// grlibCan_transmitAsync(dev, frames, 0);
	// grlibCan_recvAsync(dev, frames, 0);

	debug("Transmission successful\n");

	for (;;) {
	}
}


static void __attribute__((used)) grlibCan_messageThread(void *args)
{
	grlibCan_driver_t *driverInstance = (grlibCan_driver_t *)args;

#ifdef VERBOSE
	debug("grlib-can: Entered main message thread\n");
#endif

	msg_t msg;
	msg_rid_t rid;

	for (;;) {
		while (msgRecv(driverInstance->port, &msg, &rid) < 0) {
			switch (msg.type) {
				case mtRead:
				case mtWrite:
				case mtDevCtl:

					break;

				default:
					msg.o.err = -ENOSYS;
					break;
			}

			msgRespond(driverInstance->port, &msg, rid);
		}
	}
}

/* Interrupt handler */
static int __attribute__((section(".interrupt"), aligned(0x1000))) grlibCan_irqHandler(unsigned int irqNum, void *arg)
{
	grlibCan_dev_t *dev = (grlibCan_dev_t *)arg;
	volatile grlibCan_hwDev_t *device = dev->device;
	volatile uint32_t pending = device->penIsr;

	int ret = 0;

	/* Handlers for SYNC TX/RX */
	if ((pending & GRLIB_CAN_rx_IRQ) != 0) {
		ret = 1;
		dev->rxBufStatus = RX_RDY;
		dev->recv++;
		dev->device->irqClrReg |= GRLIB_CAN_rx_IRQ;
	}

	if ((pending & GRLIB_CAN_tx_IRQ) != 0) {
		ret = 1;
		dev->txBufStatus = TX_RDY;
		dev->sent++;
	}
	if ((pending & GRLIB_CAN_txEmpty_IRQ) != 0) {
		ret = 1;
		dev->txBufStatus = TX_RDY;
	}

	if (((pending)&GRLIB_CAN_txPtr_IRQ) != 0) {
		ret = 1;
		dev->txTransactionStatus = GRLIB_CAN_TRANSATION_FINISHED;
		dev->txBufStatus = TX_RST;
	}

	if ((pending & GRLIB_CAN_txLoss_irq) != 0) {
		dev->txErr++;
	}

	if ((pending & GRLIB_CAN_or_IRQ) != 0) {
		dev->overRun++;
	}

	if ((pending & GRLIB_CAN_rxMiss_irq) != 0) {
		dev->rxMiss++;
	}

	if ((pending & GRLIB_CAN_rxErrCnt_IRQ) != 0) {
		dev->rxErrCntr++;
	}

	dev->counter++;

	dev->device->irqClrReg = ~(GRLIB_CAN_rx_IRQ);

	return ret;
}

static int grlibCan_popFrame(grlibCan_dev_t *dev, grlibCan_msg_t *msg)
{
	uint32_t rdPtr = (dev->device->rxRdPtr) >> 4;
	(void)memcpy((void *)msg, (void *)(&dev->rxBufAdd[rdPtr]), sizeof(grlibCan_msg_t));
	dev->device->rxRdPtr = (((rdPtr + 1) % dev->rxBufSz) << 4);
	return 0;
}

static int grlibCan_pushFrame(grlibCan_dev_t *dev, const grlibCan_msg_t *msg)
{
	uint32_t wrPtr = (dev->device->txWrtPtr) >> 4;
	(void)memcpy((void *)(&dev->txBufAdd[wrPtr]), (void *)msg, sizeof(grlibCan_msg_t));
	dev->device->txWrtPtr = (((wrPtr + 1) % dev->txBufSz) << 4);
	return 0;
}

static int grlibCan_applyDefConf(grlibCan_dev_t *dev)
{
	volatile grlibCan_hwDev_t *device = dev->device;

	mutexLock(dev->ctrlLock);

	/* Reset device and turn off codec */
	device->ctrlReg = (device->ctrlReg & ~GRLIB_CAN_ctrlReg_MASK) |
			((1 << 1) & GRLIB_CAN_ctrlReg_MASK);

	/* Sleeping until reset finished */
	while ((device->ctrlReg & (1u << 1)) != 0) { }

	/* Configure ids to listen to */
	device->confReg = 0;
	device->confReg |= (1 << 2) | (1 << 1) | (1 << 1) | (1 << 6);

	/* Do no compare on any bits - listen to all */
	device->syncMask = (0u);
	device->syncCode = ~(0u);

	/* Set baud-rate for both nominal and data */
	device->nomTimConf.reg = defTimConfNom.reg;
	device->dataTimConf.reg = defTimConfData.reg;

	/* Here also delat compensation is to set up - implementation specific */

	/* Turn off CANopen */
	device->copReg &= ~1;

	/* Configure interrupts */
	/* Reading value of the pending interrupt to clear the register */
	volatile uint32_t pending = device->penIrq;
	(void)pending;

	/* Disable all interrupts */
	device->irqMskSet = GRLIB_CAN_txLoss_irq | GRLIB_CAN_or_IRQ |
			GRLIB_CAN_rxErrCnt_IRQ | GRLIB_CAN_rxMiss_irq;

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

static int __attribute__((used)) grlibCan_applyConfig(grlibCan_dev_t *dev, grlibCan_config_t *config)
{
	grlibCan_hwDev_t *device = dev->device;

	mutexLock(dev->ctrlLock);

	/* Turn off codec */
	device->ctrlReg &= ~(1);

	/* Apply user defined configuration */
	device->confReg = config->conf;

	device->syncMask = config->syncMask;
	device->syncCode = config->syncCode;

	/* Set timing configuration */
	device->nomTimConf.reg = config->nomBdRate;   /* Calculate valid baud rate */
	device->dataTimConf.reg = config->dataBdRate; /* Calculate valid baud rate */
	device->transDelCompReg = config->transDelComp;

	/* Set TX and RX config */
	device->txCtrlReg = config->txCtrlReg;
	device->rxCtrlReg = config->rxCtrlReg;
	device->rxAccCode = config->rxAccCode;
	device->rxAccMask = config->rxAccMask;

	/* Turn codec back on */
	device->ctrlReg |= 1;

	mutexUnlock(dev->ctrlLock);
	return 0;
}

static int grlibCan_transmitSync(grlibCan_dev_t *dev, const grlibCan_msg_t *buffer,
		const uint32_t length)
{
	if (dev->txTransactionStatus == GRLIB_CAN_TRANSACTION_ONGOING)
		return -1;
	uint32_t i = 0;

	(void)mutexLock(dev->txLock);

	while (i < length) {
		/* Check for bus status, if error return how many frames sent */
		if (dev->txBufStatus == TX_ERR) {
			(void)mutexUnlock(dev->txLock);
			return -i;
		}
		/* While there is space in transmit buffer push frames */
		while (dev->device->txWrtPtr != (dev->device->txRdPtr > 1 ? dev->device->txRdPtr - 1 : dev->device->txBufSz - 1)) {
			if (i == length) {
				(void)mutexUnlock(dev->txLock);
				return length;
			}
			(void)grlibCan_pushFrame(dev, &buffer[i]);
			i++;
		}

		/* TX buffer has been filled */
		dev->txBufStatus = TX_FULL;
		dev->device->irqMskSet |= GRLIB_CAN_tx_IRQ;

		while (dev->txBufStatus == TX_FULL) {
			/* Wait for interrupt to clear TX_FULL */
			int cRet = condWait(dev->cond, dev->txLock, 10000);
		}

		dev->device->irqMskSet &= ~GRLIB_CAN_tx_IRQ;
	}

	(void)mutexUnlock(dev->txLock);
	return i;
}

/* We could also return instantly with error if there is an ongoing transaction */
/* Check if buffer is 1kB alligned? */
/* Since buffer has to be prepared eariler by the driver from user request we mmap buffer */
static int __attribute__((used)) grlibCan_transmitAsync(grlibCan_dev_t *dev, const grlibCan_msg_t *buffer,
		const uint32_t length)
{
	grlibCan_hwDev_t *device = dev->device;

	(void)mutexLock(dev->txLock);

	/* Wait until controller clears TX buffer */
	if (device->txRdPtr != device->txWrtPtr) {
		dev->txBufStatus = TX_NOT_EMPTY;
		device->irqMskSet |= GRLIB_CAN_txEmpty_IRQ;
		while (dev->txBufStatus == TX_NOT_EMPTY) {
			(void)condWait(dev->cond, dev->txLock, 10000);
		}
	}

	device->irqMskSet &= ~GRLIB_CAN_txEmpty_IRQ;

	/* Turn off TX channel */
	device->txCtrlReg &= 0;
	/* Configure TX channel - map user provided buffer */
	device->txBufAdd = (uint32_t)va2pa((void *)buffer);
	/* Make sure all the data in the buffer is mapped */
	device->txBufSz = (uint32_t)((length + 4 / 4) << 6);
	device->txWrtPtr = length - 1;
	device->txRdPtr = 0;

	device->irqMskSet |= GRLIB_CAN_txPtr_IRQ;
	device->txIrqReg = length - 1;

	device->txCtrlReg |= 1;

	dev->txTransactionStatus = GRLIB_CAN_TRANSACTION_ONGOING;
	(void)mutexUnlock(dev->txLock);

	return 0;
}

static int grlibCan_recvSync(grlibCan_dev_t *dev, grlibCan_msg_t *buffer,
		uint32_t length)
{
	uint32_t i = 0;
	(void)mutexLock(dev->rxLock);

	while (i < length) {
		/* Check for bus status, if error return how many frames read */
		if (dev->rxBufStatus == RX_ERR) {
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
				if ((frame_len - 8) / sizeof(grlibCan_msg_t) >= length - i) {
					(void)mutexUnlock(dev->rxLock);
					return i;
				}
				else {
					/* Read whole packet and continue*/
					for (int j = 0; j < (frame_len - 8) / sizeof(grlibCan_msg_t); j++) {
						(void)grlibCan_popFrame(dev, &buffer[i]);
						i++;
						continue;
					}
				}
			}
			/* Read one packet */
			(void)grlibCan_popFrame(dev, &buffer[i]);
			dev->txErr = i;
			i++;
		}

		/* RX buffer is empty */
		dev->rxBufStatus = RX_EMPTY;
		dev->device->irqMskSet |= GRLIB_CAN_rx_IRQ;

		while (dev->rxBufStatus == RX_EMPTY) {
			if(dev->device->rxRdPtr != dev->device->rxWrtPtr) break;
			if(i == length) break;
			int cRet = condWait(dev->cond, dev->rxLock, 100000);
		}

		dev->device->irqMskSet &= ~(GRLIB_CAN_rx_IRQ);
	}

	(void)mutexUnlock(dev->rxLock);
	return i;
}

static int grlibCan_recvAsync(grlibCan_dev_t *dev, grlibCan_msg_t *buffer,
		const uint32_t length)
{
	uint32_t i = 0;

	(void)mutexLock(dev->rxLock);

	if (dev->rxBufStatus == RX_ERR) {
		(void)mutexUnlock(dev->rxLock);
		return -i;
	}
	/* While there are frames to read, process them */
	while (dev->device->rxRdPtr != dev->device->rxWrtPtr) {
		/* Check if whole CAN frame can be processed */
		uint32_t frame_len = ((dev->rxBufAdd[dev->device->rxRdPtr]).frame.stat >> 28);
		if (frame_len > 8) {
			/* Whole frame cannot be consumed */
			if ((frame_len - 8) / sizeof(grlibCan_msg_t) >= length - i) {
				(void)mutexUnlock(dev->rxLock);
				return i;
			}
			else {
				/* Read whole packet and continue*/
				for (int j = 0; j < (frame_len - 8) / sizeof(grlibCan_msg_t); j++) {
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

static int grlibCan_allocateBuffers(grlibCan_dev_t *dev, uint32_t bufLen)
{
	/* Circular buffer has to be of size that is multiple of 4*/
	if (bufLen % 4 != 0 || bufLen < 4) {
		return -EINVAL;
	}

	dev->txBufAdd = mmap(NULL, _PAGE_SIZE,
			PROT_WRITE | PROT_READ, MAP_UNCACHED | MAP_ANONYMOUS, -1, 0);
	if (dev->txBufAdd == MAP_FAILED) {
		return -ENOMEM;
	}

	dev->txBufSz = (_PAGE_SIZE / (sizeof(grlibCan_msg_t) * 4)) * 4;

	dev->rxBufAdd = mmap(NULL, _PAGE_SIZE,
			PROT_WRITE | PROT_READ, MAP_UNCACHED | MAP_ANONYMOUS, -1, 0);
	if (dev->rxBufAdd == MAP_FAILED) {
		(void)munmap((void *)dev->txBufAdd, sizeof(grlibCan_msg_t) * dev->txBufSz);
		dev->txBufSz = 0;
		return -ENOMEM;
	}

	dev->rxBufSz = (_PAGE_SIZE / (sizeof(grlibCan_msg_t) * 4)) * 4;
	return 0;
}

static int grlibCan_allocateResources(grlibCan_dev_t *dev, uint32_t base, int id)
{
	dev->canId = id;
	dev->device = MAP_FAILED;
	dev->ctrlLock = (handle_t)-1;
	dev->cond = (handle_t)-1;
	dev->sent = 0;
	dev->recv = 0;
	dev->counter = 0;
	dev->txBufAdd = MAP_FAILED;
	dev->rxBufAdd = MAP_FAILED;
	dev->txBufSz = 0;
	dev->rxBufSz = 0;
	dev->rxErrCntr = 0;
	dev->rxMiss = 0;
	dev->overRun = 0;

#ifdef VERBOSE
	debug("grlib-can: Initialized device struct\n");
#endif
	uintptr_t offset = (base & ~(_PAGE_SIZE - 1));
	dev->device = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)offset);

	if (dev->device == MAP_FAILED) {
		debug("grlib-can: Failed to map device physicall address\n");
		return -1;
	}

	if (grlibCan_allocateBuffers(dev, GRLIB_CAN_DEF_CIRCBUFSZ) < 0) {
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

static void grlibCan_cleanupResources(grlibCan_dev_t *dev)
{
	if (dev->device != MAP_FAILED) {
		(void)munmap((void *)dev->device, _PAGE_SIZE);
	}

	if (dev->txBufAdd != MAP_FAILED) {
		(void)munmap((void *)dev->txBufAdd, sizeof(grlibCan_msg_t) * dev->txBufSz);
	}

	if (dev->rxBufAdd != MAP_FAILED) {
		(void)munmap((void *)dev->rxBufAdd, sizeof(grlibCan_msg_t) * dev->rxBufSz);
	}

	if (dev->ctrlLock != (handle_t)-1) {
		resourceDestroy(dev->ctrlLock);
	}
}

static int grlibCan_initDevices(grlibCan_dev_t *devices, int num, bool loopback)
{
	(void)loopback;

	if (grlibCan_allocateResources(&devices[0], GRLIB_CAN0_BASE_ADD, 0) < 0) {
		grlibCan_cleanupResources(&devices[0]);
		return -1;
	}

	if (interrupt(GRLIB_CAN0_IRQ, grlibCan_irqHandler,
				(void *)&devices[0], devices[0].cond, NULL) < 0) {
		debug("grlib-can: Failed to set up the interrupt\n");
	}

#ifdef VERBOSE
	debug("grlib-can: Managed to register callback to device interrupt\n");
#endif

	/* Now having memory for all needed operations we can configure the device */
	grlibCan_applyDefConf(&devices[0]);

#ifdef VERBOSE
	debug("grlib-can: Applied default config to the device\n");
#endif

	return 0;
}

static int grlibCan_registerDevices(oid_t *port, grlibCan_dev_t *devices, uint32_t length)
{
	oid_t dir;

	while (lookup("/dev", NULL, &dir) < 0) {
		usleep(100000);
	}

	for (int i = 0; i < length; i++) {
		char buf[8];
		if (snprintf(buf, sizeof(buf), "can%d", i) >= sizeof(buf)) {
			debug("grlib-can: Failed to create devices\n");
			return -1;
		}
		port->id = i;
		if (create_dev(port, buf)) {
			debug("grlib-can: Failed to create devices\n");
			return -1;
		}
	}

	return 0;
}

int main(int argc, char **argv)
{
	debug("grlib-can: Started driver process\n");

	oid_t port;
	grlibCan_dev_t devices[GRLIB_CAN_NUM];

	grlibCan_driver_t driverInstance = {
		.devices = devices,
		.port = *((uint32_t *)&port)
	};

	/* Create port for driver server*/
	if (portCreate((uint32_t *)&port) < 0) {
		debug("grlib-can: Failed to create port\n");
		return EXIT_FAILURE;
	}

	/* Initialize CAN devices */
	if (grlibCan_initDevices(devices, GRLIB_CAN_NUM, false) < 0) {
		/* IDK - compiler wants uint32_t here, and casting not working */
		portDestroy(*((uint32_t *)(&port)));
		debug("grlib-can: Failed to initialize CAN devices\n");
		return EXIT_FAILURE;
	}

	/* Register device and start thread */
	if (grlibCan_registerDevices(&port, devices, 1) < 0) {
		portDestroy(*((uint32_t *)(&port)));
		grlibCan_cleanupResources(devices);
		debug("grlib-can: Failed to register devices\n");
		return EXIT_FAILURE;
	}

	// grlibCan_messageThread(&driverInstance);
	debug_routine(devices);

	grlibCan_cleanupResources(devices);
	portDestroy(driverInstance.port);

	return 0;
}
