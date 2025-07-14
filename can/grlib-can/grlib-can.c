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
#include <sys/time.h>
#include <sys/mman.h>
#include <posix/utils.h>

/* Platform specific incldues */
#include <sys/platform.h>
#include <phoenix/gaisler/ambapp.h>
#include <phoenix/arch/riscv64/riscv64.h>

/* Local includes */
#include "grlib-can.h"

#pragma GCC optimize("O0")

void sendingThread(void *args)
{
	grlibCan_dev_t *dev = (grlibCan_dev_t *)args;
	grlibCan_msg_t frame[GRLIB_CAN_DEF_CIRCBUFSZ * 10];

	frame->frame.head = (1 << 18);
	frame->frame.stat = 0x10u << 24u;
	frame->frame.payload[0] = 0x0F;

	for (int i = 1; i < GRLIB_CAN_DEF_CIRCBUFSZ * 10; i++) {
		memcpy((void *)&frame[i], (void *)frame, sizeof(grlibCan_msg_t));
		frame[i].frame.payload[0] = 0x0F + i;
	}

	printf("Sending messages\n");

	grlibCan_transmitSync(dev, frame, GRLIB_CAN_DEF_CIRCBUFSZ * 10);

	grlibCan_transmitAsync(dev, frame, GRLIB_CAN_DEF_CIRCBUFSZ * 10);

	printf("Bursting finished\n");
	endthread();
}

uint8_t stack[4096];

static void __attribute__((used)) debug_routine(void *args, int num)
{
	debug("Started debug routine\n");
	usleep(1000000);

	grlibCan_dev_t *dev = (grlibCan_dev_t *)args;
	grlibCan_msg_t frames[GRLIB_CAN_DEF_CIRCBUFSZ * 20];

	for (int i = 0; i < num; i++) {
		printf("Testing CAN%d\n", i);
		beginthread(sendingThread, 4, stack, 4096, &dev[i]);

		debug("Receiving frames\n");
		usleep(20000);
		int recved = grlibCan_recvAsync(&dev[i], frames, 160);

		printf("Received: %d\n", recved);
		debug("Transmission successful\n");
	}

	for (;;) {
	}
}

static int grlibCan_handleDevCtl(msg_t *msg, grlibCan_dev_t *device)
{
	grlibCan_devCtrl_t *idevctl = (grlibCan_devCtrl_t *)(msg->i.raw);
	int ret;

	switch (idevctl->type) {
		case can_config:
			if (msg->i.size < sizeof(grlibCan_config_t)) {
				msg->o.err = -1;
				return -1;
			}
			return grlibCan_applyConfig(device, (grlibCan_config_t *)msg->i.data);
		case can_getStatus:
			if (msg->i.size < sizeof(uint32_t)) {
				msg->o.err = -1;
				return -1;
			}
			msg->o.err = 0;
			*(uint32_t *)(msg->o.data) = device->device->statReg;
			msg->o.size = sizeof(uint32_t);
			return 0;
		case can_writeSync:
			ret = grlibCan_transmitSync(device, (grlibCan_msg_t *)msg->i.data, (uint32_t)msg->i.size);
			msg->o.err = ret;
			return ret;
		case can_readSync:
			ret = grlibCan_recvSync(device, (grlibCan_msg_t *)msg->o.data, (uint32_t)msg->o.size);
			msg->o.err = ret;
			return ret;
		case can_writeAsync:
			ret = grlibCan_transmitAsync(device, (grlibCan_msg_t *)msg->i.data, (uint32_t)msg->i.size);
			msg->o.err = ret;
			return ret;
		case can_readAsync:
			ret = grlibCan_recvAsync(device, (grlibCan_msg_t *)msg->o.data, (uint32_t)msg->o.size);
			msg->o.err = ret;
			return ret;
		default:
			return -1;
	}
}

static int grlibCan_dispatchMsg(msg_t *msg, grlibCan_dev_t *devices, int num)
{
	id_t id = msg->oid.id;
	if (id < num && devices[id].canId != -1) {
		return grlibCan_handleDevCtl(msg, &devices[id]);
	}
	return -1;
}

static void __attribute__((used)) grlibCan_messageThread(void *args)
{
	grlibCan_driver_t *driverInstance = (grlibCan_driver_t *)args;
	grlibCan_dev_t *devices = driverInstance->devices;
	int num = driverInstance->num;

	msg_t msg;
	msg_rid_t rid;

	for (;;) {
		while (msgRecv(driverInstance->port, &msg, &rid) < 0) { }
		switch (msg.type) {
			case mtDevCtl:
				if (msg.oid.id < num && devices[msg.oid.id].canId != -1 && devices[msg.oid.id].ownerRid == rid) {
					grlibCan_dispatchMsg(&msg, devices, num);
				}
				else {
					msg.o.err = -EPERM;
				}
				break;
			case mtOpen:
				if (msg.oid.id < num && devices[msg.oid.id].canId != -1 && devices[msg.oid.id].ownerRid == 0) {
					devices[msg.oid.id].ownerRid = rid;
					msg.o.err = EOK;
				}
				else {
					msg.o.err = -EBUSY;
				}
				break;
			case mtClose:
				if (msg.oid.id < num && devices[msg.oid.id].canId != -1 && devices[msg.oid.id].ownerRid == rid) {
					devices[msg.oid.id].ownerRid = 0;
				}
				else {
					msg.o.err = -EPERM;
				}
				break;
			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(driverInstance->port, &msg, rid);
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
			if (i == length)
				break;
			/* Wait for interrupt to clear TX_FULL */
			(void)condWait(dev->cond, dev->txLock, 1000000);
		}

		dev->device->irqMskSet &= ~GRLIB_CAN_tx_IRQ;
	}

	(void)mutexUnlock(dev->txLock);
	return i;
}

static int grlibCan_transmitAsync(grlibCan_dev_t *dev, const grlibCan_msg_t *buffer,
		const uint32_t length)
{
	uint32_t i = 0;

	(void)mutexLock(dev->txLock);

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

	(void)mutexUnlock(dev->txLock);
	return i;
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
			// if (dev->device->rxRdPtr != dev->device->rxWrtPtr)
			// 	break;
			if (i == length)
				break;
			(void)condWait(dev->cond, dev->rxLock, 100000);
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
		return -0xFF;
	}
	/* While there are frames to read, process them */
	while (dev->device->rxRdPtr != dev->device->rxWrtPtr && i < length) {
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
		i++;
	}

	(void)mutexUnlock(dev->rxLock);
	return i;
}

static int grlibCan_allocateBuffers(grlibCan_dev_t *dev)
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
		(void)munmap((void *)dev->txBufAdd, sizeof(grlibCan_msg_t) * dev->txBufSz);
		dev->txBufSz = 0;
		return -ENOMEM;
	}

	dev->rxBufSz = (_PAGE_SIZE / (sizeof(grlibCan_msg_t) * 4)) * 4;
	return 0;
}

static int grlibCan_allocateResources(grlibCan_dev_t *dev, uintptr_t base, int id)
{
	dev->ownerRid = 0;
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

static int grlibCan_registerDevices(oid_t *oid, grlibCan_dev_t *devices, uint32_t length)
{
	oid_t dir;

	while (lookup("/dev", &dir, NULL) < 0) {
		usleep(100000);
	}

	for (int i = 0; i < length; i++) {
		char buf[8];
		if (snprintf(buf, sizeof(buf), "can%d", i) >= sizeof(buf)) {
			debug("grlib-can: Failed to create devices\n");
			return -1;
		}
		oid->id = i;

		// if (create_dev(oid, buf)) {
		// 	debug("grlib-can: Failed to create devices\n");
		// 	return -1;
		// }

		msg_t msg;

		msg.type = mtCreate;
		msg.oid = dir;
		msg.i.create.type = otDev;
		msg.i.create.mode = 0;
		msg.i.create.dev.port = oid->port; /* Port number assigned by portCreate */
		msg.i.create.dev.id = i;           /* Id assigned by the driver itself */
		msg.i.data = buf;                  /* Filename */
		msg.i.size = strlen(msg.i.data);
		msg.o.data = NULL;
		msg.o.size = 0;

		msgSend(dir.port, &msg);
	}

	return 0;
}

static int grlibCan_unregisterDevices(oid_t *port, grlibCan_dev_t *devices, uint32_t length)
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

int main(int argc, char **argv)
{
	debug("grlib-can: Started driver process\n");

	int detectedDevices = 0;
	grlibCan_dev_t devices[GRLIB_MAX_CAN_DEVICES];

	/* System query for number of GRCAN controllers and their properites */
	for (unsigned int i = 0; i < GRLIB_MAX_CAN_DEVICES; i++) {
		unsigned int id = i;

		ambapp_dev_t dev = { .devId = CORE_ID_GRCANFD };
		platformctl_t ctl = {
			.action = pctl_get,
			.type = pctl_ambapp,
			.task.ambapp.dev = &dev,
			.task.ambapp.instance = &id
		};

		if (platformctl(&ctl) < 0)
			break;

		detectedDevices++;

		if (dev.bus == BUS_AMBA_AHB) {
			devices[i].canId = -1;
			devices[i].irqNum = 0;
			devices[i].device = 0;
			/* GRCANFD should be on APB bus */
			continue;
		}
		else if (dev.bus == BUS_AMBA_APB) {
			devices[i].canId = dev.devId;
			devices[i].irqNum = dev.irqn;
			devices[i].device = (grlibCan_hwDev_t *)dev.info.apb.base;
		}
	}

	oid_t oid;

	/* Create port for driver server*/
	if (portCreate(&oid.port) < 0) {
		debug("grlib-can: Failed to create port\n");
		return EXIT_FAILURE;
	}

	grlibCan_driver_t driverInstance = {
		.devices = devices,
		.port = oid.port,
		.num = detectedDevices
	};

	/* Initialize CAN devices */
	if (grlibCan_initDevices(driverInstance.devices, driverInstance.num, false) < 0) {
		/* IDK - compiler wants uint32_t here, and casting not working */
		portDestroy(oid.port);
		debug("grlib-can: Failed to initialize CAN devices\n");
		return EXIT_FAILURE;
	}

	/* Register device and start thread */
	if (grlibCan_registerDevices(&oid, driverInstance.devices, driverInstance.num) < 0) {
		portDestroy(oid.port);
		for (int i = 0; i < driverInstance.num; i++)
			grlibCan_cleanupResources(&devices[i]);
		debug("grlib-can: Failed to register devices\n");
		return EXIT_FAILURE;
	}

	grlibCan_messageThread(&driverInstance);
	// debug_routine(driverInstance.devices, driverInstance.num);

	grlibCan_cleanupResources(driverInstance.devices);
	portDestroy(driverInstance.port);
	grlibCan_unregisterDevices(&oid, driverInstance.devices, driverInstance.num);

	return 0;
}
