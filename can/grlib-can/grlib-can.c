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
#include <sys/mman.h>

/* Local includes */
#include "grlib-can.h"

#pragma GCC optimize("Og")

static void __attribute__((used)) debug_routine(void *args)
{
	grlibCan_dev_t *dev = (grlibCan_dev_t *)args;

	{
		grlibCan_config_t cfg;
		(void)memset((void *)&cfg, 0xFF, sizeof(grlibCan_config_t));

		grlibCan_applyConfig(dev, &cfg);
	}

	return;

	grlibCan_msg_t frame;
	frame.frame.head = ~(0u);
	frame.frame.stat = 0x10u << 24u;
	frame.frame.payload[0] = 0xFAu;

	{
		char buf[250];

		snprintf(buf, 250, "Header: %x, Stat: %x, Payload: %x\n",
				frame.frame.head, frame.frame.stat, *frame.frame.payload);
		debug(buf);

		snprintf(buf, 250, "%x : %x : %lx", dev->device->rxBufAdd, (uint32_t)va2pa((void *)dev->rxBufAdd), (uint64_t)(void *)(dev->rxBufAdd));
		debug(buf);
	}

	// grlibCan_msg_t cpy;

	for (int k = 0; k < 4; k++) {
		for (int i = 0; i < GRLIB_CAN_DEF_CIRCBUFSZ - 1; i++) {
			frame.frame.head = (i << 18);
			grlibCan_pushFrame(dev, &frame, false);
			usleep(250000);

			char buf[250];

			snprintf(buf, 250, "Send: %x, Recv: %x\n", dev->sent, dev->recv);
			debug(buf);
		}

		// for (int i = 0; i < GRLIB_CAN_DEF_CIRCBUFSZ - 1; i++) {
		// 	grlibCan_popFrame(dev, &cpy, false);
		// 	usleep(250000);
		// }
	}

	{
		char buf[100];
		snprintf(buf, 100, "Send: %x, Recv: %x, Counter: %x\n", dev->sent, dev->recv, dev->counter);
		debug(buf);
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
	volatile uint32_t pending = device->penIrq;

	if ((pending & (1 << 9)) != 0) {
		dev->recv++;
	}

	if ((pending & (1 << 10)) != 0) {
		dev->sent++;
	}

	if ((pending & (1 << 15)) != 0) {
		dev->counter++;
	}

	return 0;
}

static int grlibCan_popFrame(grlibCan_dev_t *dev, grlibCan_msg_t *msg, bool async)
{
	uint32_t rdPtr = (dev->device->rxRdPtr) >> 4;
	(void)memcpy((void *)msg, (void *)(&dev->rxBufAdd[rdPtr]), sizeof(grlibCan_msg_t));
	dev->device->rxRdPtr = (((rdPtr + 1) % dev->rxBufSz) << 4);
	return 0;
}

static int grlibCan_pushFrame(grlibCan_dev_t *dev, const grlibCan_msg_t *msg, bool block)
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
	device->confReg |= (1 << 2) | (1 << 1) | (1 << 1) | (1 << 6);
	device->confReg &= ~((1 << 7) | (1 << 3) | 1);
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

	/* Enable all interrupts */
	device->irqMskSet = (device->irqMskSet & ~GRLIB_CAN_irqReg_MASK) |
			(((~(uint32_t)0) & GRLIB_CAN_irqReg_MASK));

	/* Configure TX channel */
	device->txBufAdd = (uint32_t)va2pa((void *)dev->txBufAdd);
	device->txBufSz = (uint32_t)((dev->txBufSz / 4) << 6);
	device->txWrtPtr = 0;
	device->txRdPtr = 0;
	device->txCtrlReg |= 1;

	/* Configure RX channel */
	device->rxBufAdd = (uint32_t)va2pa((void *)dev->rxBufAdd);
	device->rxBufSz = (dev->rxBufSz / 4) << 6;
	device->rxWrtPtr = 0;
	device->rxRdPtr = 0;
	/* Put all frames that we got from the bus into the buffer */
	device->rxAccCode = (0u);
	device->rxAccMask = (0u);
	device->rxCtrlReg |= 1 | (1 << 3);

	/* Turn codec back on */
	device->ctrlReg |= 1;

	mutexUnlock(dev->ctrlLock);
	return 0;
}

static void grlibCan_enableSelfLb(grlibCan_dev_t *dev)
{
	dev->device->confReg = dev->device->confReg | (1 << 7) | (1 << 6);
}

static int grlibCan_applyConfig(grlibCan_dev_t *dev, grlibCan_config_t *config)
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


static int grlibCan_allocateBuffers(grlibCan_dev_t *dev, uint32_t bufLen)
{
	/* Circular buffer has to be of size that is multiple of 4*/
	if (bufLen % 4 != 0 || bufLen < 4) {
		return -EINVAL;
	}

	dev->txBufAdd = mmap(NULL, sizeof(grlibCan_msg_t) * bufLen,
			PROT_WRITE | PROT_READ, MAP_UNCACHED | MAP_ANONYMOUS, -1, 0);
	if (dev->txBufAdd == MAP_FAILED) {
		return -ENOMEM;
	}

	dev->txBufSz = bufLen;

	dev->rxBufAdd = mmap(NULL, sizeof(grlibCan_msg_t) * bufLen,
			PROT_WRITE | PROT_READ, MAP_UNCACHED | MAP_ANONYMOUS, -1, 0);
	if (dev->rxBufAdd == MAP_FAILED) {
		(void)munmap((void *)dev->txBufAdd, sizeof(grlibCan_msg_t) * dev->txBufSz);
		dev->txBufSz = 0;
		return -ENOMEM;
	}

	dev->rxBufSz = bufLen;
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
	dev->txBufAdd = MAP_FAILED;
	dev->rxBufAdd = MAP_FAILED;
	dev->txBufSz = 0;
	dev->rxBufSz = 0;

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
