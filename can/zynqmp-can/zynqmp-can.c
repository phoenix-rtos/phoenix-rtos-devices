/*
 * Phoenix-RTOS
 *
 * ZynqMP CAN driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Dariusz Sabala
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <endian.h>
#include <sys/interrupt.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/platform.h>
#include <sys/threads.h>
#include <posix/utils.h>

#include <phoenix/arch/aarch64/zynqmp/zynqmp.h>

#include <board_config.h>

#include "zynqmp-can-priv.h"
#include "zynqmp-can-if.h"

/**
 * Watermark configuration for "TX FIFO empty elements above watermark" interrupt.
 * Setting this to low values (e.g., 1) reduces the blocking time for the sending function
 * but increases the frequency of interrupts.
 */
#define CAN_TX_FIFO_WATERMARK 4

/* Default CAN baud rate in kbps */
#define CAN_BAUDRATE_KBPS_DEFAULT (1000)

/* Base addresses for ZynqMP CAN peripherals */
#define ZYNQMP_CAN0_BASE (0x00ff060000)
#define ZYNQMP_CAN1_BASE (0x00ff070000)

/* Number of CAN peripheral instances */
#define ZYNQMP_CAN_NUMBER_OF_INSTANCES (2)

/* Structure representing the CAN peripheral registers */
typedef struct {
	volatile uint32_t srr;             /* Software Reset and Enable, Address offset:  0x0 */
	volatile uint32_t msr;             /* Mode Select, Address offset:  0x4 */
	volatile uint32_t brpr;            /* Baud Rate Prescaler, Address offset:  0x8 */
	volatile uint32_t btr;             /* Bit Timing and Synchronization, Address offset:  0xc */
	volatile const uint32_t ecr;       /* Rx and Tx Error Counters, Address offset: 0x10 */
	volatile uint32_t esr;             /* Error Status, Address offset: 0x14 */
	volatile uint32_t sr;              /* Controller Status, Address offset: 0x18 */
	volatile uint32_t isr;             /* Interrupt Status, Address offset: 0x1c */
	volatile uint32_t ier;             /* Interrupt Enable, Address offset: 0x20 */
	volatile uint32_t icr;             /* Interrupt Clear, Address offset: 0x24 */
	volatile uint32_t tcr;             /* Timestamp Clear., Address offset: 0x28 */
	volatile uint32_t wir;             /* Rx and Tx Watermark Settings., Address offset: 0x2c */
	volatile uint32_t txfifo_id;       /* Tx Message FIFO, Identifier, Request., Address offset: 0x30 */
	volatile uint32_t txfifo_dlc;      /* Tx Message FIFO Data Length Code., Address offset: 0x34 */
	volatile uint32_t txfifo_data1;    /* Tx Message FIFO, data word 1., Address offset: 0x38 */
	volatile uint32_t txfifo_data2;    /* Tx Message FIFO, data word 2., Address offset: 0x3c */
	volatile uint32_t txhpb_id;        /* High Priority Tx Message FIFO, Identifier, Request., Address offset: 0x40 */
	volatile uint32_t txhpb_dlc;       /* High Priority Tx Message FIFO Data Length Code., Address offset: 0x44 */
	volatile uint32_t txhpb_data1;     /* High Priority Tx Message FIFO, data word 1., Address offset: 0x48 */
	volatile uint32_t txhpb_data2;     /* High Priority Tx Message FIFO, data word 0., Address offset: 0x4c */
	volatile const uint32_t rxfifo_id; /* Rx Message FIFO, Identifier, Request., Address offset: 0x50 */
	volatile uint32_t rxfifo_dlc;      /* Rx Message FIFO Data Length Code., Address offset: 0x54 */
	volatile uint32_t rxfifo_data1;    /* Rx Message FIFO, data word 1., Address offset: 0x58 */
	volatile uint32_t rxfifo_data2;    /* Rx Message FIFO, data word 2., Address offset: 0x5c */
	volatile uint32_t afr;             /* Acceptance Filter Enables., Address offset: 0x60 */
	volatile uint32_t afmr1;           /* Acceptance Filter 1 Mask., Address offset: 0x64 */
	volatile uint32_t afir1;           /* Acceptance Filter 1 ID., Address offset: 0x68 */
	volatile uint32_t afmr2;           /* Acceptance Filter 2 Mask., Address offset: 0x6c */
	volatile uint32_t afir2;           /* Acceptance Filter 2 ID., Address offset: 0x70 */
	volatile uint32_t afmr3;           /* Acceptance Filter 3 Mask., Address offset: 0x74 */
	volatile uint32_t afir3;           /* Acceptance Filter 3 ID., Address offset: 0x78 */
	volatile uint32_t afmr4;           /* Acceptance Filter 4 Mask., Address offset: 0x7c */
	volatile uint32_t afir4;           /* Acceptance Filter 4 ID., Address offset: 0x80 */
} can_periph_t;

/* Data structure used by the driver */
static struct {
	volatile can_periph_t *base; /**< Pointer to the peripheral base address */
	oid_t oid;                   /**< Object identifier for the related device file */
	handle_t cond;               /**< Conditional variable for synchronizing interrupts */
	handle_t inth;               /**< Interrupt handler object */
	handle_t lock;               /**< Mutex used with the conditional variable for synchronization */
} can;

/* Immutable information about CAN peripheral instances */
static const struct {
	uint32_t base;    /**< Peripheral base address */
	unsigned int irq; /**< IRQ number */
	uint16_t clk;     /**< Identifier of the related clock in CPU clock registers */
	uint16_t rst;     /**< Identifier of the related reset line in the CPU reset peripheral */
	uint16_t rxPin;   /**< CAN RX pin (configured in board_config.h) */
	uint16_t txPin;   /**< CAN TX pin (configured in board_config.h) */
} info[ZYNQMP_CAN_NUMBER_OF_INSTANCES] = {
	{ ZYNQMP_CAN0_BASE, 55, pctl_devclock_lpd_can0, pctl_devreset_lpd_can0, CAN0_RX, CAN0_TX },
	{ ZYNQMP_CAN1_BASE, 56, pctl_devclock_lpd_can1, pctl_devreset_lpd_can1, CAN1_RX, CAN1_TX }
};

/* Popular baud rate configurations */
static const struct {
	uint32_t baudrate_kbps; /**< Baud rate in kbps */
	uint32_t brpr;          /**< Baud Rate Prescaler register value */
	uint32_t btr;           /**< Bit Timing register value */
	char *sample;           /**< Sampling point percentage */
} timeCfg[] = {
	{ .baudrate_kbps = 50, .sample = "87.5", .brpr = 0x7C, .btr = 0x05 },
	{ .baudrate_kbps = 100, .sample = "90.0", .brpr = 0x31, .btr = 0x07 },
	{ .baudrate_kbps = 125, .sample = "87.5", .brpr = 0x31, .btr = 0x05 },
	{ .baudrate_kbps = 250, .sample = "87.5", .brpr = 0x18, .btr = 0x05 },
	{ .baudrate_kbps = 500, .sample = "90.0", .brpr = 0x09, .btr = 0x07 },
	{ .baudrate_kbps = 1000, .sample = "90.0", .brpr = 0x04, .btr = 0x07 },
};

/* Function to display command-line help */
static void zynqmpCan_CliHelp(const char *progname)
{
	printf("Usage: %s [OPTIONS]\n", progname);
	printf("Options:\n");
	printf("\t-n <id>        - CAN controller ID [0...1] \n");
	printf("\t-b <baudrate>  - Baud rate [kbps] supported (sample points in brackets): ");
	for (uint32_t i = 0; i < (sizeof(timeCfg) / sizeof(timeCfg[0])); i++) {
		printf("%u (%s%%), ", timeCfg[i].baudrate_kbps, timeCfg[i].sample);
	}
	printf("\n");
	printf("\t-h             - Print this message\n");
}

/**
 * Initializes the clock for the specified CAN peripheral.
 * Sets the IO_PLL as the clock source and configures the divider.
 */
static int zynqmpCan_initClock(int can_id)
{
	platformctl_t ctl;

	ctl.action = pctl_set;
	ctl.type = pctl_devclock;

	/* Set IO_PLL as source clock and set divider:
	 * IO_PLL / 20 :  1000 MHz / 20 = 50 MHz */
	ctl.devclock.dev = info[can_id].clk;
	ctl.devclock.src = 0;
	ctl.devclock.div0 = 20;
	ctl.devclock.div1 = 0;
	ctl.devclock.active = 0x1;

	return platformctl(&ctl);
}

/**
 * Configures the MIO pin for CAN RX or TX.
 * Sets default properties and enables pull-up resistors.
 */
static int zynqmpCan_configMio(uint32_t pin)
{
	platformctl_t ctl;

	ctl.action = pctl_set;
	ctl.type = pctl_mio;

	/* Set default properties for pins */
	ctl.mio.pin = pin;
	ctl.mio.l0 = ctl.mio.l1 = ctl.mio.l2 = 0;
	ctl.mio.l3 = 0x1;
	ctl.mio.config = PCTL_MIO_SLOW_nFAST | PCTL_MIO_PULL_UP_nDOWN | PCTL_MIO_PULL_ENABLE;

	if ((pin == CAN0_RX) || (pin == CAN1_RX)) {
		ctl.mio.config |= PCTL_MIO_TRI_ENABLE;
	}

	return platformctl(&ctl);
}

/**
 * Resets the specified CAN peripheral using the reset subsystem.
 */
static int zynqmpCan_resetPeriphal(int can_id)
{
	platformctl_t ctl;

	ctl.action = pctl_set;
	ctl.type = pctl_devreset;

	ctl.devreset.dev = info[can_id].rst;
	ctl.devreset.state = 0;

	return platformctl(&ctl);
}

/**
 * CAN interrupt handler.
 * Handles RX FIFO non-empty, TX FIFO empty, and TX FIFO watermark interrupts.
 */
static int zynqmpCan_irqHandler(unsigned int can_id, void *arg)
{
	uint32_t isr = can.base->isr;

	/* RX FIFO Non-empty IRQ occurred */
	if (isr & (1 << 7)) {
		can.base->ier &= ~(1 << 7);
	}

	/* TX FIFO empty IRQ occurred */
	if (isr & (1 << 14)) {
		can.base->ier &= ~(1 << 14);
	}

	/* TX FIFO empty elements above watermark IRQ occurred */
	if (isr & (1 << 13)) {
		can.base->ier &= ~(1 << 13);
	}

	return 1;
}

/**
 * Initializes the CAN peripheral with the specified ID and baud rate.
 * Configures the clock, MIO pins, and synchronization primitives.
 */
static int zynqmpCan_init(unsigned int can_id, int baudrate)
{
	/* Find the matching time configuration */
	bool timeCfgFound = false;
	uint32_t timeCfgN = 0;
	for (uint32_t i = 0; i < (sizeof(timeCfg) / sizeof(timeCfg[0])); i++) {
		if (timeCfg[i].baudrate_kbps == baudrate) {
			timeCfgFound = true;
			timeCfgN = i;
		}
	}
	if (!timeCfgFound) {
		fprintf(stderr, "zynqmp-can: failed to configure desired baud rate\n");
		return -1;
	}

	/* Initialize the clock */
	if (zynqmpCan_initClock(can_id) < 0) {
		fprintf(stderr, "zynqmp-can: cannot initialize clocks\n");
		return -1;
	}

	/* Map access to the peripheral */
	can.base = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, info[can_id].base);
	if (can.base == MAP_FAILED) {
		fprintf(stderr, "zynqmp-can: failed to map peripheral\n");
		return -1;
	}

	/* Initialize synchronization primitives */
	if (condCreate(&can.cond) != 0) {
		munmap((void *)can.base, _PAGE_SIZE);
		fprintf(stderr, "zynqmp-can: failed to create conditional variable\n");
		return -1;
	}
	if (mutexCreate(&can.lock) != 0) {
		fprintf(stderr, "zynqmp-can: failed to create mutex\n");
		munmap((void *)can.base, _PAGE_SIZE);
		resourceDestroy(can.cond);
		return -1;
	}

	/* Reset the CAN peripheral */
	if (zynqmpCan_resetPeriphal(can_id) < 0) {
		fprintf(stderr, "zynqmp-can: failed to reset CAN peripheral\n");
		munmap((void *)can.base, _PAGE_SIZE);
		resourceDestroy(can.cond);
		resourceDestroy(can.lock);
		return -1;
	}

	/* Configure MIO pins */
	if (zynqmpCan_configMio(info[can_id].rxPin) < 0 || zynqmpCan_configMio(info[can_id].txPin) < 0) {
		fprintf(stderr, "zynqmp-can: failed to initialize MIO pins\n");
		munmap((void *)can.base, _PAGE_SIZE);
		resourceDestroy(can.cond);
		resourceDestroy(can.lock);
		return -1;
	}

	/* Wait for configuration mode */
	while ((can.base->sr & (1 << 0)) != 1) {
		usleep(100);
	}

	/* Disable and clear all interrupts */
	can.base->ier = 0;
	can.base->icr = 0xffffffff;

	/* Program the bit sampling clock */
	can.base->brpr = timeCfg[timeCfgN].brpr;
	can.base->btr = timeCfg[timeCfgN].btr;

	/* Configure the watermark for TX FIFO */
	can.base->wir = (CAN_TX_FIFO_WATERMARK << 8);

	/* Register the interrupt handler */
	if (interrupt(info[can_id].irq, zynqmpCan_irqHandler, &can, can.cond, &can.inth) != 0) {
		fprintf(stderr, "zynqmp-can: failed to register interrupt handler\n");
		munmap((void *)can.base, _PAGE_SIZE);
		resourceDestroy(can.cond);
		resourceDestroy(can.lock);
		return -1;
	}

	/* Enable the controller */
	can.base->srr = (1 << 1);

	printf("zynqmp-can: Initialized CAN%u with baud rate: %u kbps, sample point: %s%%\n",
			can_id,
			timeCfg[timeCfgN].baudrate_kbps,
			timeCfg[timeCfgN].sample);

	return 0;
}

/**
 * Sends CAN frames to the hardware TX FIFO.
 * Blocks if the FIFO is full and blocking is enabled.
 */
static int zynqmpCan_framesSent(zynqmp_canFrame *frames, uint32_t count, bool block, uint32_t *framesSent)
{
	/* Validate arguments */
	if (frames == NULL) {
		return -EINVAL;
	}
	if ((count == 0) || (count > 64)) {
		return -EINVAL;
	}

	*framesSent = 0;

	/* Flush all frames to the hardware TX FIFO */
	for (uint32_t i = 0; i < count; i++) {
		/* Check if TX FIFO is not full */
		if ((can.base->sr & (1 << 10)) != 0) {
			if (block) {
				/* Wait for empty space in TX FIFO */
				while ((can.base->sr & (1 << 10)) != 0) {
					/* Clear and enable "Transmit FIFO Watermark Empty" interrupt */
					can.base->icr = (1 << 13);
					can.base->ier |= (1 << 13);
					/* Wait for this interrupt to trigger */
					condWait(can.cond, can.lock, 0);
				}
			}
			else {
				/* Not enough space in the buffer */
				return -EAGAIN;
			}
		}

		/* Write a single CAN frame into the hardware TX FIFO */
		can.base->txfifo_id = (frames[i].id << 21);
		can.base->txfifo_dlc = (frames[i].len << 28);
		can.base->txfifo_data1 = be32toh(frames[i].payload.words[0]);
		can.base->txfifo_data2 = be32toh(frames[i].payload.words[1]);
		(*framesSent)++;
	}

	return 0;
}

/**
 * Receives CAN frames from the hardware RX FIFO.
 * Blocks if the FIFO is empty and a timeout is specified.
 */
static int zynqmpCan_recvFrames(zynqmp_canFrame *buf, uint32_t bufLen, bool block, uint32_t timeoutUs, uint32_t *recvFrames)
{
	/* Validate arguments */
	if ((buf == NULL) || (recvFrames == NULL)) {
		return -EINVAL;
	}
	if (bufLen == 0) {
		return -EINVAL;
	}

	*recvFrames = 0;

	/* Try clearing the interrupt (it will only clear if there is no data in FIFO) */
	can.base->icr = (1 << 7);

	/* If there is no data in RX FIFO then block if its desired */
	if (((can.base->isr & (1 << 7)) == 0) && (block)) {
		/* Wait for any data in RX FIFO */
		while ((can.base->isr & (1 << 7)) == 0) {
			/* Clear and enable "Receive FIFO Not Empty" interrupt */
			can.base->icr = (1 << 7);
			can.base->ier |= (1 << 7);
			/* Wait for single "Receive FIFO Not Empty" IRQ */
			int condRet = condWait(can.cond, can.lock, timeoutUs);
			if (condRet == -ETIME) {
				/* Timeout, disable "Receive FIFO Not Empty" interrupt */
				can.base->ier &= ~(1 << 7);
				return condRet;
			}
			else if (condRet == 0) {
				/* Start reading frames */
			}
			else {
				/* Unknown error, disable "Receive FIFO Not Empty" interrupt */
				can.base->ier &= ~(1 << 7);
				fprintf(stderr, "zynqmp-can: unknown condWait error %i\n", condRet);
				return condRet;
			}
		}
	}

	/* Read all received frames */
	for (uint32_t i = 0; i < bufLen; i++) {
		/* Try clearing the interrupt (it will only clear if there is no data) */
		can.base->icr = (1 << 7);

		/* Check if there is still data in the RX FIFO (by reading pending IRQ status) */
		if ((can.base->isr & (1 << 7)) == 0) {
			break;
		}

		/* Read a frame from the FIFO */
		buf[i].id = (can.base->rxfifo_id >> 21) & 0x7ff;
		buf[i].len = (can.base->rxfifo_dlc >> 28) & 0xf;
		buf[i].payload.words[0] = be32toh(can.base->rxfifo_data1);
		buf[i].payload.words[1] = be32toh(can.base->rxfifo_data2);
		(*recvFrames)++;
	}

	return 0;
}

/**
 * Main thread for handling CAN operations.
 * Processes incoming messages and performs the requested operations.
 */
static void zynqmpCan_thread(void *arg)
{
	msg_t msg;
	msg_rid_t rid;
	uint32_t port = can.oid.port;

	mutexLock(can.lock);

	for (;;) {
		/* Wait indefinitely for a message */
		if (msgRecv(port, &msg, &rid) < 0) {
			continue;
		}
		switch (msg.type) {
			/* This driver supports only one way of communication with it */
			case mtDevCtl: {
				/* Decode extra information passed with the request */
				zynqmp_canDriverReq *req = (zynqmp_canDriverReq *)msg.i.raw;
				zynqmp_canDriverResp *resp = (zynqmp_canDriverResp *)msg.o.raw;

				/* Choose operation based on opcode */
				if (req->operation == zynqmp_canOpSend) {
					zynqmp_canFrame *frames = (zynqmp_canFrame *)msg.i.data;
					uint32_t count = msg.i.size / sizeof(zynqmp_canFrame);
					msg.o.err = zynqmpCan_framesSent(frames, count, req->block, &resp->framesReceivedSend);
				}
				else if (req->operation == zynqmp_canOpRecv) {
					zynqmp_canFrame *buf = (zynqmp_canFrame *)msg.o.data;
					uint32_t bufLen = msg.o.size / sizeof(zynqmp_canFrame);
					msg.o.err = zynqmpCan_recvFrames(buf, bufLen, req->block, req->timeoutUs, &resp->framesReceivedSend);
				}
				else {
					msg.o.err = -EINVAL;
				}
			} break;

			case mtOpen:
			case mtClose:
				msg.o.err = 0;
				break;

			case mtWrite:
			case mtRead:
			case mtGetAttr:
			default: {
				msg.o.err = -ENOSYS;
				break;
			}
		}
		msgRespond(port, &msg, rid);
	}

	mutexUnlock(can.lock);
}

/**
 * Main entry point for the CAN driver.
 * Parses command-line arguments, initializes the CAN peripheral, and starts the message handling thread.
 */
int main(int argc, char **argv)
{
	int c = 0;

	/* Default arguments */
	int can_id = 0;
	int baudrate = CAN_BAUDRATE_KBPS_DEFAULT;

	/* Parse command-line arguments */
	if (argc > 1) {
		while ((c = getopt(argc, argv, "n:b:h")) != -1) {
			switch (c) {
				case 'b': {
					baudrate = atoi(optarg);
				} break;

				case 'n': {
					can_id = atoi(optarg);
					if ((can_id < 0) || (can_id > 1)) {
						fprintf(stderr, "zynqmp-can: invalid CAN peripheral ID\n");
						return EXIT_FAILURE;
					}
				} break;

				case 'h':
					zynqmpCan_CliHelp(argv[0]);
					return EXIT_SUCCESS;

				default:
					zynqmpCan_CliHelp(argv[0]);
					return EXIT_FAILURE;
			}
		}
	}

	/* Initialize the CAN peripheral */
	if (zynqmpCan_init(can_id, baudrate) < 0) {
		fprintf(stderr, "zynqmp-can: cannot initialize CAN\n");
		return EXIT_FAILURE;
	}

	/* Create a port for communication */
	if (portCreate(&can.oid.port) < 0) {
		fprintf(stderr, "zynqmp-can: cannot open port\n");
		return EXIT_FAILURE;
	}

	/* Create the device file */
	char path[12];
	snprintf(path, sizeof(path), "can%u", can_id);
	if (create_dev(&can.oid, path) < 0) {
		fprintf(stderr, "zynqmp-can: cannot create device file\n");
		return EXIT_FAILURE;
	}

	/* Start the message handling thread */
	zynqmpCan_thread(NULL);

	return EXIT_SUCCESS;
}
