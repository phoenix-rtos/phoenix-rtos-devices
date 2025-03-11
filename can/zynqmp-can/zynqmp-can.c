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

#include <sys/debug.h>
#include <sys/file.h>
#include <sys/interrupt.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/platform.h>
#include <sys/stat.h>
#include <sys/threads.h>

#include <board_config.h>

#include <posix/utils.h>

#include <phoenix/arch/aarch64/zynqmp/zynqmp.h>

#include "zynqmp-can-priv.h"
#include "zynqmp-can-if.h"

/* Macro to calculate the length of an array */
#define ARRAY_LENGTH(array) (sizeof((array)) / sizeof((array)[0]))

/**
 * Watermark configuration for "TX FIFO empty elements above watermark" interrupt.
 * Setting this to low values (e.g., 1) reduces the blocking time for the sending function
 * but increases the frequency of interrupts.
 */
#define CAN_TX_FIFO_WATERMARK 4

/* Default CAN baud rate in kbps */
#define CAN_BAUDRATE_KBPS_DEFAULT (1000)

/* Base addresses for ZynqMP CAN peripherals */
#define ZYNQMP_CAN0_BASE (0x00FF060000)
#define ZYNQMP_CAN1_BASE (0x00FF070000)

/* CAN peripheral instances */
enum { can_0 = 0,
	can_1 = 1,
	canInstancesCount = 2 };

/* Structure representing the CAN peripheral registers */
typedef struct {
	uint32_t SRR;             /* Software Reset and Enable, Address offset:  0x0 */
	uint32_t MSR;             /* Mode Select, Address offset:  0x4 */
	uint32_t BRPR;            /* Baud Rate Prescaler, Address offset:  0x8 */
	uint32_t BTR;             /* Bit Timing and Synchronization, Address offset:  0xc */
	const uint32_t ECR;       /* Rx and Tx Error Counters, Address offset: 0x10 */
	uint32_t ESR;             /* Error Status, Address offset: 0x14 */
	uint32_t SR;              /* Controller Status, Address offset: 0x18 */
	uint32_t ISR;             /* Interrupt Status, Address offset: 0x1c */
	uint32_t IER;             /* Interrupt Enable, Address offset: 0x20 */
	uint32_t ICR;             /* Interrupt Clear, Address offset: 0x24 */
	uint32_t TCR;             /* Timestamp Clear., Address offset: 0x28 */
	uint32_t WIR;             /* Rx and Tx Watermark Settings., Address offset: 0x2c */
	uint32_t TXFIFO_ID;       /* Tx Message FIFO, Identifier, Request., Address offset: 0x30 */
	uint32_t TXFIFO_DLC;      /* Tx Message FIFO Data Length Code., Address offset: 0x34 */
	uint32_t TXFIFO_DATA1;    /* Tx Message FIFO, data word 1., Address offset: 0x38 */
	uint32_t TXFIFO_DATA2;    /* Tx Message FIFO, data word 2., Address offset: 0x3c */
	uint32_t TXHPB_ID;        /* High Priority Tx Message FIFO, Identifier, Request., Address offset: 0x40 */
	uint32_t TXHPB_DLC;       /* High Priority Tx Message FIFO Data Length Code., Address offset: 0x44 */
	uint32_t TXHPB_DATA1;     /* High Priority Tx Message FIFO, data word 1., Address offset: 0x48 */
	uint32_t TXHPB_DATA2;     /* High Priority Tx Message FIFO, data word 0., Address offset: 0x4c */
	const uint32_t RXFIFO_ID; /* Rx Message FIFO, Identifier, Request., Address offset: 0x50 */
	uint32_t RXFIFO_DLC;      /* Rx Message FIFO Data Length Code., Address offset: 0x54 */
	uint32_t RXFIFO_DATA1;    /* Rx Message FIFO, data word 1., Address offset: 0x58 */
	uint32_t RXFIFO_DATA2;    /* Rx Message FIFO, data word 2., Address offset: 0x5c */
	uint32_t AFR;             /* Acceptance Filter Enables., Address offset: 0x60 */
	uint32_t AFMR1;           /* Acceptance Filter 1 Mask., Address offset: 0x64 */
	uint32_t AFIR1;           /* Acceptance Filter 1 ID., Address offset: 0x68 */
	uint32_t AFMR2;           /* Acceptance Filter 2 Mask., Address offset: 0x6c */
	uint32_t AFIR2;           /* Acceptance Filter 2 ID., Address offset: 0x70 */
	uint32_t AFMR3;           /* Acceptance Filter 3 Mask., Address offset: 0x74 */
	uint32_t AFIR3;           /* Acceptance Filter 3 ID., Address offset: 0x78 */
	uint32_t AFMR4;           /* Acceptance Filter 4 Mask., Address offset: 0x7c */
	uint32_t AFIR4;           /* Acceptance Filter 4 ID., Address offset: 0x80 */
} can_periph_t;

/* Data structure used by the driver */
static struct {
	volatile can_periph_t *base; /**< Pointer to the peripheral base address */
	oid_t oid;                   /**< Object identifier for the related device file */
	handle_t cond;               /**< Conditional variable for synchronizing interrupts */
	handle_t inth;               /**< Interrupt handler object */
	handle_t lock;               /**< Mutex used with the conditional variable for synchronization */
} can = { 0 };

/* Immutable information about CAN peripheral instances */
static const struct {
	uint32_t base;    /**< Peripheral base address */
	unsigned int irq; /**< IRQ number */
	uint16_t clk;     /**< Identifier of the related clock in CPU clock registers */
	uint16_t rst;     /**< Identifier of the related reset line in the CPU reset peripheral */
	uint16_t rxPin;   /**< CAN RX pin (configured in board_config.h) */
	uint16_t txPin;   /**< CAN TX pin (configured in board_config.h) */
} info[canInstancesCount] = {
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
static void cli_help(const char *progname)
{
	printf("Usage: %s [OPTIONS]\n", progname);
	printf("Options:\n");
	printf("\t-n <id>        - CAN controller ID [0...1] \n");
	printf("\t-b <baudrate>  - Baud rate [kbps] supported (sample points in brackets): ");
	for (uint32_t i = 0; i < ARRAY_LENGTH(timeCfg); i++) {
		printf("%u (%s%%), ", timeCfg[i].baudrate_kbps, timeCfg[i].sample);
	}
	printf("\n");
	printf("\t-h             - Print this message\n");
}

/**
 * Initializes the clock for the specified CAN peripheral.
 * Sets the IO_PLL as the clock source and configures the divider.
 */
static int init_clock(int can_id)
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
static int config_mio_pin(uint32_t pin)
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
static int reset_can_peripheral(int can_id)
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
static int can_irq_hdl(unsigned int can_id, void *arg)
{
	uint32_t isr = can.base->ISR;

	/* RX FIFO Non-empty IRQ occurred */
	if (isr & (1 << 7)) {
		can.base->IER &= ~(1 << 7);
	}

	/* TX FIFO empty IRQ occurred */
	if (isr & (1 << 14)) {
		can.base->IER &= ~(1 << 14);
	}

	/* TX FIFO empty elements above watermark IRQ occurred */
	if (isr & (1 << 13)) {
		can.base->IER &= ~(1 << 13);
	}

	return 1;
}

/**
 * Initializes the CAN peripheral with the specified ID and baud rate.
 * Configures the clock, MIO pins, and synchronization primitives.
 */
static int can_init(unsigned int can_id, int baudrate)
{
	uint32_t reg_val;

	/* Find the matching time configuration */
	bool timeCfgFound = false;
	uint32_t timeCfgN = 0;
	for (uint32_t i = 0; i < ARRAY_LENGTH(timeCfg); i++) {
		if (timeCfg[i].baudrate_kbps == baudrate) {
			timeCfgFound = true;
			timeCfgN = i;
		}
	}
	if (!timeCfgFound) {
		debug("zynqmp-can: failed to configure desired baud rate\n");
		return EXIT_FAILURE;
	}

	/* Initialize the clock */
	if (init_clock(can_id) < 0) {
		debug("zynqmp-can: cannot initialize clocks\n");
		return EXIT_FAILURE;
	}

	/* Configure MIO pins */
	if (config_mio_pin(info[can_id].rxPin) < 0 || config_mio_pin(info[can_id].txPin) < 0) {
		debug("zynqmp-can: failed to initialize MIO pins\n");
		return -EINVAL;
	}

	/* Map access to the peripheral */
	can.base = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, info[can_id].base);
	if (can.base == MAP_FAILED) {
		debug("zynqmp-can: failed to map peripheral\n");
		return -ENOMEM;
	}

	/* Initialize synchronization primitives */
	if (condCreate(&can.cond) != EOK) {
		munmap((void *)can.base, _PAGE_SIZE);
		debug("zynqmp-can: failed to create conditional variable\n");
		return -ENOENT;
	}
	if (mutexCreate(&can.lock) != EOK) {
		debug("zynqmp-can: failed to create mutex\n");
		munmap((void *)can.base, _PAGE_SIZE);
		resourceDestroy(can.cond);
		return -ENOENT;
	}

	/* Reset the CAN peripheral */
	if (reset_can_peripheral(can_id) < 0) {
		debug("zynqmp-can: failed to reset CAN peripheral\n");
		return -EINVAL;
	}

	/* Wait for configuration mode */
	do {
		reg_val = can.base->SR & (1 << 0);
	} while (reg_val != 1);

	/* Program the bit sampling clock */
	can.base->BRPR = timeCfg[timeCfgN].brpr;
	can.base->BTR = timeCfg[timeCfgN].btr;

	/* Configure the watermark for TX FIFO */
	can.base->WIR = (CAN_TX_FIFO_WATERMARK << 8);

	/* Register the interrupt handler */
	interrupt(info[can_id].irq, can_irq_hdl, &can, can.cond, &can.inth);

	/* Enable the controller */
	can.base->SRR = (1 << 1);

	printf("zynqmp-can: Initialized CAN%u with baud rate: %u kbps, sample point: %s%%\n",
			can_id,
			timeCfg[timeCfgN].baudrate_kbps,
			timeCfg[timeCfgN].sample);

	return 0;
}

/**
 * Swaps the endianness of a 32-bit value.
 */
uint32_t swap_endianness(uint32_t value)
{
	return ((value >> 24) & 0x000000FF) |
			((value >> 8) & 0x0000FF00) |
			((value << 8) & 0x00FF0000) |
			((value << 24) & 0xFF000000);
}

/**
 * Sends CAN frames to the hardware TX FIFO.
 * Blocks if the FIFO is full and blocking is enabled.
 */
static int send_frames(zynqmp_canFrame *frames, uint32_t count, bool block, uint32_t *sendFrames)
{
	*sendFrames = 0;

	/* Flush all frames to the hardware TX FIFO */
	for (uint32_t i = 0; i < count; i++) {
		/* Check if TX FIFO is not full */
		if (0U != (can.base->SR & (1 << 10))) {
			if (block) {
				/* Disable, clear, and enable "Transmit FIFO Watermark Empty" interrupt */
				can.base->IER &= ~(1 << 13);
				can.base->ICR = (1 << 13);
				can.base->IER |= (1 << 13);

				/* Wait for "Transmit FIFO Watermark Empty" interrupt */
				condWait(can.cond, can.lock, 0);
			}
			else {
				/* Not enough space in the buffer */
				return -ENOMEM;
			}
		}

		/* Write a single CAN frame into the hardware TX FIFO */
		can.base->TXFIFO_ID = (frames[i].id << 21);
		can.base->TXFIFO_DLC = (frames[i].len << 28);
		can.base->TXFIFO_DATA1 = swap_endianness(frames[i].payload.words[0]);
		can.base->TXFIFO_DATA2 = swap_endianness(frames[i].payload.words[1]);
		(*sendFrames)++;
	}

	return EOK;
}

/**
 * Receives CAN frames from the hardware RX FIFO.
 * Blocks if the FIFO is empty and a timeout is specified.
 */
static int recv_frames(zynqmp_canFrame *buf, uint32_t bufLen, time_t timeoutUs, uint32_t *recvFrames)
{
	*recvFrames = 0;

	/* Disable, clear, and enable "Receive FIFO Not Empty" interrupt */
	can.base->IER &= ~(1 << 7);
	can.base->ICR = (1 << 7);
	can.base->IER |= (1 << 7);

	/* Wait for data in the RX FIFO (signaled by "Receive FIFO Not Empty" IRQ) */
	int condRet = condWait(can.cond, can.lock, timeoutUs);
	if (condRet == -ETIME) {
		/* Disable "Receive FIFO Not Empty" interrupt */
		can.base->IER &= ~(1 << 7);
		return condRet;
	}
	else if (condRet == EOK) {
		/* Do nothing, start reading frames */
	}
	else {
		/* Disable "Receive FIFO Not Empty" interrupt */
		can.base->IER &= ~(1 << 7);
		printf("zynqmp-can: unknown condWait error %i\n", condRet);
		return condRet;
	}

	/* Read all received frames */
	for (uint32_t i = 0; i < bufLen; i++) {
		/* Try clearing the interrupt (it will only clear if there is no data) */
		can.base->ICR = (1 << 7);

		/* Check if there is still data in the RX FIFO */
		uint32_t isr = can.base->ISR;
		if (0 == (isr & (1 << 7))) {
			break;
		}

		/* Read a frame from the FIFO */
		buf[i].id = (can.base->RXFIFO_ID >> 21) & 0x7FF;
		buf[i].len = (can.base->RXFIFO_DLC >> 28) & 0xF;
		buf[i].payload.words[0] = swap_endianness(can.base->RXFIFO_DATA1);
		buf[i].payload.words[1] = swap_endianness(can.base->RXFIFO_DATA2);
		(*recvFrames)++;
	}

	return EOK;
}

/**
 * Main thread for handling CAN operations.
 * Processes incoming messages and performs the requested operations.
 */
static void can_thread(void *arg)
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
				if (req->operation == zynqmp_can_opSend) {
					zynqmp_canFrame *frames = (zynqmp_canFrame *)msg.i.data;
					uint32_t count = msg.i.size / sizeof(zynqmp_canFrame);
					msg.o.err = send_frames(frames, count, req->blockTxOperation, &resp->framesReceivedSend);
				}
				else if (req->operation == zynqmp_can_opRecv) {
					zynqmp_canFrame *buf = (zynqmp_canFrame *)msg.o.data;
					uint32_t bufLen = msg.o.size / sizeof(zynqmp_canFrame);
					msg.o.err = recv_frames(buf, bufLen, req->timeoutUs, &resp->framesReceivedSend);
				}
				else {
					printf("zynqmp-can: unknown opcode\n");
					msg.o.err = -ENOSYS;
				}
			} break;

			case mtOpen:
			case mtClose:
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
	int can_id = can_1;
	int baudrate = CAN_BAUDRATE_KBPS_DEFAULT;

	/* Parse command-line arguments */
	if (argc > 1) {
		while ((c = getopt(argc, argv, "n:b:h")) != -1) {
			switch (c) {
				case 'b': {
					baudrate = atoi(optarg);
					/* Find the matching time configuration */
					bool timeCfgFound = false;
					for (uint32_t i = 0; i < ARRAY_LENGTH(timeCfg); i++) {
						if (timeCfg[i].baudrate_kbps == baudrate) {
							timeCfgFound = true;
						}
					}
					if (!timeCfgFound) {
						debug("zynqmp-can: failed to configure desired baud rate\n");
						return EXIT_FAILURE;
					}
				} break;

				case 'n': {
					can_id = atoi(optarg);
					if ((can_id < can_0) || (can_id > can_1)) {
						debug("zynqmp-can: invalid CAN peripheral ID\n");
						return EXIT_FAILURE;
					}
				} break;

				case 'h':
					cli_help(argv[0]);
					return EXIT_SUCCESS;

				default:
					cli_help(argv[0]);
					return EXIT_FAILURE;
			}
		}
	}

	/* Initialize the CAN peripheral */
	if (can_init(can_id, baudrate) < 0) {
		debug("zynqmp-can: cannot initialize CAN\n");
		return EXIT_FAILURE;
	}

	/* Create a port for communication */
	if (portCreate(&can.oid.port) < 0) {
		debug("zynqmp-can: cannot open port\n");
		return EXIT_FAILURE;
	}

	/* Create the device file */
	char path[12];
	snprintf(path, sizeof(path), "can%u", can_id);
	if (create_dev(&can.oid, path) < 0) {
		debug("zynqmp-can: cannot create device file\n");
	}

	/* Start the message handling thread */
	can_thread(NULL);

	return EXIT_SUCCESS;
}
