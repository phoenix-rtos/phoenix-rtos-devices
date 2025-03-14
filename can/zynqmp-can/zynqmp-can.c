/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * ZynqMP CAN driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Dariusz Sabala
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <paths.h>

#include <sys/msg.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/file.h>
#include <sys/debug.h>
#include <sys/threads.h>
#include <sys/platform.h>
#include <sys/interrupt.h>

#include <board_config.h>
#include <libklog.h>
#include <posix/utils.h>

#include <phoenix/ioctl.h>
#include <phoenix/arch/aarch64/zynqmp/zynqmp.h>

#include "zynqmp-can-priv.h"
#include "zynqmp-can-if.h"


/* CAN peripheral instances */
#define CAN_0  0
#define CAN_1  1
#define CAN_COUNT 2

/* CAN peripheral settings */
#define CAN_BAUDRATE_MIN                   100000
#define CAN_BAUDRATE_MAX                  1000000
#define CAN_BAUDRATE_DEFAULT   (CAN_BAUDRATE_MAX)

/** @brief ZynqMP CAN0 base register address */
#define ZYNQMP_CAN0_BASE (0x00FF060000)

/** @brief CAN1 base register address */
#define ZYNQMP_CAN1_BASE (0x00FF070000)

/** @brief CAN peripheral registers map */
typedef struct {
  volatile uint32_t SRR;              /* Software Reset and Enable, Address offset:  0x0 */
  volatile uint32_t MSR;              /* Mode Select, Address offset:  0x4 */
  volatile uint32_t BRPR;             /* Baud Rate Prescaler, Address offset:  0x8 */
  volatile uint32_t BTR;              /* Bit Timing and Synchronization, Address offset:  0xc */
  volatile const uint32_t ECR;        /* Rx and Tx Error Counters, Address offset: 0x10 */
  volatile uint32_t ESR;              /* Error Status, Address offset: 0x14 */
  volatile uint32_t SR;               /* Controller Status, Address offset: 0x18 */
  volatile uint32_t ISR;              /* Interrupt Status, Address offset: 0x1c */
  volatile uint32_t IER;              /* Interrupt Enable, Address offset: 0x20 */
  volatile uint32_t ICR;              /* Interrupt Clear, Address offset: 0x24 */
  volatile uint32_t TCR;              /* Timestamp Clear., Address offset: 0x28 */
  volatile uint32_t WIR;              /* Rx and Tx Watermark Settings., Address offset: 0x2c */
  volatile uint32_t TXFIFO_ID;        /* Tx Message FIFO, Identifier, Request., Address offset: 0x30 */
  volatile uint32_t TXFIFO_DLC;       /* Tx Message FIFO Data Length Code., Address offset: 0x34 */
  volatile uint32_t TXFIFO_DATA1;     /* Tx Message FIFO, data word 1., Address offset: 0x38 */
  volatile uint32_t TXFIFO_DATA2;     /* Tx Message FIFO, data word 2., Address offset: 0x3c */
  volatile uint32_t TXHPB_ID;         /* High Priority Tx Message FIFO, Identifier, Request., Address offset: 0x40 */
  volatile uint32_t TXHPB_DLC;        /* High Priority Tx Message FIFO Data Length Code., Address offset: 0x44 */
  volatile uint32_t TXHPB_DATA1;      /* High Priority Tx Message FIFO, data word 1., Address offset: 0x48 */
  volatile uint32_t TXHPB_DATA2;      /* High Priority Tx Message FIFO, data word 0., Address offset: 0x4c */
  volatile const uint32_t RXFIFO_ID;  /* Rx Message FIFO, Identifier, Request., Address offset: 0x50 */
  volatile uint32_t RXFIFO_DLC;       /* Rx Message FIFO Data Length Code., Address offset: 0x54 */
  volatile uint32_t RXFIFO_DATA1;     /* Rx Message FIFO, data word 1., Address offset: 0x58 */
  volatile uint32_t RXFIFO_DATA2;     /* Rx Message FIFO, data word 2., Address offset: 0x5c */
  volatile uint32_t AFR;              /* Acceptance Filter Enables., Address offset: 0x60 */
  volatile uint32_t AFMR1;            /* Acceptance Filter 1 Mask., Address offset: 0x64 */
  volatile uint32_t AFIR1;            /* Acceptance Filter 1 ID., Address offset: 0x68 */
  volatile uint32_t AFMR2;            /* Acceptance Filter 2 Mask., Address offset: 0x6c */
  volatile uint32_t AFIR2;            /* Acceptance Filter 2 ID., Address offset: 0x70 */
  volatile uint32_t AFMR3;            /* Acceptance Filter 3 Mask., Address offset: 0x74 */
  volatile uint32_t AFIR3;            /* Acceptance Filter 3 ID., Address offset: 0x78 */
  volatile uint32_t AFMR4;            /* Acceptance Filter 4 Mask., Address offset: 0x7c */
  volatile uint32_t AFIR4;            /* Acceptance Filter 4 ID., Address offset: 0x80 */
} can_periph_t;

/* Structure holding data used by driver */
typedef struct {
	volatile can_periph_t *base;
	oid_t oid;
	handle_t cond;
	handle_t inth;
	handle_t lock;
	uint8_t stack[_PAGE_SIZE] __attribute__((aligned(16)));
} can_t;

/* Agregated informations about CAN instances */
static const struct {
	uint32_t base;
	unsigned int irq;
	uint16_t clk;
	uint16_t rst;
	uint16_t rxPin;
	uint16_t txPin;
} info[CAN_COUNT] = {
	{ ZYNQMP_CAN0_BASE, 55, pctl_devclock_lpd_can0, pctl_devreset_lpd_can0, CAN0_RX, CAN0_TX },
	{ ZYNQMP_CAN1_BASE, 56, pctl_devclock_lpd_can1, pctl_devreset_lpd_can1, CAN1_RX, CAN1_TX }
};

/* Structure holding data used by driver */
static struct {
	can_t can;
	uint8_t stack[_PAGE_SIZE] __attribute__((aligned(16))); /* stack of msg dispatch thread */
} can_common;


static void cli_help(const char *progname)
{
	printf("Usage: %s [OPTIONS]\n", progname);
	printf("Options:\n");
    printf("\t-n <id>        - CAN controller ID [0...1] default 0 \n");
	printf("\t-b <baudrate>  - baudrate [100'000 bit/s ... 1'000'000 bit/s] default 1 Mbit/s\n");
	printf("\t-h             - print this message\n");
}


static int init_clock(int can_id)
{
	platformctl_t ctl;

	ctl.action = pctl_set;
	ctl.type = pctl_devclock;

	/* Set IO PLL as source clock and set divider:
	 * IO_PLL / 20 :  1000 MHz / 20 = 50 MHz     */
	ctl.devclock.dev = info[can_id].clk;
	ctl.devclock.src = 0;
	ctl.devclock.div0 = 20;
	ctl.devclock.div1 = 0;
	ctl.devclock.active = 0x1;

	return platformctl(&ctl);
}


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


static int reset_can_peripheral(int can_id)
{
	platformctl_t ctl;

	ctl.action = pctl_set;
	ctl.type = pctl_devreset;

	ctl.devreset.dev = info[can_id].rst;
	ctl.devreset.state = 0;

	return platformctl(&ctl);
}


static int can_irq_hdl(unsigned int can_id, void *arg)
{
	can_t *can = (can_t *)arg;

	/* RX FIFO Non empty IRQ occurred */
    if (can->base->ISR & (1 << 7)) {
        can->base->ICR = (1 << 7); /* Clear interrupt */
        //can->base->IER = 0; /* Turn off interrupt */
	}

    /* Return 1 to trigger send signal on conditional variable */
	return 1;
}


static int can_init(unsigned int can_id, int baudrate)
{
    /* This routine follows "Example: Start-up Sequence" from UG1085 p. 587 */
    can_t *can = &can_common.can;
    uint32_t reg_val;

    /* 1. Initialise clock */
    if (init_clock(can_id) < 0) {
        debug("zynqmp-can: cannot initialize clocks\n");
        return EXIT_FAILURE;
    }

    /* 2. Configure MIO pin */
	if (config_mio_pin(info[can_id].rxPin) < 0 || config_mio_pin(info[can_id].txPin) < 0) {
        debug("zynqmp-can: fail to initialise mio pins");
		return -EINVAL;
	}

    /* Map access to peripheral */
    can->base = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, info[can_id].base);
	if (can->base == MAP_FAILED) {
        debug("zynqmp-can: fail map peripheral");
		return -ENOMEM;
	}

    if (condCreate(&can->cond) != EOK) {
		munmap((void *)can->base, _PAGE_SIZE);
        debug("zynqmp-can: fail to create conditional var");
		return -ENOENT;
	}

	if (mutexCreate(&can->lock) != EOK) {
        debug("zynqmp-can: fail to create mutex");
		munmap((void *)can->base, _PAGE_SIZE);
		resourceDestroy(can->cond);
		return -ENOENT;
	}

    /* Reset CAN peripheral using reset subsystem */
    if (reset_can_peripheral(can_id) < 0) {
        debug("zynqmp-can: fail to reset CAN peripheral");
        return -EINVAL;
    }

    /* Perform self test if CAN peripheral is in correct state */
    uint32_t wir_value = can->base->WIR;
    if (wir_value != 0x00003F3F) {
        debug("zynqmp-can: WIR register invalid\n");
    }

    /* Wait for configuration mode */
	do {
		reg_val = can->base->SR & (1 << 0);
	} while (reg_val != 1);

    /* 5. Program the bit sampling clock */
    can->base->BRPR = 5 - 1; /* 50MHz / 5 = 10 MHz */
    can->base->BTR = ((8 - 1) << 0) /* Time segment 1 = 8 */
        | ((1 - 1) << 4)  /* Time segment 2 = 1 */
        | ((1 - 1) << 7); /* Sync jump = 1 */

    /* 6. Program the interrupts */
    can->base->IER = (1 << 7); /* RX FIFO non empty IRQ */

    /* 7. Skip enabling acceptance filters */

    /* 8. Enter normal mode, skip this */

    /* Register interrupt handler */
    interrupt(info[can_id].irq, can_irq_hdl, can, can->cond, &can->inth);

    /* 9. Enable the controller */
    can->base->SRR = (1 << 1);

    return 0;
}


static int send_frames(zynqmp_canFrame *frames, uint32_t count)
{
    can_t *can = &can_common.can;

    for (uint32_t i = 0; i < count; i++) {
        can->base->TXFIFO_ID = (frames[i].id << 21);
        can->base->TXFIFO_DLC = (frames[i].len << 28);
        can->base->TXFIFO_DATA1 = frames[i].payload.words[0];
        can->base->TXFIFO_DATA2 = frames[i].payload.words[1];
    }

    return 0;
}


static int recv_frames(zynqmp_canFrame *buf, uint32_t bufLen, uint32_t *recvFrames)
{
    /* Wait for data ready in RX FIFO */
    can_t *can = &can_common.can;
    condWait(can->cond, can->lock, 0);

    /* Read all received frames */
    *recvFrames = 0;
    for (uint32_t i = 0; i < bufLen; i++) {
        uint32_t isr = can->base->ISR;
        printf("ISR before = %u\n", isr);
        if (0 == (isr & (1 << 7))) {
            break;
        }

        /* Read frame */
        buf[i].id = (can->base->RXFIFO_ID >> 21) & 0x7FF;
        buf[i].len = (can->base->RXFIFO_DLC >> 28) & 0xF;
        buf[i].payload.words[0] = can->base->RXFIFO_DATA1;
        buf[i].payload.words[1] = can->base->RXFIFO_DATA2;
        (*recvFrames)++;

        /* Check if there are still frames in FIFO, if no then return */
        isr = can->base->ISR;
        printf("ISR after = %u\n", isr);
    }

    can->base->ICR = (1 << 7); /* Clear interrupt */
    //can->base->IER = (1 << 7); /* RX FIFO non empty IRQ */

    return 0;
}


static void can_thread(void *arg)
{
    msg_t msg;
	msg_rid_t rid;
	uint32_t port = can_common.can.oid.port;

    can_t *can = &can_common.can;
    mutexLock(can->lock);

	for (;;) {
        /* Wait infinitely for a message */
		if (msgRecv(port, &msg, &rid) < 0) {
			continue;
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.err = 0;
				break;

			case mtWrite:
                msg.o.err = -ENOSYS;
				break;

			case mtRead: {
                printf("SRR: 0x%x\n", can->base->SRR);
                printf("MSR: 0x%x\n", can->base->MSR);
                printf("BRPR:0x%x\n", can->base->BRPR);
                printf("BTR: 0x%x\n", can->base->BTR);
                printf("ECR: 0x%x\n", can->base->ECR);
                printf("ESR: 0x%x\n", can->base->ESR);
                printf("IER: 0x%x\n", can->base->IER);
                printf("AFR: 0x%x\n", can->base->AFR);
                msg.o.err = -ENOSYS;
            	}break;

			case mtGetAttr:
				msg.o.err = -ENOSYS;
				break;

			case mtDevCtl: {
                /* Choose operation based on opcode */
                if (msg.i.io.mode == zynqmp_can_opSend) {
                    zynqmp_canFrame *frames = (zynqmp_canFrame *)msg.i.data;
                    uint32_t count = msg.i.size / sizeof(zynqmp_canFrame);;
                    msg.o.err = send_frames(frames, count);
                }
                else if (msg.i.io.mode == zynqmp_can_opRecv) {
                    zynqmp_canFrame *buf = (zynqmp_canFrame *)msg.o.data;
                    uint32_t bufLen = msg.o.size / sizeof(zynqmp_canFrame);
                    uint32_t recvFrames = 0;
                    msg.o.err = recv_frames(buf, bufLen, &recvFrames);
                    msg.o.attr.val = recvFrames;
                }
                else {
                    printf("zynqmp-can: invalid opcode");
                    msg.o.err = -1;
                }
            } break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(port, &msg, rid);
	}

    mutexUnlock(can->lock);
}


int main(int argc, char **argv)
{
    int c = 0;

    /* Default arguments */
    int can_id = CAN_1;
	int baudrate = CAN_BAUDRATE_DEFAULT;

    /* Parse arguments passed */
	if (argc > 1) {
		while ((c = getopt(argc, argv, "n:b:h")) != -1) {
			switch (c) {
				case 'b':
                    baudrate = atoi(optarg);
					if ((baudrate < CAN_BAUDRATE_MIN) || (baudrate > CAN_BAUDRATE_MAX)) {
						debug("zynqmp-can: wrong baudrate value\n");
						return EXIT_FAILURE;
					}
					break;

				case 'n':
                    can_id = atoi(optarg);
					if ((can_id < CAN_0) || (can_id > CAN_1)) {
						debug("zynqmp-can: wrong CAN peripheral ID\n");
						return EXIT_FAILURE;
					}
					break;

				case 'h':
                    cli_help(argv[0]);
					return EXIT_SUCCESS;

				default:
                    cli_help(argv[0]);
					return EXIT_FAILURE;
			}
		}
	}

    /* Create port */
    if (portCreate(&can_common.can.oid.port) < 0) {
        debug("zynqmp-can: cannot open port\n");
        return EXIT_FAILURE;
    }

    /* Initialise peripheral */
    if (can_init(can_id, baudrate) < 0) {
        debug("zynqmp-can: cannot initialize CAN\n");
        return EXIT_FAILURE;
    }

    /* Create device */
    char path[12];
	snprintf(path, sizeof(path), "can%u", can_id);
    if (create_dev(&can_common.can.oid, path) < 0) {
		debug("zynqmp-can: cannot create device file\n");
	}

    /* Start message handling thread */
    can_thread(NULL);

    return EXIT_SUCCESS;
}
