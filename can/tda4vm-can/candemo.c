/*
 * Phoenix-RTOS
 *
 * TDA4VM MCU CAN 2.0 driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Rafal Mikielis
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <sys/platform.h>
#include <phoenix/arch/armv7r/tda4vm/tda4vm.h>

#include <can-tda4vm.h>
#include <gpio-tda4vm.h>


#define TX_FRAME_CNT           16U
#define STACK_SIZE             256U
#define XFER_TX                0U
#define XFER_RX                1U
#define XFER_LOOPBACK_SEQUENCE 2U
#define XFER_LOOPBACK_THREADS  3U


#define MCAN_GPIO_MODULE   WKUP_GPIO0_IDX
#define MCAN_GPIO_EN_PIN   0U
#define MCAN_GPIO_nSTB_PIN 54U


char txThreadStack[STACK_SIZE] __attribute__((aligned(8)));
;
char rxThreadStack[STACK_SIZE] __attribute__((aligned(8)));
;

struct mcan_txBuff txBuff[TX_FRAME_CNT];

static struct mcu_mcanModuleInit init;
static struct mcan_txReq txReq;
static struct mcan_rxReq rxReq;
static struct mcan_txEvent txEvt;
int tidRx, tidTx;


static void printRxFrames(struct mcan_rxReq *rxReq)
{
	struct mcan_rxMessage *data = rxReq->data;

	for (int i = 0; i < rxReq->cnt; i++) {
		printf("[MCAN DEMO]: Rx frame number %d:\n", i);
		printf("esi: %u\n", data->esi);
		printf("xtd: %u\n", data->xtd);
		printf("rtr: %u\n", data->rtr);
		printf("id: %u\n", data->id);
		printf("anmf: %u\n", data->anmf);
		printf("fdf: %u\n", data->fdf);
		printf("fidx: %u\n", data->fidx);
		printf("dlc: %u\n", data->dlc);
		printf("rxts: %u ms\n", data->rxts);
		for (int j = 0; j < data->dlc; j++) {
			printf("payload byte %d: %u\n", j, data->data[j]);
		}
		printf("\n");
		data += 1;
	}
}


static void printTxEvt(struct mcan_txEvent *txEvt)
{
	printf("[MCAN CORE] Tx Event:\n");
	printf("esi: %u\n", (uint32_t)txEvt->esi);
	printf("xtd: %u\n", txEvt->xtd);
	printf("rtr: %u\n", txEvt->rtr);
	printf("mm: %u\n", txEvt->mm);
	printf("et: %u\n", txEvt->et);
	printf("fdf: %u\n", txEvt->fdf);
	printf("brs: %u\n", txEvt->brs);
	printf("dlc: %u\n", txEvt->dlc);
	printf("id: %u\n", txEvt->id);
	printf("txts: %u ms\n\n", txEvt->txts);
}


static void xferTx(void *arg)
{
	int ret;

	ret = mcan_txData(&txReq);
	if (ret < 0) {
		fprintf(stderr, "[MCAN DEMO] tx failed with error %s\n", strerror(ret));
		return;
	}
}


static void xferTxThread(void *arg)
{
	xferTx(NULL);
	endthread();
}


static void xferRx(void *arg)
{
	int ret;
	ret = mcan_rxData(&rxReq);
	if (ret < 0) {
		fprintf(stderr, "[MCAN DEMO] rx failed with error %s\n", strerror(ret));
		return;
	}
}


static void xferRxThread(void *arg)
{
	xferRx(NULL);
	endthread();
}


static void printHelp(void)
{
	printf("Command line arguments:\n");
	printf(" -s <size>:  count of transfer frames\n");
	printf(" -t <transfer type>: (0 - tx, 1 - rx, 2 - loopback (sequence), 3 - loopback (threads))\n");
}


int main(int argc, char **argv)
{
	int ret, opt;
	int xferSize;
	uint8_t xferType;

	/* default vales */
	xferSize = 8U;
	xferType = XFER_TX;

	/* Process options */
	if (argc < 3) {
		printf("Wrong argument count\n");
		printHelp();
		return -1;
	}
	while ((opt = getopt(argc, argv, "s:t:")) != -1) {
		switch (opt) {
			case 's':
				xferSize = (uint32_t)strtoul(optarg, NULL, 10);
				if (xferSize < 1 || xferSize > TX_FRAME_CNT) {
					fprintf(stderr, "Wrong number of frames (1-%d)\n", TX_FRAME_CNT);
					return -1;
				}
				break;
			case 't':
				xferType = (uint32_t)strtoul(optarg, NULL, 10);
				if (xferType < XFER_TX || xferType > XFER_LOOPBACK_THREADS) {
					fprintf(stderr, "Wrong transfer type...\n");
					printHelp();
					return -1;
				}
				break;
			case 'h':
			default:
				printHelp();
				return -1;
		}
	}

	for (int i = 0; i < TX_FRAME_CNT; i++) {
		for (int j = 0; j < 8U; j++) {
			txBuff[i].payload[j] = (i * 8U) + j;
			txBuff[i].dlc = 8U;
		}
	}

	/* MCAN module initialisation parameters */
	init.idx = MCU_MCAN_MODULE0;
	init.canMode = MCU_MCAN_MODE_MONITOR;
	init.protocol = MCAN_MCAN_CC;
	init.extendedID = MCU_MCAN_STANDARDID;
	init.filterByID = MCU_MCAN_FILTERING_OFF;
	init.lowID = 0UL;
	init.highID = 0UL;
	init.dataBitrate = 1000000;

	ret = mcan_moduleInit(&init);
	if (ret != EOK) {
		fprintf(stderr, "[MCAN DEMO] initialization failed...\n");
	}

	/* Configure GPIO
	 *
	 * CAN Transceiver relies on 2 input control signals (EN & nSTB).
	 * We must set these levels through GPIO to get desired transceiver state (datasheet TCAN1043xx-Q1 SLLSEV0G)
	 */
	if (init.canMode != MCU_MCAN_MODE_LOOPBACK) {
		(void)gpio_initModule(MCAN_GPIO_MODULE);
		gpio_writePin(MCAN_GPIO_MODULE, MCAN_GPIO_nSTB_PIN, GPIO_PIN_STATE_HIGH);
		if (init.canMode == MCU_MCAN_MODE_NORMAL) {
			gpio_writePin(MCAN_GPIO_MODULE, MCAN_GPIO_EN_PIN, GPIO_PIN_STATE_HIGH);
		}
		else {
			gpio_writePin(MCAN_GPIO_MODULE, MCAN_GPIO_EN_PIN, GPIO_PIN_STATE_LOW);
		}
	}

	/* configure tx transfer */
	txReq.data = &txBuff[0];
	txReq.cnt = xferSize;
	txReq.moduleIdx = MCU_MCAN_MODULE0;
	txReq.msgID = 1UL;
	txReq.rtr = 0UL;

	/* configure rx transer */
	rxReq.data = malloc(xferSize * sizeof(struct mcan_rxMessage));
	rxReq.cnt = xferSize;
	rxReq.moduleIdx = MCU_MCAN_MODULE0;

	if (xferType == XFER_TX) {
		xferTx(NULL);
	}
	else if (xferType == XFER_RX) {
		xferRx(NULL);
	}
	else if (xferType == XFER_LOOPBACK_SEQUENCE) {
		xferTx(NULL);
		sleep(1);
		xferRx(NULL);
	}
	else if (xferType == XFER_LOOPBACK_THREADS) {
		tidTx = beginthread(xferTxThread, 4U, (void *)txThreadStack, STACK_SIZE, NULL);
		if (tidTx < 0) {
			fprintf(stderr, "[MCAN DEMO] failed to create tidTx\n");
			return -ENOMEM;
		}
		tidRx = beginthread(xferRxThread, 4U, (void *)rxThreadStack, STACK_SIZE, NULL);
		if (tidRx < 0) {
			fprintf(stderr, "[MCAN DEMO] failed to create tidRx\n");
			return -ENOMEM;
		}

		threadJoin(-1, 0);
		threadJoin(-1, 0);
	}

	/* print Tx Events */
	if (xferType != XFER_RX) {
		for (int i = 0; i < txReq.cnt; i++) {
			(void)mcan_getTxEvt(&txEvt);
			printTxEvt(&txEvt);
		}
	}

	/* print Rx frames */
	if (xferType != XFER_TX) {
		printRxFrames(&rxReq);
	}

	sleep(1);

	mcan_moduleDeinit(init.idx);

	(void)gpio_deinitModule(MCAN_GPIO_MODULE);

	return 0;
}
