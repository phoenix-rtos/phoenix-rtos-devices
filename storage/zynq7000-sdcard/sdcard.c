/*
 * Phoenix-RTOS
 *
 * SD memory card driver
 * Compatible with SD Specifications Part A2: SD Host Controller Simplified Specification Version 2.00
 *
 * Copyright 2023 Phoenix Systems
 * Author: Ziemowit Leszczynski, Artur Miller, Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "sdcard.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <sys/mman.h>
#include <sys/platform.h>
#include <sys/threads.h>
#include <sys/interrupt.h>

#include "sdhost_defs.h"
#include "zynq7000-sdio.h"

#define LOG_TAG "sdcard"
/* clang-format off */
#define LOG_ERROR(str, ...) do { fprintf(stderr, LOG_TAG " error: " str "\n", ##__VA_ARGS__); } while (0)
#define TRACE(str, ...)     do { if (0) fprintf(stderr, LOG_TAG " trace: " str "\n", ##__VA_ARGS__); } while (0)
/* clang-format on */


#define SDHOST_ERROR_REASONS ( \
	SDHOST_INTR_CMD_ERRORS | \
	SDHOST_INTR_DAT_ERRORS | \
	SDHOST_INTR_OVERCURRENT_ERROR | \
	SDHOST_INTR_AUTO_CMD12_ERROR | \
	SDHOST_INTR_ADMA_ERROR | \
	SDHOST_INTR_DMA_ERROR)

#define SDHOST_STATUS_MASK ( \
	SDHOST_INTR_CMD_STATUS | \
	SDHOST_INTR_CARD_IN | \
	SDHOST_INTR_CARD_OUT | \
	SDHOST_ERROR_REASONS)

#define AWAITABLE_INTRS ( \
	SDHOST_INTR_CMD_DONE | \
	SDHOST_INTR_TRANSFER_DONE | \
	SDHOST_INTR_BLOCK_GAP | \
	SDHOST_INTR_CARD_OUT | \
	SDHOST_ERROR_REASONS)

/* configuration values */
#define THREAD_STACK_SIZE 1024
#define SDHOST_RETRIES    10

/* Number of blocks to erase in a single transaction
 * Number is limited to not trigger timeouts
 */
#define ERASE_N_BLOCKS 128

#define SD_FREQ_INITIAL 400000   /* 400 kHz clock used for card initialization */
#define SD_FREQ_25M     25000000 /* 25 MHz clock usable when card is initialized */
#define SD_FREQ_50M     50000000 /* 50 MHz clock usable when card is initialized and supports high speed */

#define ARRAY_LENGTH(array) (sizeof(array) / sizeof(array[0]))

typedef struct {
	/* Relative Card Address of the card. Needed for selecting the card so it can use the data bus. */
	uint32_t rca;
	/* Size of SD card in blocks */
	uint32_t sizeBlocks;
	/* Size (in blocks) of an erase sector */
	uint32_t eraseSizeBlocks;
	/* Number of command timeouts since last card initialization */
	uint32_t commandTimeouts;
	/* Whether the card supports SDHC protocol (requires slightly different handling) */
	bool highCapacity;
} sdcard_cardMetadata_t;

typedef struct {
	/* Base address (in virtual memory) of SD Host Controller register set */
	volatile uint32_t *base;
	sdcard_cardMetadata_t card;
	/* Reference clock frequency in Hz */
	uint32_t refclkFrequency;

	/* Address of DMA buffer in virtual memory (for access by the CPU) */
	void *dmaBuffer;
	/* Address of DMA buffer in physical memory (for access by the SD Host Controller) */
	addr_t dmaBufferPhys;

	bool sdioInitialized;
	bool isCDPinSupported;
	bool isWPPinSupported;

	handle_t cmdLock;
	handle_t isrHandle;
	handle_t eventLock;
	handle_t eventCond;
} sdcard_hostData_t;

static sdcard_hostData_t sdio_hosts[PLATFORM_SDIO_N_HOSTS] = { 0 };
static unsigned int initializedHosts = 0;

static struct {
	handle_t lock;
	handle_t cond;
	bool initialized;
} presenceEvents = { .initialized = false };

static inline sdcard_hostData_t *sdcard_getHostForSlot(unsigned int slot)
{
	return (slot >= PLATFORM_SDIO_N_HOSTS) ? NULL : &sdio_hosts[slot];
}


static int sdcard_configClockAndPower(sdcard_hostData_t *host, uint32_t freq);
static int sdcard_wideAndFast(sdcard_hostData_t *host);


/* Resets parts or all of the host according to the given reset type.
 * Argument must be one of CLOCK_CONTROL_RESET_*
 */
static int sdhost_reset(sdcard_hostData_t *host, uint32_t resetType)
{
	*(host->base + SDHOST_REG_CLOCK_CONTROL) |= resetType;
	sdio_dataBarrier();
	for (int i = 0; i < SDHOST_RETRIES; ++i) {
		usleep(10);
		if ((*(host->base + SDHOST_REG_CLOCK_CONTROL) & resetType) == 0) {
			return 0;
		}
	}

	return -1;
}


static inline bool sdcard_isWriteProtected(sdcard_hostData_t *host)
{
	if (host->isWPPinSupported) {
		if ((*(host->base + SDHOST_REG_PRES_STATE) & PRES_STATE_WRITE_PROT_PIN) == 0) {
			return true;
		}
	}

	return false;
}


static int sdhost_allocDMA(sdcard_hostData_t *host)
{
	if (SDCARD_MAX_TRANSFER > _PAGE_SIZE) {
		return -ENOMEM;
	}

	void *p = mmap(NULL, SDCARD_MAX_TRANSFER, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_UNCACHED | MAP_CONTIGUOUS, -1, 0);
	if (p == MAP_FAILED) {
		return -ENOMEM;
	}

	host->dmaBuffer = p;
	if (host->dmaBuffer == NULL) {
		return -ENOMEM;
	}

	host->dmaBufferPhys = va2pa(host->dmaBuffer);
	return 0;
}


static int sdhost_isr(unsigned int n, void *arg)
{
	/* SD Host controller interrupts are triggered by level, not by edge.
	 * This means that ISR has to remove the reason for interrupt (zero out bit in STATUS register)
	 * or disable the interrupt (zero out bit in SIGNAL_ENABLE register) - otherwise we get stuck
	 * in infinite interrupt loop.
	 */
	sdcard_hostData_t *host = (sdcard_hostData_t *)arg;
	*(host->base + SDHOST_REG_INTR_SIGNAL_ENABLE) = 0;
	return 0;
}


static int _sdio_cmdExecutionWait(sdcard_hostData_t *host, uint32_t flags, time_t wait_us)
{
	int ret = -ETIME;
	bool doResetCmd = false, doResetDat = false;

	*(host->base + SDHOST_REG_INTR_SIGNAL_ENABLE) = AWAITABLE_INTRS;
	/* If there is a pending interrupt from flagsWithErrors set, it will fire now */
	int waitret = condWait(host->eventCond, host->eventLock, wait_us);
	if (waitret == -ETIME) {
		/* It's a bit odd, but card ejection doesn't cause any errors - only timeouts
		 * We may get a SDHOST_INTR_CARD_OUT, but only if physical card detect is available
		 */
		*(host->base + SDHOST_REG_INTR_SIGNAL_ENABLE) = 0;
		host->card.commandTimeouts++;
	}

	uint32_t val = *(host->base + SDHOST_REG_INTR_STATUS);
	if ((val & SDHOST_ERROR_REASONS) != 0) {
		doResetCmd = (val & SDHOST_INTR_CMD_ERRORS) != 0;
		doResetDat = (val & SDHOST_INTR_DAT_ERRORS) != 0;
		*(host->base + SDHOST_REG_INTR_STATUS) = SDHOST_ERROR_REASONS;
		/* Under QEMU it is important to zero out this register if an incomplete transfer happens */
		*(host->base + SDHOST_REG_TRANSFER_BLOCK) = 0;
		ret = -EIO;
	}
	else if ((val & flags) == flags) {
		*(host->base + SDHOST_REG_INTR_STATUS) = flags;
		ret = EOK;
	}

	if ((val & SDHOST_INTR_BLOCK_GAP) != 0) {
		/* Not strictly an error, but should not happen in the current implementation */
		*(host->base + SDHOST_REG_INTR_STATUS) = SDHOST_INTR_BLOCK_GAP;
		doResetDat = true;
		ret = -EIO;
	}

	if (doResetCmd || (ret == -ETIME)) {
		sdhost_reset(host, CLOCK_CONTROL_RESET_CMD);
	}

	if (doResetDat || (ret == -ETIME)) {
		sdhost_reset(host, CLOCK_CONTROL_RESET_DAT);
	}

	if ((val & SDHOST_INTR_CARD_OUT) != 0) {
		condSignal(presenceEvents.cond);
	}

	return ret;
}


/* res is single u32 if isLongResponse == 0, array of 4 u32 otherwise */
static int _sdio_cmdSend(sdcard_hostData_t *host, uint8_t cmd, uint32_t arg, uint32_t *res, uint16_t blockCount, bool isLongResponse)
{
	sdhost_command_reg_t cmdFrame;
	uint32_t val;

	if (cmd >= MAX_SD_COMMANDS) {
		return -EINVAL;
	}

	sdhost_command_data_t dataType = sdCmdMetadata[cmd].dataType;
	if (dataType == CMD_INVALID) {
		return -EINVAL;
	}

	val = *(host->base + SDHOST_REG_PRES_STATE) & PRES_STATE_BUSY_FLAGS;
	if (val != 0) {
		TRACE("busy %x", val);
		return -EBUSY;
	}

	val = *(host->base + SDHOST_REG_INTR_STATUS) & (SDHOST_INTR_TRANSFER_DONE | SDHOST_INTR_CMD_DONE);
	if (val != 0) {
		TRACE("intr bits %x", val);
		return -EBUSY;
	}

	cmdFrame.commandIdx = cmd;
	cmdFrame.commandMeta = sdCmdMetadata[cmd].bitsWhenSending;

	if ((dataType == CMD_NO_DATA) || (dataType == CMD_NO_DATA_WAIT)) {
		*(host->base + SDHOST_REG_TRANSFER_BLOCK) = 0;
	}
	else {
		if (blockCount != 0) {
			uint32_t blockLength;
			switch (dataType) {
				case CMD_READ8:
					blockLength = 8;
					break;

				case CMD_READ64:
					blockLength = 64;
					break;

				default:
					blockLength = SDCARD_BLOCKLEN;
					break;
			}

			*(host->base + SDHOST_REG_SDMA_ADDRESS) = host->dmaBufferPhys;
			sdio_dataBarrier();
			*(host->base + SDHOST_REG_TRANSFER_BLOCK) =
				((uint32_t)blockCount << 16) |
				TRANSFER_BLOCK_SDMA_BOUNDARY_4K |
				blockLength;
		}

		cmdFrame.dataPresent = 1;
		cmdFrame.dmaEnable = 1;
		cmdFrame.blockCountEnable = 1;
		if ((dataType == CMD_READ) || (dataType == CMD_READ_MULTI) || (dataType == CMD_READ8) || (dataType == CMD_READ64)) {
			cmdFrame.directionRead = 1;
		}

		if ((dataType == CMD_READ_MULTI) || (dataType == CMD_WRITE_MULTI)) {
			cmdFrame.multiBlock = 1;
			cmdFrame.autoCmd12Enable = 1;
		}
	}

	*(host->base + SDHOST_REG_CMD_ARGUMENT) = arg;
	sdio_dataBarrier();
	mutexLock(host->eventLock);
	/* This register write starts command execution and must be done last */
	*(host->base + SDHOST_REG_CMD) = cmdFrame.raw;

	/* wait 1 ms max */
	int ret = _sdio_cmdExecutionWait(host, SDHOST_INTR_CMD_DONE, 1000);
	if (ret < 0) {
		TRACE("error %d on cmd %d", ret, cmd);
		mutexUnlock(host->eventLock);
		return ret;
	}

	if (dataType != CMD_NO_DATA) {
		/* wait 1 s max */
		ret = _sdio_cmdExecutionWait(host, SDHOST_INTR_TRANSFER_DONE, 1000 * 1000);
		if (ret < 0) {
			TRACE("error %d on cmd %d", ret, cmd);
			mutexUnlock(host->eventLock);
			return ret;
		}
	}

	mutexUnlock(host->eventLock);
	if (res != NULL) {
		int responseLen = isLongResponse ? 4 : 1;
		for (int i = 0; i < responseLen; i++) {
			res[i] = *(host->base + SDHOST_REG_RESPONSE_0 + i);
		}
	}

	return 0;
}


/* res is single u32 if isLongResponse == 0, array of 4 u32 otherwise */
static int sdio_cmdSendEx(sdcard_hostData_t *host, uint8_t cmd, uint32_t arg, uint32_t *res, bool isLongResponse, uint8_t *data)
{
	int ret;

	mutexLock(host->cmdLock);
	if ((cmd & SDIO_ACMD_BIT) != 0) {
		uint32_t resAcmd;
		ret = _sdio_cmdSend(host, SDIO_CMD55_APP_CMD, host->card.rca, &resAcmd, 0, false);
		if (ret < 0) {
			mutexUnlock(host->cmdLock);
			return ret;
		}

		if ((resAcmd & CARD_STATUS_APP_CMD) == 0) {
			LOG_ERROR("app cmd not accepted");
			mutexUnlock(host->cmdLock);
			return -EOPNOTSUPP;
		}
	}

	if (cmd >= MAX_SD_COMMANDS) {
		mutexUnlock(host->cmdLock);
		return -EINVAL;
	}

	size_t dataSize;
	switch (sdCmdMetadata[cmd].dataType) {
		case CMD_READ8:
			dataSize = 8;
			break;

		case CMD_READ64:
			dataSize = 64;
			break;

		default:
			dataSize = 0;
			break;
	}

	uint16_t blockCount = (dataSize > 0) ? 1 : 0;
	ret = _sdio_cmdSend(host, cmd, arg, res, blockCount, isLongResponse);
	if (data != NULL) {
		memcpy(data, host->dmaBuffer, dataSize);
	}

	mutexUnlock(host->cmdLock);

	return ret;
}

/* Send common command with no data transfer */
static inline int sdio_cmdSend(sdcard_hostData_t *host, uint8_t cmd, uint32_t arg, uint32_t *res)
{
	return sdio_cmdSendEx(host, cmd, arg, res, false, NULL);
}

/* Send common command with no data transfer and a long response */
static inline int sdio_cmdSendWithLongResponse(sdcard_hostData_t *host, uint8_t cmd, uint32_t arg, uint32_t res[4])
{
	return sdio_cmdSendEx(host, cmd, arg, res, true, NULL);
}


static int sdcard_getCardSize(sdcard_hostData_t *host)
{
	uint32_t resp[4];
	uint32_t blockNr;
	uint32_t eraseSectorSize;
	sdcard_cardMetadata_t *card = &host->card;
	if (sdio_cmdSendWithLongResponse(host, SDIO_CMD9_SEND_CSD, card->rca, resp) < 0) {
		return -EIO;
	}

	uint32_t csdVersion = CSD_VERSION(resp);
	if (csdVersion == 0) {
		uint32_t cSize = CSDV1_C_SIZE(resp);
		uint32_t cSizeMultiplier = CSDV1_C_SIZE_MULT(resp);
		uint32_t mult = 4 << cSizeMultiplier;
		uint32_t readBlLen = CSDV1_READ_BL_LEN(resp);
		uint32_t writeBlLen = CSDV1_WRITE_BL_LEN(resp);
		blockNr = (cSize + 1) * mult;
		if (readBlLen >= 9) {
			blockNr <<= readBlLen - 9;
		}
		else {
			blockNr >>= 9 - readBlLen;
		}

		/* In cards with CSD v1 if ERASE_BLK_EN == 0 the card may erase more blocks
		 * than we selected - rounding on both sides of the range up to eraseSectorSize.
		 */
		eraseSectorSize = CSDV1_ERASE_SECTOR_SIZE(resp);
		if (writeBlLen >= 9) {
			eraseSectorSize <<= writeBlLen - 9;
		}
		else {
			uint32_t divider = 1 << (9 - writeBlLen);
			/* Round up - it's better to erase more than necessary than try to erase less
			 * and end up erasing more than intended.
			 */
			eraseSectorSize = (eraseSectorSize + divider - 1) / divider;
		}
	}
	else if (csdVersion == 1) {
		uint32_t cSize = CSDV2_C_SIZE(resp);
		blockNr = cSize << 10;
		/* In CSD v2 this field is not used and card can always erase blocks one by one */
		eraseSectorSize = 1;
	}
	else {
		/* Unknown CSD structure version; cannot compute size. */
		return -EOPNOTSUPP;
	}

	TRACE("Memory card size: %u blocks", blockNr);
	card->sizeBlocks = blockNr;
	card->eraseSizeBlocks = eraseSectorSize;
	return 0;
}


int sdcard_initCard(unsigned int slot, bool fallbackMode)
{
	sdcard_hostData_t *host = sdcard_getHostForSlot(slot);
	if (host == NULL) {
		return -ENOENT;
	}

	host->card.commandTimeouts = 0;
	/* Switch off 4-bit mode, because card will be in 1-bit mode after CMD0 */
	*(host->base + SDHOST_REG_HOST_CONTROL) &= ~HOST_CONTROL_4_BIT_MODE;
	sdcard_configClockAndPower(host, SD_FREQ_INITIAL);
	/* Before we know the card's RCA, set to 0 to send to all cards */
	host->card.rca = 0;

	sdhost_reset(host, CLOCK_CONTROL_RESET_CMD);
	sdhost_reset(host, CLOCK_CONTROL_RESET_DAT);
	if (sdio_cmdSend(host, SDIO_CMD0_GO_IDLE_STATE, 0, NULL) < 0) {
		/* This will only fail if something is wrong with host, succeeds even with no card inserted */
		LOG_ERROR("go idle fail");
		return -EIO;
	}

	bool trySdhc = true;
	uint32_t cmd8Response;
	if (sdio_cmdSend(host, SDIO_CMD8_SEND_IF_COND, (IF_COND_ECHO_PATTERN | IF_COND_3V3_SUPPORTED), &cmd8Response) < 0) {
		/* This command is not supported on SD v1 cards so it must be treated as Standard Capacity*/
		trySdhc = false;
	}
	else {
		if ((cmd8Response & 0xff) != IF_COND_ECHO_PATTERN) {
			LOG_ERROR("connection error");
			return -EIO;
		}

		if (((cmd8Response >> 8) & 0xf) != 0b0001) {
			LOG_ERROR("incompatible voltage");
			return -EOPNOTSUPP;
		}
	}

	uint32_t acmd41Response;
	uint32_t acmd41Arg = trySdhc ? (1 << 30) : 0;
	if (sdio_cmdSend(host, SDIO_ACMD41_SD_SEND_OP_COND, acmd41Arg, &acmd41Response) < 0) {
		LOG_ERROR("op cond fail");
		return -EIO;
	}

	/* NOTE: on some hosts the host voltage could be changed at this point */
	if ((acmd41Response & ((1 << 20) | (1 << 21))) == 0) {
		LOG_ERROR("3.3V not supported");
		return -EOPNOTSUPP;
	}

	acmd41Arg |= acmd41Response & 0xffffff;
	/* According to docs timeout value for initialization process == 1 sec */
	for (int i = 0; i < 1000; i++) {
		if (sdio_cmdSend(host, SDIO_ACMD41_SD_SEND_OP_COND, acmd41Arg, &acmd41Response) < 0) {
			LOG_ERROR("waiting for card init failed");
			return -EIO;
		}

		if ((acmd41Response & IF_COND_READY) != 0) {
			break;
		}

		/* We can wait up to 50 ms before reissuing ACMD41 */
		usleep(1000);
	}

	if ((acmd41Response & IF_COND_READY) == 0) {
		return -ETIME;
	}

	host->card.highCapacity = trySdhc && (((acmd41Response >> 30) & 1) != 0);

	/* Not sure what that is for, but it's in the documentation that we should do this */
	if (sdio_cmdSend(host, SDIO_CMD2_ALL_SEND_CID, 0, NULL) < 0) {
		return -EIO;
	}

	uint32_t cardRCA;
	if (sdio_cmdSend(host, SDIO_CMD3_RELATIVE_ADDR, 0, &cardRCA) < 0) {
		return -EIO;
	}

	host->card.rca = cardRCA & 0xffff0000;
	if (sdcard_getCardSize(host) < 0) {
		LOG_ERROR("cannot determine size");
		host->card.sizeBlocks = 0;
		host->card.eraseSizeBlocks = 1;
	}

	if (sdio_cmdSend(host, SDIO_CMD7_SELECT_CARD, host->card.rca, NULL) < 0) {
		return -EIO;
	}

	if (!host->card.highCapacity) {
		if (sdio_cmdSend(host, SDIO_CMD16_SET_BLOCKLEN, SDCARD_BLOCKLEN, NULL) < 0) {
			TRACE("set blocklen fail");
			return -EIO;
		}
	}

	uint32_t finalStatus;
	if (sdio_cmdSend(host, SDIO_CMD13_SEND_STATUS, host->card.rca, &finalStatus) < 0) {
		return -EIO;
	}

	if ((finalStatus & CARD_STATUS_ERRORS) != 0) {
		return -EIO;
	}

	if (CARD_STATUS_CURRENT_STATE(finalStatus) != CARD_STATUS_CURRENT_STATE_TRAN) {
		/* Something unexpected must have happened because card is not in a state to transfer data */
		return -EIO;
	}

	return fallbackMode ? 0 : sdcard_wideAndFast(host);
}


/* Extract information about supported functions in a given function group from the function register */
static inline uint16_t sdcard_extractFunctionGroupInfo(uint8_t fnRegister[64], uint8_t fnGroup)
{
	if ((fnGroup > 6) || (fnGroup < 1)) {
		return 0;
	}

	uint8_t fnGroupIndex = (6 - fnGroup) * 2 + 2;
	return ((uint16_t)fnRegister[fnGroupIndex] << 8) | fnRegister[fnGroupIndex + 1];
}


/* Extract status of function from a given function group from the function register */
static inline uint8_t sdcard_extractFunctionSwitchResult(uint8_t fnRegister[64], uint8_t fnGroup)
{
	if ((fnGroup > 6) || (fnGroup < 1)) {
		return 0;
	}

	fnGroup = (6 - fnGroup);
	uint8_t fnGroupIndex = fnGroup / 2 + 14;
	uint8_t fnGroupBits = ((fnGroup % 2) == 0) ? 4 : 0;
	return (fnRegister[fnGroupIndex] >> fnGroupBits) & 0xf;
}


static bool sdcard_hasHighSpeedFunction(sdcard_hostData_t *host, uint8_t tmpReg[64])
{
	uint32_t getFunctionArg = SDIO_SWITCH_FUNC_GET | SDIO_SWITCH_FUNC_HIGH_SPEED;
	if (sdio_cmdSendEx(host, SDIO_CMD6_SWITCH_FUNC, getFunctionArg, NULL, false, tmpReg) < 0) {
		return false;
	}

	uint16_t accessModeFunctions = sdcard_extractFunctionGroupInfo(tmpReg, SDCARD_FUNCTION_GROUP_ACCESS_MODE);
	return (accessModeFunctions & SDCARD_FUNCTION_GROUP_ACCESS_MODE_HIGH_SPEED) != 0;
}


/* Switches card to maximum width and speed supported */
static int sdcard_wideAndFast(sdcard_hostData_t *host)
{
	uint8_t bigRegs[64];
	/* Get SD card configuration register with some useful info about the card */
	if (sdio_cmdSendEx(host, SDIO_ACMD51_SEND_SCR, 0, NULL, false, bigRegs) < 0) {
		return -EIO;
	}

	bool cmd6Supported = SCR_SD_SPEC(bigRegs) >= SCR_SD_SPEC_V1_10;
	/* In theory all SD cards should support 4-bit, but make sure */
	if ((SCR_BUS_WIDTHS(bigRegs) & SCR_BUS_WIDTHS_4_BIT) != 0) {
		if (sdio_cmdSend(host, SDIO_ACMD6_SET_BUS_WIDTH, 2, NULL) < 0) {
			LOG_ERROR("bus widening failed");
			return -EIO;
		}

		*(host->base + SDHOST_REG_HOST_CONTROL) |= HOST_CONTROL_4_BIT_MODE;
		usleep(10);
		if (sdio_cmdSendEx(host, SDIO_ACMD13_SD_STATUS, 0, NULL, false, bigRegs) < 0) {
			LOG_ERROR("bus widening failed");
			return -EIO;
		}

		if (SD_STATUS_DAT_BUS_WIDTH(bigRegs) != SD_STATUS_DAT_BUS_WIDTH_4_BIT) {
			LOG_ERROR("bus widening failed");
			return -EIO;
		}
	}

	bool isHighSpeedSupported = false;
	if (cmd6Supported && sdcard_hasHighSpeedFunction(host, bigRegs)) {
		uint32_t switchFunctionArg = SDIO_SWITCH_FUNC_SET | SDIO_SWITCH_FUNC_HIGH_SPEED;
		if (sdio_cmdSendEx(host, SDIO_CMD6_SWITCH_FUNC, switchFunctionArg, NULL, false, bigRegs) == 0) {
			if (sdcard_extractFunctionSwitchResult(bigRegs, SDCARD_FUNCTION_GROUP_ACCESS_MODE) == 1) {
				isHighSpeedSupported = true;
			}
		}
	}

	if (isHighSpeedSupported) {
		TRACE("using HS mode");
		if (sdcard_configClockAndPower(host, SD_FREQ_50M) < 0) {
			return -EIO;
		}
	}
	else {
		TRACE("HS mode not supported");
		if (sdcard_configClockAndPower(host, SD_FREQ_25M) < 0) {
			return -EIO;
		}
	}

	usleep(10);

	/* Perform a final transaction to check if data transfer is working (we only care about retcode this time) */
	if (sdio_cmdSendEx(host, SDIO_ACMD13_SD_STATUS, 0, NULL, false, NULL) < 0) {
		LOG_ERROR("bus speed change failed");
		return -EIO;
	}

	return 0;
}


static int sdcard_startEventISR(sdcard_hostData_t *host, int interruptNum)
{
	if (mutexCreate(&host->eventLock) < 0) {
		return -ENOMEM;
	}

	if (condCreate(&host->eventCond) < 0) {
		resourceDestroy(host->eventLock);
		return -ENOMEM;
	}

	interrupt(interruptNum, sdhost_isr, host, host->eventCond, &host->isrHandle);

	return 0;
}


static void _sdcard_free(sdcard_hostData_t *host)
{
	TRACE("freeing resources");
	sdhost_reset(host, CLOCK_CONTROL_RESET_ALL);
	host->sdioInitialized = false;

	if (host->isrHandle != 0) {
		resourceDestroy(host->isrHandle);
	}

	if (host->cmdLock != 0) {
		resourceDestroy(host->cmdLock);
	}

	if (host->eventCond != 0) {
		resourceDestroy(host->eventCond);
	}

	if (host->eventLock != 0) {
		resourceDestroy(host->eventLock);
	}

	if (host->dmaBuffer != NULL) {
		munmap(host->dmaBuffer, SDCARD_MAX_TRANSFER);
		host->dmaBuffer = NULL;
		host->dmaBufferPhys = (addr_t)NULL;
	}

	*(host->base + SDHOST_REG_INTR_STATUS_ENABLE) = 0;
	*(host->base + SDHOST_REG_INTR_SIGNAL_ENABLE) = 0;
	*(host->base + SDHOST_REG_CLOCK_CONTROL) = 0;
}


void sdcard_free(unsigned int slot)
{
	sdcard_hostData_t *host = sdcard_getHostForSlot(slot);
	if (host == NULL) {
		return;
	}

	if (!host->sdioInitialized) {
		return;
	}

	mutexLock(host->eventLock);
	_sdcard_free(host);
	mutexUnlock(host->eventLock);
	resourceDestroy(host->eventLock);
	host->eventLock = 0;
}


int sdcard_initHost(unsigned int slot)
{
	if (!presenceEvents.initialized) {
		if (mutexCreate(&presenceEvents.lock) < 0) {
			return -ENOMEM;
		}

		if (condCreate(&presenceEvents.cond) < 0) {
			resourceDestroy(presenceEvents.lock);
			return -ENOMEM;
		}

		presenceEvents.initialized = true;
	}

	sdcard_hostData_t *host = sdcard_getHostForSlot(slot);
	if (host == NULL) {
		return -ENOENT;
	}

	if (host->sdioInitialized) {
		return (initializedHosts < PLATFORM_SDIO_N_HOSTS) ?
			(PLATFORM_SDIO_N_HOSTS - initializedHosts) :
			0;
	}

	host->cmdLock = 0;
	host->eventCond = 0;
	host->eventLock = 0;
	host->isrHandle = 0;

	/* Perform platform-specific configuration */
	sdio_platformInfo_t info;
	if (sdio_platformConfigure(slot, &info) < 0) {
		return -EIO;
	}

	host->refclkFrequency = info.refclkFrequency;
	host->isCDPinSupported = info.isCDPinSupported;
	host->isWPPinSupported = info.isWPPinSupported;

	/* Map register bank into our virtual memory */
	void *ptr = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, info.regBankPhys);
	if (ptr == MAP_FAILED) {
		_sdcard_free(host);
		return -EIO;
	}

	host->base = ptr;
	if (sdhost_allocDMA(host) < 0) {
		_sdcard_free(host);
		return -ENOMEM;
	}

	if (mutexCreate(&host->cmdLock) < 0) {
		_sdcard_free(host);
		return -ENOMEM;
	}

	if (sdhost_reset(host, CLOCK_CONTROL_RESET_ALL) < 0) {
		_sdcard_free(host);
		return -EIO;
	}

	if (sdcard_configClockAndPower(host, SD_FREQ_INITIAL) < 0) {
		_sdcard_free(host);
		return -EIO;
	}

	if (!host->isCDPinSupported) {
		/* Spoof card state so that it always appears inserted */
		*(host->base + SDHOST_REG_HOST_CONTROL) |= HOST_CONTROL_CARD_DET_TEST | HOST_CONTROL_CARD_DET_TEST_ENABLE;
	}

	*(host->base + SDHOST_REG_INTR_STATUS_ENABLE) = SDHOST_STATUS_MASK;
	if (sdcard_startEventISR(host, info.interruptNum) < 0) {
		_sdcard_free(host);
		return -ENOMEM;
	}

	host->sdioInitialized = true;
	initializedHosts++;
	return (initializedHosts < PLATFORM_SDIO_N_HOSTS) ?
		(PLATFORM_SDIO_N_HOSTS - initializedHosts) :
		0;
}


/* Calculate the divisor value so the output frequency is no larger than `freq` */
static int sdcard_calculateDivisor(uint32_t refclk, uint32_t freq, uint32_t *divisor)
{
	if (freq >= refclk) {
		*divisor = CLOCK_CONTROL_DIV_1;
		return 0;
	}

	for (uint32_t val = CLOCK_CONTROL_DIV_2; val <= CLOCK_CONTROL_DIV_256; val <<= 1) {
		refclk /= 2;
		if (freq >= refclk) {
			*divisor = val;
			return 0;
		}
	}

	return -1;
}


static int sdcard_configClockAndPower(sdcard_hostData_t *host, uint32_t freq)
{
	if ((freq != SD_FREQ_INITIAL) && (freq != SD_FREQ_25M) && (freq != SD_FREQ_50M)) {
		return -EINVAL;
	}

	/* NOTE: In theory, the Capabilities register can hold the reference clock frequency,
	 * but this doesn't have to be implemented. For this reason, we get the frequency
	 * from sdio_platformConfigure().
	 */
	uint32_t divRegValue;
	if (sdcard_calculateDivisor(host->refclkFrequency, freq, &divRegValue) < 0) {
		return -EINVAL;
	}

	*(host->base + SDHOST_REG_CLOCK_CONTROL) &= ~CLOCK_CONTROL_START_SD_CLOCK;

	if (freq == SD_FREQ_50M) {
		*(host->base + SDHOST_REG_HOST_CONTROL) |= HOST_CONTROL_HIGH_SPEED;
	}
	else {
		*(host->base + SDHOST_REG_HOST_CONTROL) &= ~HOST_CONTROL_HIGH_SPEED;
	}

	/* This looks weird because we may set the "divisor" to 0, but this is intended */
	*(host->base + SDHOST_REG_CLOCK_CONTROL) = divRegValue | CLOCK_CONTROL_START_INTERNAL_CLOCK;
	sdio_dataBarrier();
	for (int i = 0; i < SDHOST_RETRIES; i++) {
		uint32_t val = *(host->base + SDHOST_REG_CLOCK_CONTROL) & CLOCK_CONTROL_INTERNAL_CLOCK_STABLE;
		if (val != 0) {
			*(host->base + SDHOST_REG_CLOCK_CONTROL) |= CLOCK_CONTROL_START_SD_CLOCK | CLOCK_CONTROL_DATA_TIMEOUT_VALUE(0b1110UL);
			*(host->base + SDHOST_REG_HOST_CONTROL) |= HOST_CONTROL_BUS_VOLTAGE_3V3 | HOST_CONTROL_BUS_POWER;
			return 0;
		}

		usleep(10);
	}

	return -ETIME;
}


static int _sdcard_transferBlocks(sdcard_hostData_t *host, sdio_dir_t dir, uint32_t blockOffset, void *data, size_t len)
{
	uint8_t cmd;

	if (dir == sdio_write) {
		memcpy(host->dmaBuffer, data, len);
	}

	uint16_t blockCount = len / SDCARD_BLOCKLEN;
	if (dir == sdio_read) {
		if (blockCount > 1) {
			cmd = SDIO_CMD18_READ_MULTIPLE_BLOCK;
		}
		else {
			cmd = SDIO_CMD17_READ_SINGLE_BLOCK;
		}
	}
	else {
		if (blockCount > 1) {
			cmd = SDIO_CMD25_WRITE_MULTIPLE_BLOCK;
		}
		else {
			cmd = SDIO_CMD24_WRITE_SINGLE_BLOCK;
		}
	}

	/* The unit of “data address” in argument is byte for Standard Capacity SD Memory Card
	 * and block (512 bytes) for High Capacity SD Memory Card.
	 */
	uint32_t arg = host->card.highCapacity ? blockOffset : (blockOffset * SDCARD_BLOCKLEN);
	uint32_t resp;
	int ret = _sdio_cmdSend(host, cmd, arg, &resp, blockCount, false);

	if (ret < 0) {
		return ret;
	}

	if (dir == sdio_read) {
		memcpy(data, host->dmaBuffer, len);
	}

	if ((resp & CARD_STATUS_ERRORS) != 0) {
		LOG_ERROR("transfer error %08x", resp);
		return -EIO;
	}

	return 0;
}


int sdcard_transferBlocks(unsigned int slot, sdio_dir_t dir, uint32_t blockOffset, void *data, size_t len)
{
	sdcard_hostData_t *host = sdcard_getHostForSlot(slot);
	if (host == NULL) {
		return -ENOENT;
	}

	if ((len % SDCARD_BLOCKLEN != 0) || (len > SDCARD_MAX_TRANSFER)) {
		return -EINVAL;
	}

	if ((dir == sdio_write) && sdcard_isWriteProtected(host)) {
		return -EPERM;
	}

	mutexLock(host->cmdLock);
	int ret = _sdcard_transferBlocks(host, dir, blockOffset, data, len);
	mutexUnlock(host->cmdLock);

	return ret;
}


static int _sdcard_eraseBlocks(sdcard_hostData_t *host, uint32_t start, uint32_t end)
{
	uint32_t resp;
	int ret;

	ret = _sdio_cmdSend(host, SDIO_CMD32_ERASE_WR_BLK_START, start, &resp, 0, false);
	if ((ret < 0) || ((resp & CARD_STATUS_ERRORS) != 0)) {
		LOG_ERROR("erase start %d %08x", ret, resp);
		return -EIO;
	}

	ret = _sdio_cmdSend(host, SDIO_CMD33_ERASE_WR_BLK_END, end, &resp, 0, false);
	if ((ret < 0) || ((resp & CARD_STATUS_ERRORS) != 0)) {
		LOG_ERROR("erase end %d %08x", ret, resp);
		return -EIO;
	}

	ret = _sdio_cmdSend(host, SDIO_CMD38_ERASE, 0, &resp, 0, false);
	if ((ret < 0) || ((resp & CARD_STATUS_ERRORS) != 0)) {
		LOG_ERROR("do erase %d %08x", ret, resp);
		return -EIO;
	}

	return ret;
}


int sdcard_eraseBlocks(unsigned int slot, uint32_t blockOffset, uint32_t nBlocks)
{
	int ret = EOK;

	sdcard_hostData_t *host = sdcard_getHostForSlot(slot);
	if (host == NULL) {
		return -ENOENT;
	}

	if (sdcard_isWriteProtected(host)) {
		return -EPERM;
	}

	mutexLock(host->cmdLock);
	if ((blockOffset % host->card.eraseSizeBlocks != 0) || (nBlocks % host->card.eraseSizeBlocks != 0)) {
		mutexUnlock(host->cmdLock);
		return -EINVAL;
	}

	uint32_t erasePerIteration = (host->card.eraseSizeBlocks > ERASE_N_BLOCKS) ?
		host->card.eraseSizeBlocks :
		ERASE_N_BLOCKS;
	while (nBlocks > 0) {
		if (nBlocks < erasePerIteration) {
			erasePerIteration = nBlocks;
		}

		uint32_t start = blockOffset;
		uint32_t end = blockOffset + erasePerIteration - 1;
		if (!host->card.highCapacity) {
			start *= SDCARD_BLOCKLEN;
			end *= SDCARD_BLOCKLEN;
		}

		ret = _sdcard_eraseBlocks(host, start, end);
		if (ret < 0) {
			break;
		}

		blockOffset += erasePerIteration;
		nBlocks -= erasePerIteration;
	}

	mutexUnlock(host->cmdLock);

	return ret;
}


int sdcard_writeFF(unsigned int slot, uint32_t blockOffset, uint32_t nBlocks)
{
	int ret = EOK;

	sdcard_hostData_t *host = sdcard_getHostForSlot(slot);
	if (host == NULL) {
		return -ENOENT;
	}

	if (sdcard_isWriteProtected(host)) {
		return -EPERM;
	}

	mutexLock(host->cmdLock);
	uint32_t erasePerIteration = SDCARD_MAX_TRANSFER / SDCARD_BLOCKLEN;
	memset(host->dmaBuffer, 0xff, SDCARD_MAX_TRANSFER);
	while (nBlocks > 0) {
		if (nBlocks < erasePerIteration) {
			erasePerIteration = nBlocks;
		}

		uint8_t cmd = (erasePerIteration > 1) ? SDIO_CMD25_WRITE_MULTIPLE_BLOCK : SDIO_CMD24_WRITE_SINGLE_BLOCK;
		uint32_t arg = host->card.highCapacity ? blockOffset : (blockOffset * SDCARD_BLOCKLEN);
		uint32_t resp;
		ret = _sdio_cmdSend(host, cmd, arg, &resp, erasePerIteration, false);
		if (ret < 0) {
			break;
		}

		if ((resp & CARD_STATUS_ERRORS) != 0) {
			LOG_ERROR("transfer error %08x", resp);
			ret = -EIO;
			break;
		}

		blockOffset += erasePerIteration;
		nBlocks -= erasePerIteration;
	}

	mutexUnlock(host->cmdLock);

	return ret;
}


uint32_t sdcard_getSizeBlocks(unsigned int slot)
{
	sdcard_hostData_t *host = sdcard_getHostForSlot(slot);
	if (host == NULL) {
		return 0;
	}

	return host->card.sizeBlocks;
}


uint32_t sdcard_getEraseSizeBlocks(unsigned int slot)
{
	sdcard_hostData_t *host = sdcard_getHostForSlot(slot);
	if (host == NULL) {
		return 0;
	}

	return host->card.eraseSizeBlocks;
}


sdcard_insertion_t sdcard_isInserted(unsigned int slot)
{
	sdcard_hostData_t *host = sdcard_getHostForSlot(slot);
	if (host == NULL) {
		return 0;
	}

	uint32_t val = *(host->base + SDHOST_REG_PRES_STATE);
	if ((val & PRES_STATE_CARD_DET_STABLE) == 0) {
		return SDCARD_INSERTION_UNSTABLE;
	}

	return ((val & PRES_STATE_CARD_INSERTED) != 0) ? SDCARD_INSERTION_IN : SDCARD_INSERTION_OUT;
}


void sdcard_handlePresence(sdcard_event_handler_t onInsert, sdcard_event_handler_t onRemove)
{
	int ret;
	for (unsigned int i = 0; i < PLATFORM_SDIO_N_HOSTS; i++) {
		sdcard_insertion_t state = sdcard_isInserted(i);
		if (state != SDCARD_INSERTION_UNSTABLE) {
			*(sdio_hosts[i].base + SDHOST_REG_INTR_STATUS) = SDHOST_INTR_CARD_OUT | SDHOST_INTR_CARD_IN;
		}

		if (state == SDCARD_INSERTION_OUT) {
			if (onRemove != NULL) {
				ret = onRemove(i);
				if (ret < 0) {
					TRACE("Card eject callback returned %d", ret);
				}
			}
		}
		else if (state == SDCARD_INSERTION_IN) {
			if (onInsert != NULL) {
				ret = onInsert(i);
				if (ret < 0) {
					TRACE("Card insert callback returned %d", ret);
				}
			}
		}
	}
}


void sdcard_presenceThread(sdcard_event_handler_t onInsert, sdcard_event_handler_t onRemove)
{
	int ret;
	if (!presenceEvents.initialized) {
		return;
	}

	for (;;) {
		mutexLock(presenceEvents.lock);
		for (unsigned int i = 0; i < PLATFORM_SDIO_N_HOSTS; i++) {
			uint32_t val = *(sdio_hosts[i].base + SDHOST_REG_INTR_STATUS);
			sdio_dataBarrier();
			*(sdio_hosts[i].base + SDHOST_REG_INTR_STATUS) = SDHOST_INTR_CARD_OUT | SDHOST_INTR_CARD_IN;
			if ((val & SDHOST_INTR_CARD_OUT) != 0) {
				if (onRemove != NULL) {
					ret = onRemove(i);
					if (ret < 0) {
						TRACE("Card eject callback returned %d", ret);
					}
				}

				TRACE("Card ejected");
			}

			if ((val & SDHOST_INTR_CARD_IN) != 0) {
				if (onInsert != NULL) {
					ret = onInsert(i);
					if (ret < 0) {
						TRACE("Card insert callback returned %d", ret);
					}
				}

				TRACE("Card inserted");
			}
		}

		condWait(presenceEvents.cond, presenceEvents.lock, 1 * 1000 * 1000);
		mutexUnlock(presenceEvents.lock);
	}
}
