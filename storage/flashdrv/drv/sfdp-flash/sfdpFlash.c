/*
 * Phoenix-RTOS
 *
 * Flash driver for flash devices utilizing SFDP (Serial Flash Discoverable Parameters) table,
 * according to JEDEC Standard JESD216
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski, Amelia Waszkowska
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/time.h>

#include "../commands/flash_cmds.h"
#include "../grlib-spimctrl/flashdrv.h"
#include "sfdpFlash.h"


/* clang-format off */
#define FLASH_ID(vid, pid) ( (((pid) & 0xffu) << 16) | ((pid) & 0xff00u) | ((vid) & 0xffu))


enum { write_disable = 0, write_enable };


static const char *nor_vendors[] = {
	"\xef" " Winbond",
	"\x20" " Micron",
	"\x9d" " ISSI",
	"\xc2" " Macronix",
	NULL
};
/* clang-format on */

// #define CFI_TIMEOUT_MAX_PROGRAM(typical, maximum) ((1u << (typical)) * (1u << (maximum)))
// #define CFI_TIMEOUT_MAX_ERASE(typical, maximum)   ((1u << (typical)) * (1u << (maximum)) * 1024u)
// TO DOOOOOO !!!

static const struct nor_info flashInfo[] = {
	/* Macronix (MXIX) */
	{ FLASH_ID(0xc2u, 0x2019u), "MX25L25635F", 32 * 1024 * 1024, 0x100, 0x1000, 0x10000, 2, 120, 1000, 150 * 1000, 1 },
    /* Micron */
    { FLASH_ID(0x20u, 0xBB21u), "MT25QU01GB", 64 * 1024 * 1024, 0x100, 0x1000, 0x10000, 1, 300, 1000, 250 * 1000, 2 }
};

int activeDeviceIdx = -1;

static struct {
	void *base;
} common;


static int nor_readId(struct spimctrl *spimctrl, uint32_t *id)
{
	struct xferOp xfer;
	const uint8_t cmd = FLASH_CMD_RDID;

	xfer.type = xfer_opRead;
	xfer.cmd = &cmd;
	xfer.cmdLen = 1;
	xfer.rxData = (uint8_t *)id;
	xfer.dataLen = 3;

	return spimctrl_xfer(spimctrl, &xfer);
}


static int nor_readStatus(struct spimctrl *spimctrl, uint8_t *status)
{
	struct xferOp xfer;
	const uint8_t cmd = FLASH_CMD_RDSR;

	xfer.type = xfer_opRead;
	xfer.cmd = &cmd;
	xfer.cmdLen = 1;
	xfer.rxData = status;
	xfer.dataLen = 1;

	return spimctrl_xfer(spimctrl, &xfer);
}


static int nor_writeEnable(struct spimctrl *spimctrl, int enable)
{
	int res;
	struct xferOp xfer;
	uint8_t status = 0;
	const uint8_t cmd = (enable == 1) ? FLASH_CMD_WREN : FLASH_CMD_WRDI;

	res = nor_waitBusy(spimctrl, 0);
	if (res < 0) {
		return res;
	}

	xfer.type = xfer_opWrite;
	xfer.cmd = &cmd;
	xfer.cmdLen = 1;
	xfer.txData = NULL;
	xfer.dataLen = 0;

	res = spimctrl_xfer(spimctrl, &xfer);
	if (res < EOK) {
		return res;
	}

	res = nor_readStatus(spimctrl, &status);
	if (res < EOK) {
		return res;
	}

	status = (status & FLASH_SR_WEL) ? 1 : 0;

	if (status != enable) {
		return -EIO;
	}

	return EOK;
}


static int nor_enter4Byte(struct spimctrl *spimctrl)
{
	int res;
	struct xferOp xfer;
	const uint8_t cmd = FLASH_CMD_ENTER_4B;

	xfer.type = xfer_opWrite;
	xfer.cmd = &cmd;
	xfer.cmdLen = 1;
	xfer.txData = NULL;
	xfer.rxData = NULL;
	xfer.dataLen = 0;

	res = spimctrl_xfer(spimctrl, &xfer);
	if (res < EOK) {
		return res;
	}

	spimctrl->extendedAddress = 1;

	return EOK;
}


static int nor_readEAR(struct spimctrl *spimctrl, uint8_t *status)
{
	struct xferOp xfer;
	const uint8_t cmd = FLASH_CMD_RDEAR;

	xfer.type = xfer_opRead;
	xfer.cmd = &cmd;
	xfer.cmdLen = 1;
	xfer.rxData = status;
	xfer.dataLen = 1;

	return spimctrl_xfer(spimctrl, &xfer);
}


static int nor_writeEAR(struct spimctrl *spimctrl, uint8_t value)
{
	int res;
	struct xferOp xfer;
	const uint8_t cmd = FLASH_CMD_WREAR;

	nor_writeEnable(spimctrl, write_enable);

	xfer.type = xfer_opWrite;
	xfer.cmd = &cmd;
	xfer.cmdLen = 1;
	xfer.txData = &value;
	xfer.dataLen = 1;

	res = spimctrl_xfer(spimctrl, &xfer);
	if (res < EOK) {
		return res;
	}

	res = nor_readEAR(spimctrl, &spimctrl->ear);
	if (res < EOK) {
		return res;
	}

	if (spimctrl->ear != value) {
		return -EIO;
	}

	return EOK;
}


static int nor_validateEar(struct spimctrl *spimctrl, uint32_t addr)
{
	int res = EOK;
	const uint8_t desiredEar = (addr >> 24) & 0xffu;

	if (desiredEar != spimctrl->ear) {
		res = nor_writeEAR(spimctrl, desiredEar);
	}
	return res;
}


int nor_waitBusy(struct spimctrl *spimctrl, time_t timeout)
{
	int res;
	uint8_t status = 0;
	time_t now, end;
	(void)gettime(&end, NULL);

	end += timeout * 1000;

	do {
		res = nor_readStatus(spimctrl, &status);
		if (res < EOK) {
			return res;
		}

		(void)gettime(&now, NULL);
		if ((timeout > 0) && (now > end)) {
			return -ETIME;
		}
	} while ((status & FLASH_SR_WIP) != 0);

	return EOK;
}


int nor_eraseDie(struct spimctrl *spimctrl, time_t timeout, uint8_t selDie)
{
    int res;
	uint8_t cmd[5];
	struct xferOp xfer;

    addr_t addr = 0x00000000u;
    addr_t addr0 = 0x00000000u;
    addr_t addr1 = 0x04000000u;

    if (flashInfo[activeDeviceIdx].stacked == 1) {
        res = -EINVAL;
        return res;
    }

    if (selDie == 0) {
        addr = addr0;
    }
    else if (selDie == 1) {
        addr = addr1;
    }

    if (!spimctrl->extendedAddress) {

		cmd[0] = FLASH_CMD_DE;
        cmd[1] = (addr >> 16) & 0xff;
        cmd[2] = (addr >> 8) & 0xff;
        cmd[3] = addr & 0xff;

        res = nor_validateEar(spimctrl, addr);
        if (res < EOK) {
            return res;
        }

        xfer.cmd = cmd;
		xfer.cmdLen = 4; 
    }
    else {

		cmd[0] = FLASH_CMD_DE;
        cmd[1] = (addr >> 24) & 0xff;
        cmd[2] = (addr >> 16) & 0xff;
        cmd[3] = (addr >> 8) & 0xff;
        cmd[4] = addr & 0xff;

        xfer.cmd = cmd;
		xfer.cmdLen = 5;
    }

    res = nor_writeEnable(spimctrl, write_enable);
    if (res < EOK) {
        return res;
    }

    xfer.type = xfer_opWrite;
    xfer.txData = NULL;
    xfer.dataLen = 0;

    res = spimctrl_xfer(spimctrl, &xfer);
	if (res < EOK) {
		return res;
	}

	return nor_waitBusy(spimctrl, timeout);
}


// bool nor_isBlank(const struct _storage_devCtx_t *ctx, addr_t addr, size_t size)
// {
//     uint8_t buf[BLANK_CHECK_BUF_SIZE];
//     size_t bytesLeft = size;
//     addr_t currentAddr = addr;

//     if (size == 0) {
//         return false;
//     }

//     while (bytesLeft > 0) {
//         size_t chunkSize = (bytesLeft < BLANK_CHECK_BUF_SIZE) ? bytesLeft : BLANK_CHECK_BUF_SIZE;

//         /* Odczyt fragmentu pamięci przez standardową funkcję odczytu sterownika */
//         if (flashsrv_read((storage_t *)ctx, currentAddr, buf, chunkSize) != chunkSize) {
//             return false; /* Błąd odczytu z SPI - dla bezpieczeństwa traktujemy jako nie-czysty */
//         }

//         /* Sprawdzenie pierwszego bajtu i szybkie porównanie całego bufora */
//         if (buf[0] != 0xFF || memcmp(buf, buf + 1, chunkSize - 1) != 0) {
//             return false; /* Znaleziono bajt inny niż 0xFF */
//         }

//         bytesLeft -= chunkSize;
//         currentAddr += chunkSize;
//     }

//     return true; /* Cały obszar to 0xFF */
// }


int nor_eraseChip(struct spimctrl *spimctrl, time_t timeout)
{
	int res = ENODEV;
	struct xferOp xfer;

    if(flashInfo[activeDeviceIdx].stacked == 1) {
        const uint8_t cmd = FLASH_CMD_CE;

        res = nor_writeEnable(spimctrl, write_enable);
        if (res < EOK) {
            return res;
        }

        xfer.type = xfer_opWrite;
        xfer.cmd = &cmd;
        xfer.cmdLen = 1;
        xfer.txData = NULL;
        xfer.dataLen = 0;

        res = spimctrl_xfer(spimctrl, &xfer);
        if (res < EOK) {
            return res;
        }

        res = nor_waitBusy(spimctrl, timeout);
    }
    else if (flashInfo[activeDeviceIdx].stacked == 2) {
        res = nor_eraseDie(spimctrl, timeout, 0);
        res = nor_eraseDie(spimctrl, timeout, 1);
    }

    return res;
}


int nor_eraseSubSector(struct spimctrl *spimctrl, addr_t addr, time_t timeout)
{
	int res;
	uint8_t cmd[5];
	struct xferOp xfer;

    if(!spimctrl->extendedAddress) {

		cmd[0] = FLASH_CMD_SE;
        cmd[1] = (addr >> 16) & 0xff;
        cmd[2] = (addr >> 8) & 0xff;
        cmd[3] = addr & 0xff;

        res = nor_validateEar(spimctrl, addr);
        if (res < EOK) {
            return res;
        }

        xfer.cmd = cmd;
	    xfer.cmdLen = 4;
    }
    else {

		cmd[0] = FLASH_CMD_4B_SE;
        cmd[1] = (addr >> 24) & 0xff;
        cmd[2] = (addr >> 16) & 0xff;
        cmd[3] = (addr >> 8) & 0xff;
        cmd[4] = addr & 0xff;

		xfer.cmd = cmd;
		xfer.cmdLen = 5;
    }
	
	res = nor_writeEnable(spimctrl, write_enable);
	if (res < EOK) {
		return res;
	}

	xfer.type = xfer_opWrite;
	xfer.txData = NULL;
	xfer.dataLen = 0;

	res = spimctrl_xfer(spimctrl, &xfer);
	if (res < EOK) {
		return res;
	}

	res = nor_waitBusy(spimctrl, timeout);
	return res;
}


int nor_eraseSector(struct spimctrl *spimctrl, addr_t addr, time_t timeout)
{
	int res;
	uint8_t cmd[5];
	struct xferOp xfer;

    if(!spimctrl->extendedAddress) {

		cmd[0] = FLASH_CMD_BE;
        cmd[1] = (addr >> 16) & 0xff;
        cmd[2] = (addr >> 8) & 0xff;
        cmd[3] = addr & 0xff;
		
        res = nor_validateEar(spimctrl, addr);
        if (res < EOK) {
            return res;
        }

        xfer.cmd = cmd;
	    xfer.cmdLen = 4;
    }
    else {
	
		cmd[0] = FLASH_CMD_4B_BE;
        cmd[1] = (addr >> 24) & 0xff;
        cmd[2] = (addr >> 16) & 0xff;
        cmd[3] = (addr >> 8) & 0xff;
        cmd[4] = addr & 0xff;

		xfer.cmd = cmd;
		xfer.cmdLen = 5;
		//printf("erasing \n");
    }
	
	res = nor_writeEnable(spimctrl, write_enable);
	if (res < EOK) {
		return res;
	}

	xfer.type = xfer_opWrite;
	xfer.txData = NULL;
	xfer.dataLen = 0;

	res = spimctrl_xfer(spimctrl, &xfer);
	if (res < EOK) {
		return res;
	}

	res = nor_waitBusy(spimctrl, timeout);
	return res;
}


int nor_pageProgram(struct spimctrl *spimctrl, addr_t addr, const void *src, size_t len, time_t timeout)
{
	struct xferOp xfer;
	uint8_t cmd[5];
    int res = 0;

    if(!spimctrl->extendedAddress) {

		cmd[0] = FLASH_CMD_PP;
        cmd[1] = (addr >> 16) & 0xff;
        cmd[2] = (addr >> 8) & 0xff;
        cmd[3] = addr & 0xff;

		int res = nor_validateEar(spimctrl, addr);
		if (res < 0) {
			return res;
		}

		xfer.cmd = cmd;
		xfer.cmdLen = 4;
	}
	else {
		cmd[0] = FLASH_CMD_4B_PP;
        cmd[1] = (addr >> 24) & 0xff;
        cmd[2] = (addr >> 16) & 0xff;
        cmd[3] = (addr >> 8) & 0xff;
        cmd[4] = addr & 0xff;

		xfer.cmd = cmd;
		xfer.cmdLen = 5;
	}

	res = nor_writeEnable(spimctrl, write_enable);
	if (res < EOK) {
		return res;
	}

	xfer.type = xfer_opWrite;
	xfer.txData = src;
	xfer.dataLen = len;

	res = spimctrl_xfer(spimctrl, &xfer);
	if (res < EOK) {
		return res;
	}

	res = nor_waitBusy(spimctrl, timeout);

	return res;
}


static ssize_t nor_readCmd(struct spimctrl *spimctrl, addr_t addr, void *data, size_t size)
{
    struct xferOp xfer;
    uint8_t cmd[5]; // Zadeklarowane w zasięgu całej funkcji!
    int res = 0;

    if (!spimctrl->extendedAddress) {
        res = nor_validateEar(spimctrl, addr);
        if (res < 0) {
            return res;
        }

        cmd[0] = FLASH_CMD_READ;
        cmd[1] = (addr >> 16) & 0xff;
        cmd[2] = (addr >> 8) & 0xff;
        cmd[3] = addr & 0xff;

        xfer.cmd = cmd;
        xfer.cmdLen = 4;
    }
    else {
        cmd[0] = FLASH_CMD_4B_READ;
        cmd[1] = (addr >> 24) & 0xff;
        cmd[2] = (addr >> 16) & 0xff;
        cmd[3] = (addr >> 8) & 0xff;
        cmd[4] = addr & 0xff;

        xfer.cmd = cmd;
        xfer.cmdLen = 5;
    }

    xfer.type = xfer_opRead;
    xfer.rxData = data;
    xfer.dataLen = size;

    res = spimctrl_xfer(spimctrl, &xfer);

    return res < EOK ? res : (ssize_t)size;
}


static ssize_t nor_readAhb(struct spimctrl *spimctrl, addr_t addr, void *data, size_t size)
{

    if (!spimctrl->extendedAddress) {
        int res = nor_validateEar(spimctrl, addr);
        if (res < EOK) {
            return res;
        }
    }

	(void)memcpy(data, (uint8_t *)common.base + addr, size);

	return (ssize_t)size;
}


ssize_t nor_readData(struct spimctrl *spimctrl, addr_t addr, void *data, size_t size)
{
    if (!spimctrl->extendedAddress) {
        if (((addr & 0xff000000) == 0) && (((addr + size) & 0xff000000) != 0)) {
            /* If we'd have to change EAR register during read,
            * read data through command (can be read without EAR change)
            */
            return nor_readCmd(spimctrl, addr, data, size);
        }
        else {
            /* Direct copy */
            return nor_readAhb(spimctrl, addr, data, size);
        }
    }
    else {
        return nor_readAhb(spimctrl, addr, data, size);
		//return nor_readCmd(spimctrl, addr, data, size);
    }
}


static int nor_probe(struct _storage_devCtx_t *ctx)
{
	int res;
	uint32_t jedecId = 0;

	res = nor_readId(ctx->spimctrl, &jedecId);
	if (res < EOK) {
		return res;
	}

    if (!(ctx->spimctrl->extendedAddress)) {
        res = nor_readEAR(ctx->spimctrl, &ctx->spimctrl->ear);
        if (res < EOK) {
            return res;
        }
    }

	res = -ENXIO;
	for (size_t i = 0; i < sizeof(flashInfo) / sizeof(flashInfo[0]); ++i) {
		if ((flashInfo[i].jedecId & 0x00FFFFFFu) == (jedecId & 0x00FFFFFFu)) {
			ctx->flash_data.sfdp = &flashInfo[i];
            activeDeviceIdx = i;
			res = EOK;
			break;
		}
	}

	if (res != EOK) {
		return res;
	}

	return EOK;
}


void nor_printInfo(const struct _storage_devCtx_t *ctx)
{
	int res;
	uint32_t jedecId = 0;

	res = nor_readId(ctx->spimctrl, &jedecId);
	if (res < EOK) {
		return;
	}

	const char *pVendor = "Unknown";

	for (size_t i = 0; nor_vendors[i]; ++i) {
		if (*(uint8_t *)nor_vendors[i] == (jedecId >> 16)) {
			pVendor = &nor_vendors[i][2];
			break;
		}
	}
	
	(void)printf("gr765-flashdrv: detected %s %s (0x%x)\n", pVendor, ctx->flash_data.sfdp->name, ctx->flash_data.sfdp->jedecId);
}


int nor_flash_init(struct _storage_devCtx_t *ctx, addr_t flashBase)
{
	int res;

	res = nor_probe(ctx);
	if (res != EOK) {
		return res;
	}

	/* Map entire flash */
	common.base = mmap(NULL, ((ctx->flash_data.sfdp->totalSz) * (ctx->flash_data.sfdp->stacked)), PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, flashBase);
	if (common.base == MAP_FAILED) {
		LOG_ERROR("failed to map flash");
		return -ENOMEM;
	}

	nor_enter4Byte(ctx->spimctrl);

	return EOK;
}


void nor_destroy(struct _storage_devCtx_t *ctx)
{
	spimctrl_destroy(ctx->spimctrl);
	(void)munmap(common.base, ((ctx->flash_data.sfdp->totalSz) * (ctx->flash_data.sfdp->stacked)));
}