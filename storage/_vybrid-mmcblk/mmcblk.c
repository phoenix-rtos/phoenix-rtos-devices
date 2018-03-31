#include <dev/mmcblk/mmcblk.h>
#include <dev/mmcblk/mmcblk_priv.h>

#include <dev/mmcblk/mmcblk_ce_ata.h>
#include <dev/mmcblk/mmcblk_mmc.h>
#include <dev/mmcblk/mmcblk_sd.h>
#include <dev/mmcblk/mmcblk_sdio.h>

#include <vm/if.h>

static MmcblkCard_t *card;

static int mmcblkCD(unsigned line, void *arg);


static struct MmcblkCardOps_t cardOps[eCardTypeNum] = {
	{
		.init = NULL,
		.deinit = NULL,
		.inserted = NULL,
		.switchHighSpeed = NULL,
		.write = NULL,
		.read = NULL
	},
	{
		.init = mmcblk_sd_init,
		.deinit = mmcblk_sd_deinit,
		.inserted = mmcblk_sd_inserted,
		.switchHighSpeed = mmcblk_sd_switchHighSpeed,
		.write = mmcblk_sd_write,
		.read = mmcblk_sd_read
	},
	{
		.init = mmcblk_sdio_init,
		.deinit = mmcblk_sdio_deinit,
		.inserted = mmcblk_sdio_inserted,
		.switchHighSpeed = mmcblk_sdio_switchHighSpeed,
		.write = mmcblk_sdio_write,
		.read = mmcblk_sdio_read
	},
	{
		.init = mmcblk_mmc_init,
		.deinit = mmcblk_mmc_deinit,
		.inserted = mmcblk_mmc_inserted,
		.switchHighSpeed = mmcblk_mmc_switchHighSpeed,
		.write = mmcblk_mmc_write,
		.read = mmcblk_mmc_read
	},
	{
		.init = mmcblk_ce_ata_init,
		.deinit = mmcblk_ce_ata_deinit,
		.inserted = mmcblk_ce_ata_inserted,
		.switchHighSpeed = mmcblk_ce_ata_switchHighSpeed,
		.write = mmcblk_ce_ata_write,
		.read = mmcblk_ce_ata_read
	},
	{
		.init = NULL,
		.deinit = NULL,
		.inserted = NULL,
		.switchHighSpeed = NULL,
		.write = NULL,
		.read = NULL
	}
};



int mmcblk_evaluateResponse(MmcblkResponse_t *response)
{
	int status=EOK;
	if(response == NULL)
	{
		status=EOK;
		return status;
	}
	if(response->timeout)
	{
		status=-ETIMEDOUT;
		return status;
	}
	else if(response->error)
	{
		status = -EFAULT;
		return status;
	}
	else if(response->busy)
	{
		status = -EBUSY;
		return status;
	}

	switch(response->responseType)
	{
		case MMCBLK_COMM_RESPONSE_R1:
			if(response->response.r1.bits.CARD_IS_LOCKED)
				LOG("CARD IS LOCKED!");
			if(response->response.r1.response & 0xFFFC0000)
			{
				LOG("FAULTY R1 RESPONSE: %x", response->response.r1.response );
				status = -EFAULT;
			}
		break;
		case MMCBLK_COMM_RESPONSE_NO_RESPONSE:
		case MMCBLK_COMM_RESPONSE_R1b:
		case MMCBLK_COMM_RESPONSE_R2:
		case MMCBLK_COMM_RESPONSE_R3:
		case MMCBLK_COMM_RESPONSE_R4:
		case MMCBLK_COMM_RESPONSE_R5:
		case MMCBLK_COMM_RESPONSE_R5b:
		case MMCBLK_COMM_RESPONSE_R6:
		break;
	}
	return status;
}


static int cardIdentify(MmcblkCard_t *card){
	MmcblkResponse_t response;
	MmcblkOCRReg_t ocr;
	LOG("Identifying card...");

	ocr = 0;
	ocr |= MMCBLK_OCR_V28_V29;
	ocr |= MMCBLK_OCR_V29_V30;
	ocr |= MMCBLK_OCR_V30_V31;

	card->label = eCardUnknown;
	/* CMD5, check SDIO */
	/* operation voltage, command argument is zero */

	{
		//0xDC is test pattern
		u32 arg = (1 << 8) | 0xDC;
		card->port->ioOps.sendCommand(card, MMCBLK_COMM_SEND_IF_COND, arg, 0, 0, NULL);
		response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_SEND_IF_COND);
		if(response.timeout || response.busy)
		{
			LOG("Old card");
		}
		else
		{
			LOG("New card");

			if(response.response.r7.response == arg)
			{
				LOG("Pattern match");
			}
			else
			{
				LOG("Pattern does not match! %x", response.response.r7.response);
				return -EFAULT;
			}
			ocr |= MMCBLK_OCR_CCS;
		}
	}
/*
	card->port->ioOps.sendCommand(card, MMCBLK_COMM_IO_SEND_OP_COND, 0, 0, 0, NULL);
	response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_IO_SEND_OP_COND);
	if (!response.timeout && !response.error) {
		if( ((response.response.r4.ocr >> 28) & 0x07) && (response.response.r4.ocr & 0x300000) )
		{
			int ov = 0;
			do
			{
				proc_threadSleep(100);
				++ov;
				card->port->ioOps.sendCommand(card, MMCBLK_COMM_IO_SEND_OP_COND, 0x300000, 0, 0, NULL);
				response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_IO_SEND_OP_COND);
				if(response.error || response.busy)
					return -EFAULT;
			}
			while(!!(response.response.r4.ocr & 0x88000000) && (ov <= 255));
			if(response.response.r4.ocr & 0x80000000)
			{
				card->label = eCardSDIO;
			}
			if(response.response.r4.ocr & 0x08000000)
			{
				if(card->label == eCardSDIO)
					card->label = eCardSDCombo;
				else
					card->label = eCardSD;
			}
		}
	}
	else
	{
		card->port->ioOps.reset(card);
	}
*/
	if(card->label == eCardSDIO)
		return EOK;

	/* FROM SD SPECIFICATION https://www.sdcard.org/downloads/pls/simplified_specs/part1_410.pdf
	  ACMD55 does not exist. If multiple CMD55 are issued continuously, APP_CMD bit in each
	  response is set to 1. The command issued immediately after the last CMD55 shall be
	  interpreted as ACMD. When more than one command (except CMD55) is issued directly
	  after CMD55, the first command is interpreted as ACMD and the following commands
	  are interpreted as regular commands
	*/

	// CMD55, Application specific
	card->port->ioOps.sendCommand(card, MMCBLK_COMM_APP_CMD, 0, 0, 0, NULL);//RCA is unkonwn yet
	response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_APP_CMD);
	if (EOK == mmcblk_evaluateResponse(&response)) {//no error occured
		u32 arg = (ocr & 0x00FFFFFF)  | (ocr & MMCBLK_OCR_CCS);
		/* CMD55 is accepted */
		LOG("CMD55 accepted - SD or COMBO CARD");

		/* ACMD41, to set voltage range for memory part or SD card */
		card->port->ioOps.sendCommand(card, MMCBLK_COMM_SD_APP_OP_COND, 0, 0, 0, NULL);
		response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_SD_APP_OP_COND);
		LOG("Card OP cond: %x", response.response.r3.ocr);
		if((response.response.r3.ocr & MMCBLK_OCR_V33_V34) || (response.response.r3.ocr & MMCBLK_OCR_V32_V33))
		{
			int ov = 0;
			do
			{
				proc_threadSleep(100);
				++ov;
				card->port->ioOps.sendCommand(card, MMCBLK_COMM_SD_APP_OP_COND, arg, 0, 0, NULL);
				response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_SD_APP_OP_COND);
				if(response.error || response.timeout)
				{
					LOG("Sd card init error");
					return -EFAULT;
				}
			}while(!(response.response.r3.ocr & MMCBLK_OCR_CPUPS) && ov < 255);


			if (card->label == eCardUnknown)
			{
				card->label = eCardSD;
			}
		}
		else
		{
			card->label = eCardUnsupported;
		}

		return 0;
	}
	else if (!response.timeout) {
		/* command/response pair is corrupted */
		LOG("CMD_APP failed");
		return -EFAULT;
	}
	else {
		// CMD55 is refuse, it must be MMC card or CE-ATA card
		LOG("CMD55 refused - MMC or CE-ATA");
		if (card->label == eCardSDCombo) {
			// change label
			card->label = eCardSDIO;
			LOG("Card is SDIO, not COMBO");
			//ignore the error or report it;
			// card is identified as SDIO card
			return 0;
		}
		card->port->ioOps.sendCommand(card, MMCBLK_COMM_SEND_OP_COND, ocr, 0, 0, NULL);
		response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_SEND_OP_COND);
		if(response.timeout)
		{ // CMD1 is not accepted, either
			card->label = eCardUnknown;
			LOG("CMD1 not accepted - card unknown");
			//label the card as UNKNOWN;
			return -1;
		}

		//TODO - check for CE-ATA signature succeeded
		if (0)
		{
			// the card is CE-ATA
			LOG("Detected CE-ATA card");
			// store CE-ATA specific info from the signature;
			card->label = eCardCEATA;
			//label the card as CE-ATA;
		} // of if (check for CE-ATA ...
		else
		{
			LOG("Detected MMC card");
			card->label = eCardMMC;
		}
	}
	return 0;





}


static int mmcblk_cardInit(void *cidx){
	int cardIdx = (int) cidx;
	int inserted = 0;
	int status = 0;
	proc_threadSleep(100*1000);
	inserted = !gpio_getLinePN(card[cardIdx].port->CDPort);
	if(inserted)
	{
		LOG("Mmcblk card init: %d", cardIdx);
		card[cardIdx].port->ioOps.reset(&card[cardIdx]);
		status = cardIdentify(&card[cardIdx]);
		if(status != EOK || card[cardIdx].label == eCardUnknown || card[cardIdx].label == eCardUnsupported)
			LOG("Card identification failure");
		else {
			LOG("Card identification success");
			assert(card[cardIdx].label < eCardTypeNum && card[cardIdx].label > 0);
			if(card[cardIdx].label != eCardSDCombo)
			{
				card[cardIdx].cardOps = &cardOps[card[cardIdx].label];
				if(EOK == card[cardIdx].cardOps->init(&card[cardIdx]))
				{
					LOG("Card initialized");
					int minor = dev_minorAlloc(MAKEDEV(MAJOR_MMCBLK, 0));
					if(minor < 0) {
						LOG("No minor for MMCBLK available");
						card[cardIdx].port->ioOps.reset(&card[cardIdx]);
					} else {
						status = subdev_register(MAKEDEV(MAJOR_MMCBLK, minor), &card[cardIdx]);
						if(status != EOK) {
							LOG("Subdev register failed");
							card[cardIdx].port->ioOps.reset(&card[cardIdx]);
						}
						else {
							char label[] = "mmcblk00";
							if(minor < 10) {
								label[6] = minor + '0';
								label[7] = 0;
							}
							else {
								label[6] = minor/10 + '0';
								label[7] = minor%10 + '0';
							}
							assert(EOK==dev_mknod(MAKEDEV(MAJOR_MMCBLK, minor), label));
						}
					}
				}
				else
				{
					LOG("Card initialization failure");
				}
			}
			else
			{
				assert(!"COMBO card support Not implemented");
			}
		}


		gpio_configPN( card[cardIdx].port->CDPort, IN_EDGE_RISING, mmcblkCD, NULL);
	}
	else
		gpio_configPN( card[cardIdx].port->CDPort, IN_EDGE_FALLING, mmcblkCD, NULL);
	return 0;
}




int mmcblkCD(unsigned line, void *cidx)
{
	int cardIdx = (int) cidx;

	bool inserted = !gpio_getLinePN(card[cardIdx].port->CDPort);
	int status = EOK;
	if(inserted) {
		gpio_configPN( card[cardIdx].port->CDPort, IN_EDGE_RISING, NULL, NULL);
		status =  proc_thread(NULL, mmcblk_cardInit, NULL, 0, cidx, ttRegular);
		if(status)
			LOG("Failed to create card init thread");
	}
	else {
		LOG("Card removed");
		gpio_configPN( card[cardIdx].port->CDPort, IN_EDGE_FALLING, mmcblkCD, NULL);
	}
	return 0;
}





void  cardDeInit(MmcblkCard_t *card) {
	card->baudRate = 0;
	card->capacity = 0;
	card->busWidth = eMmcblkBusWidth1b;
	memset(&card->CID, 0x0, sizeof(&card->CID));
	memset(&card->CSD, 0x0, sizeof(&card->CSD));
	memset(&card->SCR, 0x0, sizeof(&card->SCR));
	memset(&card->OCR, 0x0, sizeof(&card->OCR));
	memset(&card->RCA, 0x0, sizeof(&card->RCA));
	card->label = eCardUnknown;
	card->eventReg = 0;
	card->speed = eBasicSpeed;
	card->voltage = 0;
	card->sectorSize = 512;
	proc_mutexTerminate(&card->lock);
	proc_eventTerminate(&card->event);
	proc_mutexCreate(&card->lock);
	proc_eventCreate(&card->event);
}


int mmcblk_read(file_t* file, offs_t offs, char *buff, unsigned int len)
{
	int ret = 0;

	MmcblkCard_t *card = NULL;
	assert(file != NULL);

	card = (MmcblkCard_t *) file->vnode->file_priv;
	assert(card != NULL);
	proc_mutexLock(&card->lock);

	ret = card->cardOps->read(card, offs, buff, len);

	proc_mutexUnlock(&card->lock);
	return ret;
}

int mmcblk_write(file_t* file, offs_t offs, char *buff, unsigned int len)
{
	int ret = 0;
	MmcblkCard_t *card = NULL;
	assert(file != NULL);

	card = (MmcblkCard_t *) file->vnode->file_priv;
	assert(card != NULL);
	proc_mutexLock(&card->lock);
	ret = card->cardOps->write(card, offs, buff, len);
	proc_mutexUnlock(&card->lock);
	return ret;
}

int mmcblk_poll(file_t *file, ktime_t timeout, int op)
{
	int ret = 0;
	MmcblkCard_t *card = NULL;
	assert(file != NULL);
	card = (MmcblkCard_t *) file->vnode->file_priv;
	assert(card != NULL);
	proc_mutexLock(&card->lock);
	proc_mutexUnlock(&card->lock);
	return ret;
}

int mmcblk_select_poll(file_t *file, unsigned *ready)
{
	int ret = 0;
	MmcblkCard_t *card = NULL;
	assert(file != NULL);
	card = (MmcblkCard_t *) file->vnode->file_priv;
	assert(card != NULL);
	proc_mutexLock(&card->lock);
	proc_mutexUnlock(&card->lock);
	return ret;
}

int mmcblk_ioctl(file_t* file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	MmcblkCard_t *card = NULL;
	assert(file != NULL);
	card = (MmcblkCard_t *) file->vnode->file_priv;
	assert(card != NULL);
	proc_mutexLock(&card->lock);
	proc_mutexUnlock(&card->lock);
	return ret;
}

int mmcblk_fsync(file_t* file)
{
	int ret = 0;
	MmcblkCard_t *card = NULL;
	assert(file != NULL);
	card = (MmcblkCard_t *) file->vnode->file_priv;
	assert(card != NULL);
	proc_mutexLock(&card->lock);
	proc_mutexUnlock(&card->lock);
	return ret;
}

int mmcblk_open(vnode_t *vnode, file_t* file)
{
	int ret = 0;

	assert(vnode != NULL);
	assert(vnode->type == vnodeDevice);

	if(subdev_get(vnode->dev, &vnode->file_priv) != EOK)
	{
		vnode->file_priv = NULL;
		return -1;
	}

	return ret;
}

int mmcblk_close(vnode_t *vnode, file_t* file)
{
	int ret = 0;
	MmcblkCard_t *card = (MmcblkCard_t *) vnode->file_priv;
	assert(card != NULL);
	proc_mutexLock(&card->lock);
	proc_mutexUnlock(&card->lock);
	return ret;
}

int mmcblk_release(vnode_t *vnode)
{
	int ret = 0;
	return ret;
}

file_ops_t mmcblk_fops;

void mmcblk_init() {
	int i = 0;
	MmcblkDesc_t *mmcblk = bsp_mmcblkInitParams();

	card = vm_kmalloc(sizeof(MmcblkCard_t) * mmcblk->pnum);
	assert(card != NULL);

	memset(card, 0x0, sizeof(MmcblkCard_t) * mmcblk->pnum);

	for(i=0; i < mmcblk->pnum; ++i)
	{
		card[i].port  = &mmcblk->pdesc[i];

		proc_mutexCreate(&card[i].lock);
		proc_eventCreate(&card[i].event);
		cardDeInit(&card[i]);

		/* initialize controller */
		card[i].port->ioOps.init(&card[i]);


		/* initialize card detect */
		{
			unsigned bank, line;

			gpio_resolve(card[i].port->CDPort, &bank, &line);

			gpio_setFilter(bank, line, 0);
			gpio_setFilterLength(bank, 31);
			gpio_setFilter(bank, line, 1);

			gpio_configPN(card[i].port->CDPort, IN_EDGE_FALLING, NULL, (void *)i);

		}

		gpio_configPN(card[i].port->CDPort, IN_EDGE_FALLING, mmcblkCD, (void *)i);
	}

	mmcblk_fops.read        = mmcblk_read;
	mmcblk_fops.write       = mmcblk_write;
	mmcblk_fops.poll        = mmcblk_poll;
	mmcblk_fops.select_poll = mmcblk_select_poll;
	mmcblk_fops.ioctl       = mmcblk_ioctl;
	mmcblk_fops.fsync       = mmcblk_fsync;
	mmcblk_fops.open        = mmcblk_open;
	mmcblk_fops.close       = mmcblk_close;
	mmcblk_fops.release     = mmcblk_release;

	if (dev_register(MAKEDEV(MAJOR_MMCBLK, 0), &mmcblk_fops) < 0) {
		main_printf(ATTR_ERROR, "dev/sdhc: Can't register device for /dev/sdhc!\n" );
	}

	proc_threadSleep(1000*100);/* FIXME - without it fpio_setLine returns 0 even if line is HIGH */

	for(i=0; i < mmcblk->pnum; ++i)
	{
		if(!gpio_getLinePN(card[i].port->CDPort))
			mmcblk_cardInit((void *)i);
	}

}
