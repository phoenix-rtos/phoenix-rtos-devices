#include <dev/mmcblk/mmcblk_sd.h>
#include <dev/mmcblk/mmcblk_priv.h>

int mmcblk_sd_init(MmcblkCard_t *card) {
	int x;
	int timeout=0;
	int status=0;
	MmcblkResponse_t response;
	u32 baudrate;
	MmcblkSCRReg_t *scr=NULL;
	assert(card != NULL);
	LOG("SD memory card");

	card->port->ioOps.sendCommand(card, MMCBLK_COMM_ALL_SEND_CID, 0, 0, 0, NULL);
	response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_ALL_SEND_CID);
	status = mmcblk_evaluateResponse(&response);
	if(status != EOK)
	{
		LOG("Failed to read CID sd card register");
		return -EFAULT;
	}

	card->CID = response.response.r2.cid;

	for(x=0;x < 4; ++x)
		LOG("%08x", card->CID.cid[x]);
	LOG("Manufacturer ID: %x", MMCBLK_CID_GET_MID(card->CID));
	LOG("OEM ID: %x%x", (char) (MMCBLK_CID_GET_OID(card->CID)>>8), (char) MMCBLK_CID_GET_OID(card->CID));
	LOG("PNM ID: %c%c%c%c%c", (char) (MMCBLK_CID_GET_PNM(card->CID)>>32), (char) (MMCBLK_CID_GET_PNM(card->CID)>>24), (char) (MMCBLK_CID_GET_PNM(card->CID)>>16), (char) (MMCBLK_CID_GET_PNM(card->CID)>>8), (char) MMCBLK_CID_GET_PNM(card->CID));

	card->port->ioOps.sendCommand(card, MMCBLK_COMM_SEND_RELATIVE_ADDR, 0, 0, 0, NULL);
	response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_SEND_RELATIVE_ADDR);
	status = mmcblk_evaluateResponse(&response);
	if(status != EOK)
	{
		LOG("Failed to get RCA");
		return -EFAULT;
	}
	card->RCA = ((u32)response.response.r6.response & 0xFFFF0000);
	LOG("RCA: %x %x %x", card->RCA, card->RCA >> 16, (u32)response.response.r6.bits.RCA);

	while(!timeout)
	{
		card->port->ioOps.sendCommand(card, MMCBLK_COMM_ALL_SEND_CID, 0, 0, 0, NULL);
		response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_ALL_SEND_CID);
		if(response.timeout)
		{
			timeout = 1;
			continue;
		}
		card->CID = response.response.r2.cid;

		card->port->ioOps.sendCommand(card, MMCBLK_COMM_SEND_RELATIVE_ADDR, 0, 0, 0, NULL);
		response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_SEND_RELATIVE_ADDR);
		if(response.timeout)
		{
			timeout = 1;
			continue;
		}
		card->RCA = ((u32)response.response.r6.response & 0xFFFF0000);
	}

	card->port->ioOps.sendCommand(card, MMCBLK_COMM_SEND_CSD, card->RCA, 0, 0, NULL);
	response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_SEND_CSD);
	status = mmcblk_evaluateResponse(&response);
	if(status != EOK)
	{
		LOG("Failed to get CSD");
		return -EFAULT;
	}

	card->CSD = response.response.r2.csd;
	LOG("CSD [0] [1] [2] [3] %08x%08x%08x%08x", card->CSD.csd20.csd[0], card->CSD.csd20.csd[1], card->CSD.csd20.csd[2], card->CSD.csd20.csd[3]);
	if(MMCBLK_CSD20_GET_CSD_STRUCTURE(card->CSD.csd20))
	{
		u64 capacity = MMCBLK_CSD20_GET_C_SIZE(card->CSD.csd20);
		capacity += 1;
		card->capacity =  (u32) (capacity << 10);
		LOG("CAPACITY: %u blocks", card->capacity);

	}
	else
	{
		u32 csize = MMCBLK_CSD10_GET_C_SIZE(card->CSD.csd10);
		u32 mult = (1 << (MMCBLK_CSD10_GET_C_SIZE_MULT(card->CSD.csd10) + 2));
		u32 bLen = (1 << MMCBLK_CSD10_GET_READ_BL_LEN(card->CSD.csd10));
		u32 capacity = ((u64)(csize + 1) * mult * bLen) >> MMCBLK_CSD10_GET_READ_BL_LEN(card->CSD.csd10);
		card->capacity = capacity;
		LOG("CAPACITY: %u blocks", capacity);

	}

	baudrate = MMCBLK_CSD_GET_BAUDRATE(card->CSD.csd10);

	LOG("Switching to full speed mode");
	card->baudRate = card->port->ioOps.setupBaudRate(card, (baudrate < MMCBLK_SD_MAX_FULLSPEED_BAUDRATE)?baudrate:MMCBLK_SD_MAX_FULLSPEED_BAUDRATE);
	card->speed = eSDFullSpeed;

	LOG("Full speed baudrate: %u", card->baudRate);
	/* deselecting card - timeout expected */
	card->port->ioOps.sendCommand(card, MMCBLK_COMM_SELECT_DESELECT_CARD, 0, 0, 0, NULL);
	response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_SELECT_DESELECT_CARD);
	/* selecting a card */
	card->port->ioOps.sendCommand(card, MMCBLK_COMM_SELECT_DESELECT_CARD, card->RCA, 0, 0, NULL);
	response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_SELECT_DESELECT_CARD);
	status = mmcblk_evaluateResponse(&response);
	if(status != EOK)
	{
		LOG("Failed to select the card");
		return -EFAULT;
	}

	card->port->ioOps.sendCommand(card, MMCBLK_COMM_SET_BLOCKLEN, MMCBLK_BLOCK_LENGTH, 0, 0, NULL);
	response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_SET_BLOCKLEN);
	status = mmcblk_evaluateResponse(&response);
	if(status != EOK)
	{
		LOG("Failed to setup block size");
		return -EFAULT;
	}

	{
		void *fptr=NULL;
		FreePtr *fp;
		void *dmaDesc = NULL;

		scr = vm_dokmallocaligned(sizeof(MmcblkSCRReg_t), SIZE_CACHE_LINE, &fptr);
		if(scr == NULL)
			return -ENOMEM;

		dmaDesc = card->port->ioOps.setupDMA(card, scr, sizeof(*scr), &fp, NULL);
		if(dmaDesc == NULL) {
			vm_kfree(fptr);
			return -ENOMEM;
		}

		addr_t phyad;
		vm_kmapResolve(scr, &phyad);

		card->port->ioOps.setupEndian(card, eBigEndian);
		card->port->ioOps.sendCommand(card, MMCBLK_COMM_SEND_SCR, card->RCA, 1, sizeof(*scr), dmaDesc);
		response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_SEND_SCR);
		status = mmcblk_evaluateResponse(&response);
		if(status != EOK)
		{
			card->port->ioOps.reset(card);
			vm_kfree(fptr);
			card->port->ioOps.freeDMA(fp);
			return -EFAULT;
		}
		if(card->port->ioOps.transferWait(card) != EOK) {
			LOG("Failed to get SCR register");
			return -EFAULT;
		}

		hal_cpuInvalCache(scr, sizeof(*scr));
		card->SCR = *scr;

		card->port->ioOps.freeDMA(fp);
		vm_kfree(fptr);
	}

	LOG("SCR register: %08x%08x", card->SCR.scr[0], card->SCR.scr[1]);
	LOG("SCR cmd support: %x", MMCBLK_SCR_GET_CMD_SUPPORT(card->SCR));
	LOG("SCR sd spec 3: %x", MMCBLK_SCR_GET_SD_SPEC3(card->SCR));
	LOG("SCR sd bus widths: %x", MMCBLK_SCR_GET_SD_BUS_WIDTHS(card->SCR));
	LOG("SCR sd security: %x", MMCBLK_SCR_GET_SD_SECURITY(card->SCR));
	LOG("SCR data stat after erase: %x", MMCBLK_SCR_GET_DATA_STAT_AFTER_ERASE(card->SCR));
	LOG("SCR sd spec: %x", MMCBLK_SCR_GET_SD_SPEC(card->SCR));
	LOG("SCR scr structure: %x", MMCBLK_SCR_GET_SCR_STRUCTURE(card->SCR));

	if(MMCBLK_SCR_GET_SD_BUS_WIDTHS(card->SCR) & (1 << 2))
	{
		LOG("Switching to 4-bit mode");
		card->port->ioOps.sendCommand(card, MMCBLK_COMM_SET_BUS_WIDTH, 0x2, 0, 0, NULL);
		response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_SET_BUS_WIDTH);
		status = mmcblk_evaluateResponse(&response);
		if(status != EOK)
		{
			LOG("Failed to switch into 4-bit mode");
			return 0;
		}
		card->port->ioOps.setupBusWidth(card, eMmcblkBusWidth4b);
	}

	card->port->ioOps.setupEndian(card, eLittleEndian);

	LOG("Card baudrate: %u", card->baudRate);
	return 0;


}

void mmcblk_sd_deinit(MmcblkCard_t *card) {
	assert(card != NULL);
	assert(!"Not implemented");
}

int mmcblk_sd_inserted(MmcblkCard_t *card) {
	assert(card != NULL);
	assert(!"Not implemented");
	return 0;
}

int mmcblk_sd_switchHighSpeed(MmcblkCard_t *card) {
	assert(card != NULL);
	assert(!"Not implemented");
	return 0;
}

int mmcblk_sd_write(MmcblkCard_t *card, offs_t offs, char *buff, unsigned int len) {
	s32 ret=0;
	FreePtr *fp;
	void *dmaDesc = NULL;
	u32 sectorNum=0;
	MmcblkResponse_t response;
	int status=0;

	u32 sizeHead=SIZE_CACHE_LINE - ((u32)buff & (SIZE_CACHE_LINE-1));
	u32 sizeTail=(u32)buff & (SIZE_CACHE_LINE-1);
	status=status;
	assert(card != NULL);
	/* head/tail buffer - for cache management purposes */
	char *ht=NULL;
	void *headFreePtr=NULL;

	/* 4 - aligned buffer required, length - multiplicity of 512 */
	assert(!((u32)buff & 0x3) && !(len & (MMCBLK_BLOCK_LENGTH-1)));
	assert(!(offs & (MMCBLK_BLOCK_LENGTH-1)));
	if((offs >> 9) > card->capacity)
		return -EINVAL;
	if(len == 0)
		return ret;

	assert(buff != NULL);

	sectorNum = len >> 9;

	ht = vm_dokmallocaligned(2 * SIZE_CACHE_LINE, SIZE_CACHE_LINE, &headFreePtr);
	if(ht == NULL) {
		assert(0);
		return -ENOMEM;
	}

	dmaDesc = card->port->ioOps.setupDMA(card, buff, len, &fp, ht);
	if(dmaDesc == NULL) {
		vm_kfree(headFreePtr);
		assert(0);
		return -ENOMEM;
	}

	hal_cpuFlushCache( (char *)(((u32) buff & ~(SIZE_CACHE_LINE-1))+SIZE_CACHE_LINE), len-SIZE_CACHE_LINE);

	memcpy(ht, buff, sizeHead);
	if(sizeTail > 0)
		memcpy(ht+SIZE_CACHE_LINE, buff+len-sizeTail, sizeTail);

	hal_cpuFlushCache(ht, SIZE_CACHE_LINE*2);

	do {
		hal_cpuReschedule();
		card->port->ioOps.sendCommand(card, MMCBLK_COMM_SEND_STATUS, card->RCA, 0, 0, NULL);
		response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_SEND_STATUS);
	} while(response.response.r1.bits.CURRENT_STATE != 4);

	if(MMCBLK_CSD20_GET_CSD_STRUCTURE(card->CSD.csd10))
		offs >>= 9;

	if(len == MMCBLK_BLOCK_LENGTH) {
		card->port->ioOps.sendCommand(card, MMCBLK_COMM_WRITE_BLOCK, offs, 1, MMCBLK_BLOCK_LENGTH, dmaDesc);
		response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_WRITE_BLOCK);
	}
	else {
		card->port->ioOps.sendCommand(card, MMCBLK_COMM_WRITE_MULTIPLE_BLOCK, offs, sectorNum, MMCBLK_BLOCK_LENGTH, dmaDesc);
		response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_WRITE_MULTIPLE_BLOCK);
	}
	status = mmcblk_evaluateResponse(&response);

	if(response.error || (status != EOK && status != -EBUSY && status != -ETIMEDOUT)) {
		LOG("Write cmd error: %d", status);
		assert(0);
		ret = -1;
	}
	else {
		ret = card->port->ioOps.transferWait(card);
		if(ret == EOK)
			ret = len;
		else {
			LOG("Write error");
			if(len > MMCBLK_BLOCK_LENGTH) {
				card->port->ioOps.sendCommand(card, MMCBLK_COMM_STOP_TRANSMISSION, 0, 0, 0, NULL);
				response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_STOP_TRANSMISSION);
				card->port->ioOps.waitBusy(card);
				/* TODO - calculate number of blocks written properly */
			}
			/* TODO - appropriate reset operation sdhc->SYSCTL |= SDHC_SYSCTL_RSTD_MASK; */
			card->port->ioOps.reset(card);
		}
	}
	card->port->ioOps.freeDMA(fp);
	vm_kfree(headFreePtr);
	return ret;
}

int mmcblk_sd_read(MmcblkCard_t *card, offs_t offs, char *buff, unsigned int len) {
	s32 ret=0;
	FreePtr *fp;
	void *dmaDesc = NULL;
	u32 sectorNum=0;
	MmcblkResponse_t response;
	int status = 0;

	u32 sizeHead=SIZE_CACHE_LINE - ((u32)buff & (SIZE_CACHE_LINE-1));
	u32 sizeTail=(u32)buff & (SIZE_CACHE_LINE-1);
	status = status;
	assert(card != NULL);
	LOG("Read len: %u", len);
	/* head/tail buffer - for cache management purposes */
	char *ht=NULL;
	void *headFreePtr=NULL;


	/* 4 - aligned buffer required, length - multiplicity of 512 */
	assert(!((u32)buff & 0x3) && !(len & (MMCBLK_BLOCK_LENGTH-1)));
	assert(!(offs & (MMCBLK_BLOCK_LENGTH-1)));

	if((offs >> 9) > card->capacity)
		return -EINVAL;

	if(len == 0)
		return 0;
	assert(buff != NULL);
	sectorNum = len >> 9;


	ht = vm_dokmallocaligned(2 * SIZE_CACHE_LINE, SIZE_CACHE_LINE, &headFreePtr);
	if(ht == NULL) {
		assert(0);
		return -ENOMEM;
	}
	dmaDesc = card->port->ioOps.setupDMA(card, buff, len, &fp, ht);
	if(dmaDesc == NULL) {
		vm_kfree(headFreePtr);
		return -ENOMEM;
	}


	hal_cpuInvalCache(ht, SIZE_CACHE_LINE*2);
	LOG("BUFF: 0x%p TOTAL_LEN: %u INVALPTR: 0x%p INVALLEN: %u", buff, len, (char *)(((u32) buff & ~(SIZE_CACHE_LINE-1))+SIZE_CACHE_LINE), len-SIZE_CACHE_LINE);
	hal_cpuInvalCache((char *)(((u32) buff & ~(SIZE_CACHE_LINE-1))+SIZE_CACHE_LINE), len-SIZE_CACHE_LINE);

	do {
		hal_cpuReschedule();
		card->port->ioOps.sendCommand(card, MMCBLK_COMM_SEND_STATUS, card->RCA, 0, 0, NULL);
		response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_SEND_STATUS);
	} while(response.response.r1.bits.CURRENT_STATE != 4);

	if(MMCBLK_CSD20_GET_CSD_STRUCTURE(card->CSD.csd10))//sdhc uses block unit address, sdsc uses byte address
		offs >>= 9;

	if(len == MMCBLK_BLOCK_LENGTH) {
		card->port->ioOps.sendCommand(card, MMCBLK_COMM_READ_SINGLE_BLOCK, offs, 1, MMCBLK_BLOCK_LENGTH, dmaDesc);
		response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_READ_SINGLE_BLOCK);
	}
	else {
		card->port->ioOps.sendCommand(card, MMCBLK_COMM_READ_MULTIPLE_BLOCK, offs, sectorNum, MMCBLK_BLOCK_LENGTH, dmaDesc);
		response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_READ_MULTIPLE_BLOCK);
	}
	status = mmcblk_evaluateResponse(&response);

	if(response.error || (status != EOK && status != -EBUSY)) {
		LOG("Read cmd error");
		ret = -1;
	}
	else {
		ret = card->port->ioOps.transferWait(card);
		if(ret == EOK)
			ret = len;
		else {
			LOG("Read error");

			if(len > MMCBLK_BLOCK_LENGTH) {

				card->port->ioOps.sendCommand(card, MMCBLK_COMM_STOP_TRANSMISSION, 0, 0, 0, NULL);
				response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_STOP_TRANSMISSION);
				card->port->ioOps.waitBusy(card);
				/* TODO - calculate number of blocks read properly */
			}
			/* TODO - appropriate reset operation sdhc->SYSCTL |= SDHC_SYSCTL_RSTD_MASK; */
			card->port->ioOps.reset(card);
		}
	}

	card->port->ioOps.freeDMA(fp);
	memcpy(buff, ht, sizeHead);
	if(sizeTail > 0)
		memcpy(buff+len-sizeTail, ht+SIZE_CACHE_LINE, sizeTail);
	vm_kfree(headFreePtr);

	return ret;
}
