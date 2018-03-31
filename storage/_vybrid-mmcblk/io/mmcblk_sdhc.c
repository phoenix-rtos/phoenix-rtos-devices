#include <dev/mmcblk/io/mmcblk_sdhc.h>
#include <dev/mmcblk/mmcblk_priv.h>





typedef union __attribute__(( packed, aligned(4) )) {
	u64 desc;
	struct {
		u32 Valid : 1;
		u32 End : 1;
		u32 Int : 1;
		u32 unused : 1;
		u32 Act : 2;
		u32 reserved : 10;
		u32 DataLen : 16;
		u32 DataAddr;
	} bits;
} SdhcAdma2Desc_t;

#define ADMA_DESC_ALIGN (((2 * sizeof(SdhcAdma2Desc_t)) < SIZE_CACHE_LINE ) ? ( SIZE_CACHE_LINE ) : (((2 * sizeof(SdhcAdma2Desc_t) / SIZE_CACHE_LINE) + 1) * SIZE_CACHE_LINE))


#define  PRINT_SD_STATE(r1) do {\
	const char * name = NULL;\
	switch((r1).bits.CURRENT_STATE) {\
	case 0:\
	name = "idle";\
	break;\
	case 1:\
	name = "ready";\
	break;\
	case 2:\
	name = "ident";\
	break;\
	case 3:\
	name = "stby";\
	break;\
	case 4:\
	name = "tran";\
	break;\
	case 5:\
	name = "data";\
	break;\
	case 6:\
	name = "rcv";\
	break;\
	case 7:\
	name = "prg";\
	break;\
	case 8:\
	name = "dis";\
	break;\
	default:\
	name = "unknown";\
	break;\
	}\
	name=name;\
	LOG("CARD STATE: %u - %s", (r1).bits.CURRENT_STATE, name);\
	}while(0)



#define PRINT_SD_STATUS(r1) do{\
	if((r1).bits.ADDRESS_ERROR)\
	LOG("Address error");\
	if((r1).bits.AKE_SEQ_ERROR)\
	LOG("Ake seq error");\
	if((r1).bits.APP_CMD)\
	LOG("App cmd");\
	if((r1).bits.BLOCK_LEN_ERROR)\
	LOG("Block len error");\
	if((r1).bits.CARD_ECC_DISABLED)\
	LOG("Card ecc disabled");\
	if((r1).bits.CARD_ECC_FAILED)\
	LOG("Card ecc failed");\
	if((r1).bits.CARD_IS_LOCKED)\
	LOG("Card is locked");\
	if((r1).bits.CC_ERROR)\
	LOG("Cc error");\
	if((r1).bits.COM_CRC_ERROR)\
	LOG("Crc error");\
	if((r1).bits.CSD_OVERWRITE)\
	LOG("Csd overwrite");\
	if((r1).bits.ERASE_PARAM)\
	LOG("Erase param");\
	if((r1).bits.ERASE_RESET)\
	LOG("Erase reset");\
	if((r1).bits.ERASE_SEQ_ERROR)\
	LOG("Erase seq error");\
	if((r1).bits.ERROR)\
	LOG("Error!");\
	if((r1).bits.ILLEGAL_COMMAND)\
	LOG("Illegal command");\
	if((r1).bits.LOCK_UNLOCK_FAILED)\
	LOG("Lock unlock failed");\
	if((r1).bits.OUT_OF_RANGE)\
	LOG("Oout of range");\
	if((r1).bits.READY_FOR_DATA)\
	LOG("Ready for data");\
	if((r1).bits.WP_ERASE_SKIP)\
	LOG("Wp erase skip");\
	if((r1).bits.WP_VIOLATION)\
	LOG("Wp violation");\
	}while(0);


static int mmcblk_sdhc_waitFor(MmcblkCard_t* card, u32 eventMask, ktime_t timeout)
{
	int ret=0;
	ktime_t start=hal_upTime();
	while((ret = (card->eventReg & eventMask)) == 0) {
		if(-ETIME == proc_eventWait(&card->event, timeout-(hal_upTime() - start))) {
			if(eventMask & MMCBLK_EVENT_COMMAND_TIMEOUT) ret |= MMCBLK_EVENT_COMMAND_TIMEOUT;
			if(eventMask & MMCBLK_EVENT_TRANSFER_TIMEOUT) ret |= MMCBLK_EVENT_TRANSFER_TIMEOUT;
			break;
		}
	}
	return ret;
}







static int mmcblk_sdhc_irqHandler(unsigned int irqn, cpu_context_t *ctx, void *arg)
{
	SDHC_Type *sdhc = NULL;
	MmcblkCard_t *card = (MmcblkCard_t *) arg;
	sdhc = (SDHC_Type *) card->port->ioBase;



	u32 sdhc_irqstat;
	int served=0;

	/* Back up the IRQ status */
	sdhc_irqstat  = sdhc->IRQSTAT;

	// Clear the all sets IRQ status bits
	sdhc->IRQSTAT = sdhc_irqstat;

	/*
		DMA Error
	  Occurs when an Internal DMA transfer has failed. This bit is set to 1, when some error occurs in the data
	  transfer. This error can be caused by either Simple DMA or ADMA, depending on which DMA is in use.
	  The value in DMA System Address register is the next fetch address where the error occurs. Since any
	  error corrupts the whole data block, the host driver shall re-start the transfer from the corrupted block
	  boundary. The address of the block boundary can be calculated either from the current DSADDR value or
	  from the remaining number of blocks and the block size.
	*/
	if((sdhc->IRQSIGEN & SDHC_IRQSIGEN_DMAEIEN_MASK) && (sdhc_irqstat & SDHC_IRQSTAT_DMAE_MASK))
	{
		card->eventReg |= MMCBLK_EVENT_TRANSFER_ERROR;
		proc_eventSignal(&card->event);
		served=1;
	}

	/*
		Auto CMD12 Error
	  Occurs when detecting that one of the bits in the Auto CMD12 Error Status register has changed from 0 to
	  1. This bit is set to 1, not only when the errors in Auto CMD12 occur, but also when the Auto CMD12 is
	  not executed due to the previous command error.
	*/
	if((sdhc->IRQSIGEN & SDHC_IRQSIGEN_AC12EIEN_MASK) && (sdhc_irqstat & SDHC_IRQSTAT_AC12E_MASK))
	{
		//_lwevent_set( &esdhc_device_ptr->LWEVENT, (ESDHC_LWEVENT_CMD_ERROR | ESDHC_LWEVENT_TRANSFER_ERROR));
		card->eventReg |= (MMCBLK_EVENT_TRANSFER_ERROR | MMCBLK_EVENT_COMMAND_ERROR);
		proc_eventSignal(&card->event);
		served=1;
	}

	/*
		Data End Bit Error
	  Occurs either when detecting 0 at the end bit position of read data, which uses the DAT line, or at the end
	  bit position of the CRC.
	*/
	if((sdhc->IRQSIGEN & SDHC_IRQSIGEN_DEBEIEN_MASK) && (sdhc_irqstat & SDHC_IRQSTAT_DEBE_MASK))
	{
		card->eventReg |= MMCBLK_EVENT_TRANSFER_ERROR;
		proc_eventSignal(&card->event);
		served=1;
	}

	/*
		Data CRC Error
	  Occurs when detecting a CRC error when transferring read data, which uses the DAT line, or when
	  detecting the Write CRC status having a value other than 010.
	*/
	if((sdhc->IRQSIGEN & SDHC_IRQSIGEN_DCEIEN_MASK) && (sdhc_irqstat & SDHC_IRQSTAT_DCE_MASK))
	{
		card->eventReg |= MMCBLK_EVENT_TRANSFER_ERROR;
		proc_eventSignal(&card->event);
		served=1;
	}

	/*
		Data Timeout Error
	  Occurs when detecting one of following time-out conditions.
		\95 Busy time-out for R1b,R5b type
		\95 Busy time-out after Write CRC status
		\95 Read Data time-out
	*/
	if((sdhc->IRQSIGEN & SDHC_IRQSIGEN_DTOEIEN_MASK) && (sdhc_irqstat & SDHC_IRQSTAT_DTOE_MASK))
	{
		card->eventReg |= MMCBLK_EVENT_TRANSFER_TIMEOUT;
		proc_eventSignal(&card->event);
		served=1;
	}

	/*
		Command Index Error
	  Occurs if a Command Index error occurs in the command response.
	*/
	if((sdhc->IRQSIGEN & SDHC_IRQSIGEN_CIEIEN_MASK) && (sdhc_irqstat & SDHC_IRQSTAT_CIE_MASK))
	{
		card->eventReg |= MMCBLK_EVENT_COMMAND_ERROR;
		proc_eventSignal(&card->event);
		served=1;
	}

	/*
		Command End Bit Error
	  Occurs when detecting that the end bit of a command response is 0.
	*/
	if((sdhc->IRQSIGEN & SDHC_IRQSIGEN_CEBEIEN_MASK) && (sdhc_irqstat & SDHC_IRQSTAT_CEBE_MASK))
	{
		card->eventReg |= MMCBLK_EVENT_COMMAND_ERROR;
		proc_eventSignal(&card->event);
		served=1;
	}



	/*
		Command CRC Error
	  Command CRC Error is generated in two cases.
	  \95 If a response is returned and the Command Timeout Error is set to 0 (indicating no time-out), this bit
	  is set when detecting a CRC error in the command response.
	  \95 The SDHC detects a CMD line conflict by monitoring the CMD line when a command is issued. If
	  the SDHC drives the CMD line to 1, but detects 0 on the CMD line at the next SDCLK edge, then
	  the SDHC shall abort the command (Stop driving CMD line) and set this bit to 1. The Command
	  Timeout Error shall also be set to 1 to distinguish CMD line conflict.
	*/
	if((sdhc->IRQSIGEN & SDHC_IRQSIGEN_CCEIEN_MASK) && (sdhc_irqstat & SDHC_IRQSTAT_CCE_MASK))
	{
		card->eventReg |= MMCBLK_EVENT_COMMAND_ERROR;
		proc_eventSignal(&card->event);
		served=1;
	}


	/*
		Command Timeout Error
	  Occurs only if no response is returned within 64 SDCLK cycles from the end bit of the command. If the
	  SDHC detects a CMD line conflict, in which case a Command CRC Error shall also be set, this bit shall be
	  set without waiting for 64 SDCLK cycles. This is because the command will be aborted by the SDHC.
	*/
	if((sdhc->IRQSIGEN & SDHC_IRQSIGEN_CTOEIEN_MASK) && (sdhc_irqstat & SDHC_IRQSTAT_CTOE_MASK))
	{
		card->eventReg |= MMCBLK_EVENT_COMMAND_TIMEOUT;
		proc_eventSignal(&card->event);
		served=1;
	}

	/*
		Card Interrupt
	  This status bit is set when an interrupt signal is detected from the external card-> In 1-bit mode, the SDHC
	  will detect the Card Interrupt without the SD Clock to support wakeup. In 4-bit mode, the card interrupt
	  signal is sampled during the interrupt cycle, so the interrupt from card can only be sampled during
	  interrupt cycle, introducing some delay between the interrupt signal from the SDIO card and the interrupt
	  to the host system. Writing this bit to 1 can clear this bit, but as the interrupt factor from the SDIO card
	  does not clear, this bit is set again. In order to clear this bit, it is required to reset the interrupt factor from
	  the external card followed by a writing 1 to this bit.
	  When this status has been set, and the host driver needs to service this interrupt, the Card Interrupt
	  Signal Enable in the Interrupt Signal Enable register should be 0 to stop driving the interrupt signal to the
	  host system. After completion of the card interrupt service (It should reset the interrupt factors in the SDIO
	  card and the interrupt signal may not be asserted), write 1 to clear this bit, set the Card Interrupt Signal
	  Enable to 1, and start sampling the interrupt signal again.
	*/
	if((sdhc->IRQSIGEN & SDHC_IRQSIGEN_CINTIEN_MASK) && (sdhc_irqstat & SDHC_IRQSTAT_CINT_MASK))
	{
		//LOG("CARD INTERRUPT");
		served=1;
		//if(esdhc_device_ptr->IO_CALLBACK_STR.CALLBACK)
		//esdhc_device_ptr->IO_CALLBACK_STR.CALLBACK(esdhc_device_ptr->IO_CALLBACK_STR.USERDATA);
		//TODO
	}


	/*
		DMA Interrupt
	  Occurs only when the internal DMA finishes the data transfer successfully. Whenever errors occur during
	  data transfer, this bit will not be set. Instead, the DMAE bit will be set. Either Simple DMA or ADMA
	  finishes data transferring, this bit will be set.
	*/
	if((sdhc->IRQSIGEN & SDHC_IRQSIGEN_DINTIEN_MASK) && (sdhc_irqstat & SDHC_IRQSTAT_DINT_MASK))
	{
		card->eventReg |= MMCBLK_EVENT_TRANSFER_COMPLETED;
		proc_eventSignal(&card->event);
		//LOG("TRANSFER COMPLETED");
		served=1;
	}

	/*
		Block Gap Event
	  If the PROCTL[SABGREQ] is set, this bit is set when a read or write transaction is stopped at a block gap.
	  If PROCTL[SABGREQ] is not set to 1, this bit is not set to 1.
	  In the case of a read transaction: This bit is set at the falling edge of the DAT line active status (When the
	  transaction is stopped at SD Bus timing). The read wait must be supported in order to use this function.
	  In the case of write transaction: This bit is set at the falling edge of write transfer active status (After
	  getting CRC status at SD bus timing).
	*/
	if((sdhc->IRQSIGEN & SDHC_IRQSIGEN_BGEIEN_MASK) && (sdhc_irqstat & SDHC_IRQSTAT_BGE_MASK))
	{
		card->eventReg |= MMCBLK_EVENT_TRANSFER_ERROR;
		proc_eventSignal(&card->event);
		served=1;
	}

	/*
		Transfer Complete
	  This bit is set when a read or write transfer is completed.
	  In the case of a read transaction: This bit is set at the falling edge of the read transfer active status. There
	  are two cases in which this interrupt is generated. The first is when a data transfer is completed as
	  specified by the data length (after the last data has been read to the host system). The second is when
	  data has stopped at the block gap and completed the data transfer by setting the PROCTL[SABGREQ]
	  (after valid data has been read to the host system).
	  In the case of a write transaction: This bit is set at the falling edge of the DAT line active status. There are
	  two cases in which this interrupt is generated. The first is when the last data is written to the SD card as
	  specified by the data length and the busy signal is released. The second is when data transfers are
	  stopped at the block gap, by setting the PROCTL[SABGREQ], and the data transfers are completed.
	  (after valid data is written to the SD card and the busy signal released).
	*/
	if((sdhc->IRQSIGEN & SDHC_IRQSIGEN_TCIEN_MASK) && (sdhc_irqstat & SDHC_IRQSTAT_TC_MASK))
	{
		//LOG("TC");
		served=1;
		//card->eventReg |= MMCBLK_EVENT_TRANSFER_COMPLETED;
		//proc_eventSignal(&card->event);
	}

	/*
		Command Complete
	  This bit is set when you receive the end bit of the command response (except Auto CMD12). Refer to the
	  PRSSTAT[CIHB].
	*/
	if((sdhc->IRQSIGEN & SDHC_IRQSIGEN_CCIEN_MASK) && (sdhc_irqstat & SDHC_IRQSTAT_CC_MASK))
	{
		//LOG("CC");
		card->eventReg |= MMCBLK_EVENT_COMMAND_COMPLETED;
		proc_eventSignal(&card->event);
		served=1;
	}

	if(!served) {
		LOG("INTERRUPT NOT SERVED: 0x%x", sdhc_irqstat);
	}

	return IHRES_HANDLED;
}




void mmcblk_sdhc_init(void *cardPtr) {
	int status=0;
	SDHC_Type *sdhc = NULL;
	MmcblkCard_t *card = NULL;

	assert(cardPtr != NULL);
	card = (MmcblkCard_t *) cardPtr;
	assert(card->port->ioBase == (void *)SDHC1_BASE);/* function not reentrant - invoke only once */

	status = vm_iomap((addr_t) card->port->ioBase, sizeof(SDHC_Type), PGHD_DEV_RW, (void *)&card->port->ioBase);
	if(status < 0)
	{
		LOG("Failed to iomap SDHC");
		return;
	}

	sdhc = card->port->ioBase;

	status = hal_interruptsSetHandler(card->port->irq, mmcblk_sdhc_irqHandler, (void *)card);
	if(status < 0)
	{
		LOG("Failed to set SDHC1 irq handler");
		return;
	}


	sdhc->SYSCTL = SDHC_SYSCTL_RSTA_MASK | (0x80 << SDHC_SYSCTL_SDCLKFS_SHIFT);

	while(sdhc->SYSCTL & SDHC_SYSCTL_RSTA_MASK)
		proc_threadSleep(100);

	sdhc->SYSCTL = SDHC_SYSCTL_PEREN_MASK;

	LOG("sdhc controller initialized");
}

int mmcblk_sdhc_sendCommand(void *cardPtr, u32 cmd, u32 cmd_arg, s32 block_num, u16 block_size, void *admaDT) {
	MmcblkCard_t *card = (MmcblkCard_t *) cardPtr;
	SDHC_Type *sdhc = NULL;
	int cmdIdx = (cmd & MMCBLK_COMM_INDEX_MASK);
	u32 wCmd = cmdIdx << 24;
	int status = EOK;
	u32 waitOn=SDHC_PRSSTAT_CIHB_MASK;
	u32 blkAttr=0;
	u32 admaDescPtr=0;

	assert(card != NULL);
	sdhc = card->port->ioBase;

	/* init Transfer Type register with command index */
	LOG("Send cmd: %x %x", cmd, cmd_arg);

	/* check if command value is in valid boundaries */
	if(cmdIdx > 0x64 || cmd > 0xFFFFU)
	{
		/* error - invalid command */
		LOG("Invalid command");
		return -EINVAL;
	}

	/* set RSPTYP CCCEN CICEN bits */
	switch(cmd & MMCBLK_COMM_RESPONSE_TYPE_MASK)
	{
		case MMCBLK_COMM_RESPONSE_NO_RESPONSE:
		break;
		case MMCBLK_COMM_RESPONSE_R2:
			wCmd |= (1 << SDHC_XFERTYP_RSPTYP_SHIFT) | (1 << SDHC_XFERTYP_CCCEN_SHIFT);
		break;
		case MMCBLK_COMM_RESPONSE_R3:
		case MMCBLK_COMM_RESPONSE_R4:
			wCmd |= (2 << SDHC_XFERTYP_RSPTYP_SHIFT);
		break;
		case MMCBLK_COMM_RESPONSE_R1:
		case MMCBLK_COMM_RESPONSE_R5:
		case MMCBLK_COMM_RESPONSE_R6:
		case MMCBLK_COMM_RESPONSE_R7:
			wCmd |= (2 << SDHC_XFERTYP_RSPTYP_SHIFT) | (1 << SDHC_XFERTYP_CCCEN_SHIFT) | (1 << SDHC_XFERTYP_CICEN_SHIFT);
		break;
		case MMCBLK_COMM_RESPONSE_R1b:
		case MMCBLK_COMM_RESPONSE_R5b:
			wCmd |= (3 << SDHC_XFERTYP_RSPTYP_SHIFT) | (1 << SDHC_XFERTYP_CCCEN_SHIFT) | (1 << SDHC_XFERTYP_CICEN_SHIFT);
		break;
	}

	/* set CMDTYP bits */
	if(cmdIdx == 52 || cmdIdx == 12)
	{
		if(cmdIdx == 52 && 0)
		{
			assert(!"suspend command not implemented");
			wCmd |= (0x1U << SDHC_XFERTYP_CMDTYP_SHIFT);
		}
		else if(cmdIdx == 52 && 0)
		{
			assert(!"resume command not implemented");
			wCmd |= (0x2U << SDHC_XFERTYP_CMDTYP_SHIFT);
		}
		else if(cmdIdx == 12 && 1)
			wCmd |= (0x3U << SDHC_XFERTYP_CMDTYP_SHIFT);
		else {
			assert(!"CMD52 not implemented");
		}
	}

	/* DPSEL bit */
	if ((cmd & MMCBLK_COMM_TYPE_MASK) == MMCBLK_COMM_ADTC)
	{

		if(cmdIdx == 52 || cmdIdx == 53 || cmdIdx == 56 || cmdIdx == 60 || cmdIdx == 61) {
			/* it also should be set in resume command */
			assert(!"Resume command not implemented");
		}
		else {
			wCmd |= SDHC_XFERTYP_DPSEL_MASK;
			wCmd |= SDHC_XFERTYP_DMAEN_MASK;
			waitOn |= SDHC_PRSSTAT_DLA_MASK;
		}
	}


	/* DTDSEL - valid only when DPSEL is 1 */
	if(!(cmd & MMCBLK_COMM_DAT_OPERATION_MASK) && (wCmd & SDHC_XFERTYP_DPSEL_MASK))
		wCmd |= SDHC_XFERTYP_DTDSEL_MASK;


	if ((block_num > 0 || block_num == -1) && block_size != 0)
	{

		if(block_num != 1 ) {
			wCmd |= SDHC_XFERTYP_MSBSEL_MASK;
			if(block_num > 1) {
				wCmd |= (SDHC_XFERTYP_AC12EN_MASK | SDHC_XFERTYP_BCEN_MASK);
			}
		}
		if(block_num == -1)
		{
			/* infinite transfer */
			blkAttr = SDHC_BLKATTR_BLKCNT(0xFFFF) | SDHC_BLKATTR_BLKSIZE(block_size);
			wCmd |=  SDHC_XFERTYP_BCEN_MASK;
		}
		else
		{
			blkAttr = SDHC_BLKATTR_BLKCNT((u32)block_num) | SDHC_BLKATTR_BLKSIZE(block_size);
		}
	}


	/* before writing to registers: */
	if(cmd & MMCBLK_COMM_ACMD)
	{
		LOG("Acmd");
		status = card->port->ioOps.sendCommand(card, MMCBLK_COMM_APP_CMD, card->RCA, 0, 0, 0);
		if(status != EOK)
		{
			LOG("Cmd app failed");
			return status;
		}
		card->port->ioOps.waitForResponse(card, MMCBLK_COMM_APP_CMD);
	}

	if(cmdIdx != 0 && cmdIdx != 12 && cmdIdx != 13 && cmdIdx != 52)
	{
		waitOn |= SDHC_PRSSTAT_CDIHB_MASK;
		sdhc->SYSCTL |= (SDHC_SYSCTL_PEREN_MASK | SDHC_SYSCTL_HCKEN_MASK | SDHC_SYSCTL_IPGEN_MASK);
	}


	while(sdhc->PRSSTAT & waitOn) {
		hal_cpuReschedule();
		/* TODO - timeout */
	}

	sdhc->IRQSTATEN &= ~SDHC_IRQSTATEN_DINTSEN_MASK;

	if((cmd & MMCBLK_COMM_RESPONSE_R1b) || (cmd & MMCBLK_COMM_RESPONSE_R5b) || (wCmd & SDHC_XFERTYP_AC12EN_MASK) )
		sdhc->SYSCTL |= (SDHC_SYSCTL_PEREN_MASK | SDHC_SYSCTL_HCKEN_MASK | SDHC_SYSCTL_IPGEN_MASK);
	else
		sdhc->SYSCTL &= ~(SDHC_SYSCTL_PEREN_MASK | SDHC_SYSCTL_HCKEN_MASK | SDHC_SYSCTL_IPGEN_MASK);

	vm_kmapResolve(admaDT, &admaDescPtr);


	sdhc->IRQSTAT = sdhc->IRQSTAT;
	sdhc->DSADDR = 0;
	sdhc->ADSADDR = admaDescPtr;
	sdhc->CMDARG = cmd_arg;
	sdhc->BLKATTR = blkAttr;

	/*
	 * set Transfer Type register as wCmd value to issue the command
	 */
	sdhc->XFERTYP = wCmd;

	sdhc->IRQSTATEN |= SDHC_IRQSTATEN_DINTSEN_MASK;

	return status;
}

MmcblkResponse_t mmcblk_sdhc_waitForResponse(void *cardPtr, int cmd) {
	MmcblkResponse_t ret;
	MmcblkCard_t *card = (MmcblkCard_t *) cardPtr;
	SDHC_Type *sdhc = NULL;
	u32 resp[4];
	u32 stat=0;

	assert(card != NULL);
	sdhc = card->port->ioBase;
	memset(&ret, 0x0, sizeof(ret));

	 if(!(sdhc->PRSSTAT &  SDHC_PRSSTAT_DLSL(1))) {
		 LOG("Command busy");
	 }

	 sdhc->IRQSTAT = sdhc->IRQSTAT;

	if( ((cmd & (MMCBLK_COMM_TYPE_MASK)) == MMCBLK_COMM_ADTC) || (cmd == (MMCBLK_COMM_STOP_TRANSMISSION)) )
	{
		stat = mmcblk_sdhc_waitFor(card,
					MMCBLK_EVENT_CARD_REMOVED |
					MMCBLK_EVENT_COMMAND_COMPLETED |
					MMCBLK_EVENT_COMMAND_ERROR |
					MMCBLK_EVENT_COMMAND_TIMEOUT, MMCBLK_CMD12_TIMEOUT_US);
	}
	else
	{
		stat = mmcblk_sdhc_waitFor(card,
					MMCBLK_EVENT_CARD_REMOVED |
					MMCBLK_EVENT_COMMAND_COMPLETED |
					MMCBLK_EVENT_COMMAND_ERROR |
					MMCBLK_EVENT_COMMAND_TIMEOUT, MMCBLK_CMD_TIMEOUT_US);
	}

	LOG("Event: %u", stat);
	card->eventReg  &= ~stat;
	memset(&ret, 0, sizeof(ret));
	ret.responseType = cmd & MMCBLK_COMM_RESPONSE_TYPE_MASK;

	ret.busy = !(sdhc->PRSSTAT &  SDHC_PRSSTAT_DLSL(1));


	if(ret.busy) {
		LOG("Busy!");
	}
	else
	{
		sdhc->SYSCTL &= ~(SDHC_SYSCTL_PEREN_MASK | SDHC_SYSCTL_HCKEN_MASK | SDHC_SYSCTL_IPGEN_MASK);
	}

	ret.timeout=!!(stat & MMCBLK_EVENT_COMMAND_TIMEOUT);

	if(ret.timeout)
	{
		LOG("Timeout");
		return ret;
	}
	ret.error = !!(stat & (MMCBLK_EVENT_COMMAND_ERROR | MMCBLK_EVENT_TRANSFER_ERROR));
	if(ret.error)
	{
		LOG("Error: %08x", stat);
		return ret;
	}


	resp[0] = sdhc->CMDRSP[0];
	resp[1] = sdhc->CMDRSP[1];
	resp[2] = sdhc->CMDRSP[2];
	resp[3] = sdhc->CMDRSP[3];

	LOG("Resp DBG 0: %08x", resp[0]);
	LOG("Resp DBG 1: %08x", resp[1]);
	LOG("Resp DBG 2: %08x", resp[2]);
	LOG("Resp DBG 3: %08x", resp[3]);

	switch(ret.responseType)
	{
		case MMCBLK_COMM_RESPONSE_NO_RESPONSE:
		break;
		case MMCBLK_COMM_RESPONSE_R1:
			ret.response.r1.response = resp[0];
			PRINT_SD_STATE(ret.response.r1);
			PRINT_SD_STATUS(ret.response.r1);
		break;
		case MMCBLK_COMM_RESPONSE_R1b:
			//TODO CMD12 auto response os in CMDRSP3
			if(0)
				ret.response.r1b.response = resp[3];
			else
				ret.response.r1b.response = resp[0];
			PRINT_SD_STATE(ret.response.r1);
			PRINT_SD_STATUS(ret.response.r1);
		break;
		case MMCBLK_COMM_RESPONSE_R2:
			ret.response.r2.cid.cid[0] = resp[3] & 0x00FFFFFF;
			ret.response.r2.cid.cid[1] = resp[2];
			ret.response.r2.cid.cid[2] = resp[1];
			ret.response.r2.cid.cid[3] = resp[0];
		break;
		case MMCBLK_COMM_RESPONSE_R3:
			ret.response.r3.ocr = resp[0];
		break;
		case MMCBLK_COMM_RESPONSE_R4:
			ret.response.r4.ocr = resp[0];
		break;
		case MMCBLK_COMM_RESPONSE_R5:
			ret.response.r5.response = resp[0];
		break;
		case MMCBLK_COMM_RESPONSE_R5b:
			ret.response.r5b.response = resp[0];
		break;
		case MMCBLK_COMM_RESPONSE_R6:
			ret.response.r6.response = resp[0];
			PRINT_SD_STATE(ret.response.r1);
			PRINT_SD_STATUS(ret.response.r1);
		break;
		case MMCBLK_COMM_RESPONSE_R7:
			ret.response.r7.response = resp[0];
		break;
	}

	return ret;
}

int mmcblk_sdhc_transferWait(void *cardPtr) {
	MmcblkCard_t *card = (MmcblkCard_t *) cardPtr;
	SDHC_Type *sdhc = NULL;
	u32 stat=0;
	assert(card != NULL);
	sdhc = card->port->ioBase;

	sdhc->SYSCTL |= (SDHC_SYSCTL_PEREN_MASK | SDHC_SYSCTL_HCKEN_MASK | SDHC_SYSCTL_IPGEN_MASK);
	stat = mmcblk_sdhc_waitFor(card,
				MMCBLK_EVENT_CARD_REMOVED |
				MMCBLK_EVENT_TRANSFER_COMPLETED |
				MMCBLK_EVENT_TRANSFER_ERROR |
				MMCBLK_EVENT_TRANSFER_TIMEOUT
				,
				MMCBLK_TRANSFER_TIMEOUT_US);
	card->eventReg &= ~stat;

	while(!(sdhc->PRSSTAT & SDHC_PRSSTAT_DLSL(1))) {
		sdhc->SYSCTL |= (SDHC_SYSCTL_PEREN_MASK | SDHC_SYSCTL_HCKEN_MASK | SDHC_SYSCTL_IPGEN_MASK);
		hal_cpuReschedule();
	};

	sdhc->SYSCTL &= ~(SDHC_SYSCTL_PEREN_MASK | SDHC_SYSCTL_HCKEN_MASK | SDHC_SYSCTL_IPGEN_MASK);

	if(stat & MMCBLK_EVENT_TRANSFER_ERROR) {
		LOG("Transfer error");
		assert(0);
		return -1;
	}
	return EOK;

}

int mmcblk_sdhc_waitBusy(void *cardPtr) {
	MmcblkCard_t *card = (MmcblkCard_t *) cardPtr;
	SDHC_Type *sdhc = NULL;
	assert(card != NULL);
	sdhc = card->port->ioBase;
	/*TODO - timeout */
	while(!(sdhc->PRSSTAT & SDHC_PRSSTAT_DLSL(1)))
	{
		sdhc->SYSCTL |= (SDHC_SYSCTL_PEREN_MASK | SDHC_SYSCTL_HCKEN_MASK | SDHC_SYSCTL_IPGEN_MASK);
		proc_threadSleep(100);
	}
	sdhc->SYSCTL &= ~(SDHC_SYSCTL_PEREN_MASK | SDHC_SYSCTL_HCKEN_MASK | SDHC_SYSCTL_IPGEN_MASK);
	return 0;
}


void mmcblk_sdhc_freeDMA(FreePtr *fp)
{
	if(fp->next != NULL)
	{
		mmcblk_sdhc_freeDMA(fp->next);
	}
	if(fp->freeptr != NULL)
		vm_kfree(fp->freeptr);
	vm_kfree(fp);
}



#define ADMA_DESC_DUMP(desc) LOG("ACT: %u; VALID: %u; END: %u; INT: %u; DATA-LEN: %u; DATA-PHY-ADDR: 0x%x", desc.bits.Act, desc.bits.Valid, desc.bits.End, desc.bits.Int, desc.bits.DataLen, desc.bits.DataAddr)

/**
* @brief sd_sdhc_setupAdmaDT - fills ADMA descriptor table
*
* @param dt      - pointer to descriptor table
*                - if null, function returns number of descriptors necessary to describe provided buffer.
* @param dtLen   - number of descriptors in descriptor table
* @param bufptr  - pointer to buffer to use by DMA
* @param bufsize - size of buffer(bufptr)
* @param htBuf   - buffer for head/tail (size 2 * SIZE_CACHE_LINE ) - NULL is ok when bufptr is SIZE_CACHE_LINE aligned
* @return number of descriptors needed/used
*         -1 on error
*/

void *mmcblk_sdhc_setupDMA(void *cardPtr, void *bufptr, s32 bufsize, FreePtr **fp, char *htBuf) {
	MmcblkCard_t *card = (MmcblkCard_t *) cardPtr;
	SdhcAdma2Desc_t *dt=NULL;
	u32 dtIdx = 0;
	assert(card != NULL);


	/* adma descriptor table has to be located in physically contignuous area, or partially contignuous area
	* ----------------------------
	* DATAPTR, DATA_LEN, TRAN
	* ----------------------------
	* ----------------------------
	* DATAPTR, DATA_LEN, TRAN
	* ----------------------------
	* ----------------------------
	* DESCPTR, NOT_RELEVANT, LINK
	* ----------------------------
	*    |
	*    |
	*    |
	* ----------------------------
	* DATAPTR, DATA_LEN, TRAN, END
	* ----------------------------
	*
	*
	* descriptor has 8 bytes, allocation forses alignment to bigger of:
	*  2*sizeof(descriptor) and SIZE_CACHE_LINE
	* so there always will be contignuous area for at leas two descriptors(TRAN and LINK)
	*/

	assert((htBuf != NULL)||(htBuf == NULL && !((u32)bufptr & (SIZE_CACHE_LINE-1))));

#define DT_BUCKET_SIZE (ADMA_DESC_ALIGN / sizeof(SdhcAdma2Desc_t))

	assert(fp != NULL);
	LOG("DMA bufsize: %d", bufsize);
	/* check buffer alignment */
	if((u32) bufptr & 0x3U) {
		LOG("Improper buffer alignment");
		return NULL;
	}

	if(bufsize != 0)
	{
		s32 tmp = bufsize;
		u32 ptr = 0;
		SdhcAdma2Desc_t *tmpdt;
		FreePtr *tmpfp;
		u32 tailLen=0;

		*fp = vm_kmalloc(sizeof(FreePtr));
		if(*fp == NULL)
			return NULL;
		tmpfp = *fp;

		tmpfp->freeptr = NULL;
		tmpfp->next = NULL;

		dt = vm_dokmallocaligned(sizeof(SdhcAdma2Desc_t) * DT_BUCKET_SIZE, ADMA_DESC_ALIGN, &tmpfp->freeptr);

		tmpdt = dt;
		if(dt == NULL)
			return NULL;

		memset(tmpdt, 0x0, sizeof(SdhcAdma2Desc_t) * DT_BUCKET_SIZE);

		vm_kmapResolve(bufptr, &ptr);

		if(htBuf != NULL)
		{
			tmpdt[dtIdx].desc = 0;
			tmpdt[dtIdx].bits.Act = 0x2U;  /* Data Transfer */
			tmpdt[dtIdx].bits.Valid = 0x1U;
			tmpdt[dtIdx].bits.DataLen = SIZE_CACHE_LINE - ((u32)bufptr & (SIZE_CACHE_LINE-1));
			vm_kmapResolve(htBuf, &tmpdt[dtIdx].bits.DataAddr);
			tailLen = (u32) bufptr & (SIZE_CACHE_LINE-1);
			bufptr += tmpdt[dtIdx].bits.DataLen;
			tmp -= tmpdt[dtIdx].bits.DataLen;
			hal_cpuFlushCache(&tmpdt[dtIdx], sizeof(SdhcAdma2Desc_t));
			LOG("DESC DATA LEN: %u",tmpdt[dtIdx].bits.DataLen );
			++dtIdx;
			tmp-=tailLen;
		}

		vm_kmapResolve(bufptr, &ptr);

		assert(!((u32)bufptr & (SIZE_CACHE_LINE - 1)));

		if((u32)bufptr & (SIZE_PAGE-1))//check if bufptr is on beginning of page
		{
			/* not on beginning */

			tmpdt[dtIdx].desc = 0;
			tmpdt[dtIdx].bits.Act = 0x2U;  /* Data Transfer */
			tmpdt[dtIdx].bits.Valid = 0x1U;

			tmpdt[dtIdx].bits.DataLen = (tmp > (SIZE_PAGE - ((u32)bufptr & (SIZE_PAGE-1)))) ? (SIZE_PAGE - ((u32)bufptr & (SIZE_PAGE-1))) : tmp;
			tmpdt[dtIdx].bits.DataAddr = ptr;


			hal_cpuFlushCache(&tmpdt[dtIdx], sizeof(SdhcAdma2Desc_t));
			bufptr += tmpdt[dtIdx].bits.DataLen;
			tmp -= tmpdt[dtIdx].bits.DataLen;
			if(tmp <= 0)
				LOG("DESC DATA LEN: %u",tmpdt[dtIdx].bits.DataLen );
		}

		while(tmp > 0)
		{
			int nextDesc = 0;
			u32 ptr2 = 0;

			vm_kmapResolve(bufptr, &ptr2);
			assert(!(ptr2 & (SIZE_PAGE-1)));
			if( (ptr2 <= (ptr & (~(SIZE_PAGE-1))) + SIZE_PAGE) && ptr2 > ptr )
			{
				u32 nextLen = ((tmp > SIZE_PAGE) ? SIZE_PAGE : tmp);
				if((0xFFFFU - tmpdt[dtIdx].bits.DataLen) >=  nextLen){
					tmpdt[dtIdx].bits.DataLen += nextLen;
					hal_cpuFlushCache(&tmpdt[dtIdx], sizeof(SdhcAdma2Desc_t));
					bufptr += nextLen;
					tmp -= nextLen;
				}
				else {
					LOG("DESC DATA LEN: %u",tmpdt[dtIdx].bits.DataLen );
					nextDesc = 1;
				}
			}
			else {
				nextDesc = 1;
				LOG("DESC DATA LEN: %u",tmpdt[dtIdx].bits.DataLen );
			}

			ptr = ptr2;


			if(nextDesc) {
				if(dtIdx == DT_BUCKET_SIZE - 2)
				{
					SdhcAdma2Desc_t *next=NULL;

					tmpdt[DT_BUCKET_SIZE-1].desc=0;
					tmpdt[DT_BUCKET_SIZE-1].bits.Valid = 0x1U;
					tmpdt[DT_BUCKET_SIZE-1].bits.Act = 0x3U; /* Link */
					tmpfp->next =  vm_kmalloc(sizeof(FreePtr));
					if(tmpfp->next == NULL) {
						card->port->ioOps.freeDMA(tmpfp);
						*fp = NULL;
						return NULL;
					}
					tmpfp = tmpfp->next;
					tmpfp->next = NULL;
					tmpfp->freeptr=NULL;
					next  = vm_dokmallocaligned(sizeof(SdhcAdma2Desc_t) * DT_BUCKET_SIZE, ADMA_DESC_ALIGN, &tmpfp->freeptr);

					if(next == NULL)
					{
						tmpfp->freeptr=NULL;
						card->port->ioOps.freeDMA(tmpfp);
						*fp = NULL;
						return NULL;
					}
					vm_kmapResolve(next, &tmpdt[DT_BUCKET_SIZE-1].bits.DataAddr);
					hal_cpuFlushCache(&tmpdt[DT_BUCKET_SIZE-1], sizeof(SdhcAdma2Desc_t));
					tmpdt = next;
					memset(tmpdt, 0x0, sizeof(SdhcAdma2Desc_t) * DT_BUCKET_SIZE);
					dtIdx = 0;

				}
				else
					++dtIdx;

				tmpdt[dtIdx].desc = 0;
				tmpdt[dtIdx].bits.Act = 0x2U;  /* Data Transfer */
				tmpdt[dtIdx].bits.Valid = 0x1U;
				tmpdt[dtIdx].bits.DataLen = (tmp > SIZE_PAGE) ? SIZE_PAGE : tmp;
				tmpdt[dtIdx].bits.DataAddr = ptr;
				hal_cpuFlushCache(&tmpdt[dtIdx], sizeof(SdhcAdma2Desc_t));
				bufptr += tmpdt[dtIdx].bits.DataLen;
				tmp -= tmpdt[dtIdx].bits.DataLen;
			}



			if(tmp <= 0)
				LOG("DESC DATA LEN: %u",tmpdt[dtIdx].bits.DataLen );
			if(tmp < 0)
				LOG("tmp < 0: %d", tmp);
		}
		if(tailLen > 0 && htBuf != NULL)
		{
			assert(dtIdx <= (DT_BUCKET_SIZE-2));
			++dtIdx;
			tmpdt[dtIdx].desc=0;
			tmpdt[dtIdx].bits.Act=0x2U;
			tmpdt[dtIdx].bits.Valid=0x1U;
			tmpdt[dtIdx].bits.DataLen = tailLen;
			vm_kmapResolve(htBuf + SIZE_CACHE_LINE, &tmpdt[dtIdx].bits.DataAddr);
			LOG("DESC DATA LEN: %u",tmpdt[dtIdx].bits.DataLen );
		}
		assert(tmp == 0);

		tmpdt[dtIdx].bits.Int = 1;
		tmpdt[dtIdx].bits.End = 1;
		hal_cpuFlushCache(&tmpdt[dtIdx], sizeof(SdhcAdma2Desc_t));
	}
	return dt;



}

int mmcblk_sdhc_switchHighSpeed(void *cardPtr, u32 baudrate) {
	MmcblkCard_t *card = (MmcblkCard_t *) cardPtr;
	void *freeptr=NULL;
	u8 *sfb=NULL;
	FreePtr *fp;
	void *dmaDesc = NULL;
	MmcblkResponse_t response;
	assert(card != NULL);

	if(MMCBLK_CSD20_GET_CSD_STRUCTURE(card->CSD.csd20))  {
		if(!(MMCBLK_CSD20_GET_CCC(card->CSD.csd20) & 0x200)) {
			LOG("High speed mode not supported");
			return -1;
		}
	}
	else {
		if(!(MMCBLK_CSD10_GET_CCC(card->CSD.csd20) & 0x200)) {
			LOG("High speed mode not supported");
			return -1;
		}
	}

	//4 - aligned buffer required, length - multiplicity of 512
	assert(!((u32)sfb & 0x3));

	sfb = vm_dokmallocaligned(64, SIZE_CACHE_LINE, &freeptr);
	if(sfb == NULL)
		return -ENOMEM;

	dmaDesc = card->port->ioOps.setupDMA(card, sfb, 64, &fp, NULL);
	if(dmaDesc == NULL) {
		vm_kfree(freeptr);
		return -ENOMEM;
	}

	card->port->ioOps.sendCommand(card, MMCBLK_COMM_SWITCH_FUNC, 0x00FFFFF1, 1, 64, dmaDesc);
	response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_SWITCH_FUNC);

	if(mmcblk_evaluateResponse(&response) != -EBUSY) {
		vm_kfree(freeptr);
		card->port->ioOps.freeDMA(fp);
		return -1;
	}

	card->port->ioOps.transferWait(card);

	hal_cpuInvalCache(sfb, 64);

	/*checking bit 401 of sfb buffer - it corresponds to high speed mode support status*/
	if(!(sfb[13] & 0x01))
	{
		LOG("HIGH SPEED MODE NOT SUPPORTED!");
		vm_kfree(freeptr);
		card->port->ioOps.freeDMA(fp);
		return -1;
	}
	else
		LOG("HIGH SPEED MODE SUPPORTED!");


	// Set the high speed of bus



	card->port->ioOps.sendCommand(card, MMCBLK_COMM_SWITCH_FUNC, 0x10FFFFF1, 1, 64, dmaDesc);
	response = card->port->ioOps.waitForResponse(card, MMCBLK_COMM_SWITCH_FUNC);

	if(mmcblk_evaluateResponse(&response) != -EBUSY)
	{
		LOG("FAILED TO SWITCH TO HIGH SPEED MODE!");
		vm_kfree(freeptr);
		card->port->ioOps.freeDMA(fp);
		return -1;
	}

	card->port->ioOps.transferWait(card);

	hal_cpuInvalCache(sfb, 64);

	if(!(sfb[13] & 0x01))
	{
		LOG("FAILED TO SWITCH TO HIGH SPEED MODE!");
		vm_kfree(freeptr);
		card->port->ioOps.freeDMA(fp);
		return -1;
	}
	else
		LOG("SWITCHED TO HIGH SPEED MODE!");
	card->baudRate = card->port->ioOps.setupBaudRate(card, baudrate);
	card->speed = eSDHighSpeed;

	vm_kfree(freeptr);
	card->port->ioOps.freeDMA(fp);

	return card->baudRate;
}

int mmcblk_sdhc_setupBaudRate(void *cardPtr, u32 baudrate) {
	MmcblkCard_t *card = (MmcblkCard_t *) cardPtr;
	u32 minpres = 0x80;
	u32 mindiv = 0x0F;
	u32 pres;
	u32 div;
	u32 mindiff = (u32) -1;
	s32 val;
	SDHC_Type *sdhc = NULL;
	assert(card != NULL);
	sdhc = card->port->ioBase;

	if(baudrate > MMCBLK_SDHC_MAX_BAUDRATE)
		baudrate = MMCBLK_SDHC_MAX_BAUDRATE;

	for(pres = 2; pres <= 256; pres <<= 1)
	{
		for(div=1; div <= 16; ++div)
		{
			val = pres * div * baudrate - BUS_CLK_KHZ * 1000UL;
			if(val < mindiff)
			{
				minpres = pres;
				mindiv = div;
				mindiff = val;
			}
		}
	}

	LOG("Pres: %u Div: %u", minpres, mindiv);

	/* Disable ESDHC clocks */
	sdhc->SYSCTL &= (~ SDHC_SYSCTL_SDCLKEN_MASK);

	/* Change dividers */
	div = sdhc->SYSCTL & (~ (SDHC_SYSCTL_DTOCV_MASK | SDHC_SYSCTL_SDCLKFS_MASK | SDHC_SYSCTL_DVS_MASK));
	sdhc->SYSCTL = div | (SDHC_SYSCTL_DTOCV(0x0E) | SDHC_SYSCTL_SDCLKFS(minpres >> 1) | SDHC_SYSCTL_DVS(mindiv - 1));

	/* Wait for stable clock */
	while (0 == (sdhc->PRSSTAT & SDHC_PRSSTAT_SDSTB_MASK))
	{
		proc_threadSleep(100);
#if defined (M53015EVB) || (MPC8377RDB)
		break;
#endif
	};

	/* poll bits CIHB and CDIHB bits of PRSSTAT to wait both bits are cleared */
	while(sdhc->PRSSTAT &  (SDHC_PRSSTAT_CIHB_MASK | SDHC_PRSSTAT_CDIHB_MASK))
		hal_cpuReschedule();

	/* Enable ESDHC clocks */
	sdhc->SYSCTL |= SDHC_SYSCTL_SDCLKEN_MASK | SDHC_SYSCTL_PEREN_MASK;
	sdhc->IRQSTAT |= SDHC_IRQSTAT_DTOE_MASK;

	return (BUS_CLK_KHZ * 1000UL) / (minpres*mindiv);

}

void mmcblk_sdhc_reset(void *cardPtr) {
	MmcblkCard_t *card = (MmcblkCard_t *) cardPtr;
	SDHC_Type *sdhc = NULL;
	assert(card != NULL);
	sdhc = card->port->ioBase;

	LOG("Sdhc reset");

	card->label = eCardUnknown;
	card->CID.cid[0]=0;
	card->CID.cid[1]=0;
	card->CID.cid[2]=0;
	card->CID.cid[3]=0;
	card->CSD.csd10.csd[0]=0;
	card->CSD.csd10.csd[1]=0;
	card->CSD.csd10.csd[2]=0;
	card->CSD.csd10.csd[3]=0;
	card->OCR=0;

	card->busWidth= eMmcblkBusWidth1b;
	card->speed=eBasicSpeed;
	card->RCA=0;
	card->voltage=0;

	sdhc->WML = (1 << SDHC_WML_RDWML_SHIFT) | (1 << SDHC_WML_WRWML_SHIFT) | (1 << SDHC_WML_RDBRSTLEN_SHIFT) | (1 << SDHC_WML_WRBRSTLEN_SHIFT);

	sdhc->BLKATTR = (1 << SDHC_BLKATTR_BLKCNT_SHIFT) | (64 << SDHC_BLKATTR_BLKSIZE_SHIFT);
	sdhc->VENDOR = 0;
	sdhc->PROCTL = (2 << SDHC_PROCTL_EMODE_SHIFT) | (2 << SDHC_PROCTL_DMAS_SHIFT);

	card->port->ioOps.setupBaudRate(card, 400000U);

	sdhc->IRQSTAT = 0xFFFF;

	sdhc->IRQSTATEN = 0;

	sdhc->IRQSTATEN |= SDHC_IRQSTATEN_CCSEN_MASK;/* command completed */
	sdhc->IRQSTATEN |= SDHC_IRQSTATEN_TCSEN_MASK;/* transfer completed */

	sdhc->IRQSTATEN |= SDHC_IRQSTATEN_DINTSEN_MASK;/* DMA interrupt */
	sdhc->IRQSTATEN |= SDHC_IRQSTATEN_DMAESEN_MASK;/* DMA error */

	sdhc->IRQSTATEN |= SDHC_IRQSTATEN_CTOESEN_MASK;/* command timeout error */
	sdhc->IRQSTATEN |= SDHC_IRQSTATEN_DTOESEN_MASK;/* data timeout error */

	sdhc->IRQSTATEN |= SDHC_IRQSTATEN_CCESEN_MASK;/* command crc error */
	sdhc->IRQSTATEN |= SDHC_IRQSTATEN_CINTSEN_MASK;/* card interrupt error */

	sdhc->IRQSTATEN |= SDHC_IRQSTATEN_AC12ESEN_MASK;/* Auto CMD12 error */
	sdhc->IRQSTATEN |= SDHC_IRQSTATEN_DEBESEN_MASK;/* Data end bit error */
	sdhc->IRQSTATEN |= SDHC_IRQSTATEN_DCESEN_MASK;/* Data CRC error */
	sdhc->IRQSTATEN |= SDHC_IRQSTATEN_CIESEN_MASK;/* Command index error */
	sdhc->IRQSTATEN |= SDHC_IRQSTATEN_CEBESEN_MASK;/* Command end bit error */
	sdhc->IRQSTATEN |= SDHC_IRQSTATEN_BGESEN_MASK;/* Block gap event */


	sdhc->IRQSIGEN = sdhc->IRQSTATEN;

	sdhc->SYSCTL &= ~SDHC_SYSCTL_DTOCV_MASK;
	sdhc->SYSCTL |= (0xCU << SDHC_SYSCTL_DTOCV_SHIFT);

	/* send 80 clock ticks for card to power up */
	sdhc->SYSCTL |= SDHC_SYSCTL_INITA_MASK;
	/* TODO - timeout */
	while(sdhc->SYSCTL & SDHC_SYSCTL_INITA_MASK)
		proc_threadSleep(100);

	/* reset the card with CMD0 */
	card->port->ioOps.sendCommand(card, MMCBLK_COMM_GO_IDLE_STATE, 0, 0, 0, NULL);
	card->port->ioOps.waitForResponse(card, MMCBLK_COMM_GO_IDLE_STATE);
	LOG("Card in idle state");
	//TODO - mark card as uninitialized
	proc_threadSleep(1000 * 1000);

}


int mmcblk_sdhc_setupBusWidth(void *cardPtr, MmcblkBusWidth_t width)
{
	MmcblkCard_t *card = (MmcblkCard_t *) cardPtr;
	SDHC_Type *sdhc = NULL;
	assert(card != NULL);
	sdhc = card->port->ioBase;

	sdhc->PROCTL &= ~SDHC_PROCTL_DTW_MASK;
	switch(width) {
		case eMmcblkBusWidth1b:
			sdhc->PROCTL |= SDHC_PROCTL_DTW(0);
		break;
		case eMmcblkBusWidth4b:
			sdhc->PROCTL |= SDHC_PROCTL_DTW(1);
		break;
		case eMmcblkBusWidth8b:
			sdhc->PROCTL |= SDHC_PROCTL_DTW(2);
		break;
	}
	return 0;
}

int mmcblk_sdhc_setupEndian(void *cardPtr, MmcblkEndian_t endian)
{
	MmcblkCard_t *card = (MmcblkCard_t *) cardPtr;
	SDHC_Type *sdhc = NULL;
	assert(card != NULL);
	sdhc = card->port->ioBase;

	sdhc->PROCTL &= ~SDHC_PROCTL_EMODE_MASK;
	switch(endian) {
		case eLittleEndian:
			sdhc->PROCTL |= (2 << SDHC_PROCTL_EMODE_SHIFT);
		break;
		case eBigEndian:
			sdhc->PROCTL |= (0 << SDHC_PROCTL_EMODE_SHIFT);
		break;
		case eBigEndianHalfWords:
			sdhc->PROCTL |= (1 << SDHC_PROCTL_EMODE_SHIFT);
		break;
		default:
			return -1;
	}
	return 0;
}
