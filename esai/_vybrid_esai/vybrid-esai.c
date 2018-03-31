/**
 * Enhanced Serial Audio Interface (ESAI) implementation for Vybrid
 *
 * Phoenix-RTOS
 * 
 * Operating system kernel
 *
 * @file esai.c
 *
 * @copyright 2014 Phoenix Systems
 *
 * @author Pawel Tryfon<pawel.tryfon@phoesys.com>
 * @author Pawel Kolodziej<pawel.kolodziej@phoesys.com>
 * 
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/if.h>
#include <hal/MVF50GS10MK50.h>
#include <vm/if.h>
#include <dev/dev.h>
#include <dev/vybrid-i2c/if.h>
#include <dev/vybrid-dma/dma.h>
#include <main/if.h>
#include <include/esaictl.h>
#include "esai.h"

#define ESAI_PIT 5
#define ESAI_BUFFER_LEN ((u32)(4 * 960*8))
#define ESAI_CITER_AHEAD (128)
#define ESAI_DMA_SOURCE 35

#define ESAI_CHANNELS 8

#define DEBUG_CITER 0

#if DEBUG_CITER
void debugCiter(void);
#endif

static struct {
	ESAI_Type* base;
	s32* buf;
	page_t* bufPages;
	u8 dmaChannel;
	u32 readI;
	s32 readCycles;
	s32 remain;
	s32 balance;
	u8 busy;
	mutex_t   ulock;
	spinlock_t lock;
} esai_g;

void _esai_enable(void);

typedef struct {
	s32 *bufMapping;
} EsaiPrv_t;


int esai_ioctl(file_t* file, unsigned int cmd, unsigned long arg) {
	int ret = 0;
	assert(file != NULL);
	assert(file->priv != NULL);
	assert(file->vnode != NULL);
	assert(file->vnode->type == vnodeDevice);
	assert(MAJOR(file->vnode->dev)==MAJOR_ESAI);

	EsaiPrv_t *priv = (EsaiPrv_t *) file->priv;

	switch(cmd) {
		case ESAI_MAP_BUFFER: {
			if(priv->bufMapping != NULL)
				ret = -EEXIST;
			else {
				s32 ** map = (s32 **) arg;
				ret = vm_map_anon(esai_g.bufPages, (PGHD_PRESENT | PGHD_USER | PGHD_DEV | PGHD_READ), (void**)map);
				if(ret == EOK)
					priv->bufMapping = *map;
			}
		}
		break;
		case ESAI_UNMAP_BUFFER: {
			if(priv->bufMapping != NULL) {
				vm_unmap_anon(priv->bufMapping);
				priv->bufMapping=NULL;
			}
		}
		break;
		case ESAI_CHECK_SAMPLE_AVAIL: {
			s32 * r = (s32*)arg;
			*r = esai_available();
		}
		break;
		case ESAI_READ_BUF_PTR: {
			EsaiReadBuf_t *erb = (EsaiReadBuf_t *) arg;
			ret = esai_readBuf( erb->count, &erb->buf);
		}
		break;
		case ESAI_READ_START: {
			esai_readBufStart();
		}
		break;
		default:
			ret = -EINVAL;
	}


	return ret;
}

int esai_open(vnode_t *vnode, file_t* file) {
	int ret = 0;
	assert(vnode != NULL);
	assert(vnode->type == vnodeDevice);
	assert(MAJOR(vnode->dev) == MAJOR_ESAI );

	proc_mutexLock(&esai_g.ulock);

	if(!esai_g.busy) {

		file->priv = vm_kmalloc(sizeof(EsaiPrv_t));
		if(file->priv == NULL)
			ret = -ENOMEM;
		else {
			memset(file->priv, 0x0, sizeof(EsaiPrv_t));
			esai_g.busy = 1;
		}
	}
	else
		ret = -EBUSY;

	proc_mutexUnlock(&esai_g.ulock);

	return ret;
}

int esai_close(vnode_t *vnode, file_t* file) {
	int ret = 0;
	assert(vnode != NULL);
	assert(vnode->type == vnodeDevice);
	assert(MAJOR(vnode->dev) == MAJOR_ESAI );
	assert(file != NULL);
	assert(file->priv != NULL);

	proc_mutexLock(&esai_g.ulock);

	if(!esai_g.busy)
		ret = -EIO;
	else {
		EsaiPrv_t * prv = (EsaiPrv_t *) file->priv;
		if(prv->bufMapping != NULL) {
			vm_unmap_anon(prv->bufMapping);
			prv->bufMapping = NULL;
		}
		vm_kfree(file->priv);
		file->priv = NULL;
		esai_g.busy = 0;
	}

	proc_mutexUnlock(&esai_g.ulock);

	return ret;
}

int _esai_init(void)
{
	const static file_ops_t fops = {
		.ioctl = esai_ioctl,
		.open = esai_open,
		.close = esai_close
	};

	if(vm_iomap((addr_t)ESAI_BASE,sizeof(ESAI_Type),PGHD_DEV_RW,(void**)&esai_g.base) < 0)
		return -1;

	esai_g.bufPages = vm_pageAlloc(((sizeof(s32) * ESAI_BUFFER_LEN) + (SIZE_PAGE - 1)) / SIZE_PAGE, vm_pageAlloc);

	if(esai_g.bufPages == NULL) {
		vm_iounmap(esai_g.base,sizeof(ESAI_Type));
		return -1;
	}
	if(vm_kmap(esai_g.bufPages, PGHD_DEV_RW, (void **) &esai_g.buf) < 0) {
		vm_pageFree(esai_g.bufPages);
		esai_g.bufPages = NULL;
		vm_iounmap(esai_g.base,sizeof(ESAI_Type));
		return -1;
	}
	if(dev_register(MAKEDEV(MAJOR_ESAI, 0), &fops)) {
		main_printf(ATTR_ERROR, "Failed to register driver for /dev/esai");
		vm_kunmap(esai_g.buf);
		esai_g.buf=NULL;
		vm_pageFree(esai_g.bufPages);
		esai_g.bufPages = NULL;
		vm_iounmap(esai_g.base,sizeof(ESAI_Type));
		return -1;
	}

	memset(esai_g.buf,0xAB,sizeof(s32) * ESAI_BUFFER_LEN);
	proc_spinlockCreate(&esai_g.lock, "esai.lock");
	proc_mutexCreate(&esai_g.ulock);

	assert(EOK==dev_mknod(MAKEDEV(MAJOR_ESAI, 0), "esai"));

	main_printf(ATTR_INFO,"dev: esai initialized\n");
	_esai_enable();

	return EOK;
}


void _esai_enable(void)
{
	int i,rc;
	u8 dmaHandle = 0;
	esai_g.base->ECR = ESAI_ECR_ESAIEN_MASK;//Enable ESAI	
	for(i = 0;i < 100000;i++); //FIXME
	esai_g.base->ECR |= ESAI_ECR_ERST_MASK;
	for(i = 0;i < 100000;i++); // FIXME
	esai_g.base->ECR &= ~ESAI_ECR_ERST_MASK;
	for(i = 0;i < 100000;i++); //FIXME

	esai_g.base->ECR |= ESAI_ECR_ERI_MASK;
	esai_g.base->RFCR = ESAI_RFCR_RFR_MASK; //Reset receiver FIFO
	esai_g.base->TFCR = ESAI_TFCR_TFR_MASK; //Reset receiver FIFO
	esai_g.base->RCR = ESAI_RCR_RPR_MASK; // Reset receiver
	for(i = 0;i < 100000;i++); //FIXME

	esai_g.base->RCR &= ~ESAI_RCR_RPR_MASK;
	esai_g.base->RFCR &= ~ESAI_RFCR_RFR_MASK;
	esai_g.base->TFCR &= ~ESAI_TFCR_TFR_MASK;

	esai_g.base->RCR |= ESAI_RCR_RMOD(0x1) | ESAI_RCR_RSWS(0x1F) /* | ESAI_RCR_RFSL_MASK */  | ESAI_RCR_RFSR_MASK;
	int rfp = 1 ; // 48k -> 1  12k -> 7
	esai_g.base->RCCR =  ESAI_RCCR_RFSD_MASK | ESAI_RCCR_RCKD_MASK | ESAI_RCCR_RHCKD_MASK //| ESAI_RCCR_RCKP_MASK
		| ESAI_RCCR_RFP(rfp) | ESAI_RCCR_RDC(3) | ESAI_RCCR_RPSP_MASK | ESAI_RCCR_RPM(0x0)  //| ESAI_RCCR_RHCKP_MASK
		; 
	esai_g.base->RFCR = ESAI_RFCR_RFE_MASK | ESAI_RFCR_REXT_MASK | ESAI_RFCR_RWA(0x2) 
		| ESAI_RFCR_RE0_MASK | ESAI_RFCR_RE1_MASK | ESAI_RFCR_RFWM(1); // RE0 | RE2

	esai_g.base->PRRC = ESAI_PRRC_PDC_MASK;
	esai_g.base->PCRC = ESAI_PCRC_PC_MASK; 

	/* Initialize DMA */
	if((rc = dma_allocChannel(dmaHandle,1,&esai_g.dmaChannel)) < 0)
		main_printf(ATTR_ERROR,"dev: [_esai_init] dma channel allocation failed\n");;
	if(rc == EOK) {
		addr_t phaddr;
		vm_kmapResolve((void*)&esai_g.base->ERDR,&phaddr);
		dma_setTcdSaddr(dmaHandle,esai_g.dmaChannel,phaddr);
		vm_kmapResolve(esai_g.buf,&phaddr);
		dma_setTcdDaddr(dmaHandle,esai_g.dmaChannel,phaddr);
		dma_setTcdSoff(dmaHandle,esai_g.dmaChannel,0);
		dma_setTcdDoff(dmaHandle,esai_g.dmaChannel,sizeof(u32));
		dma_setTcdAccessSize(dmaHandle,esai_g.dmaChannel,sizeof(u32));
		dma_setTcdMinorCount(dmaHandle,esai_g.dmaChannel,sizeof(u32));
		dma_setTcdSlast(dmaHandle,esai_g.dmaChannel,0);
		dma_setTcdDlast(dmaHandle,esai_g.dmaChannel,-1 * sizeof(u32) * ESAI_BUFFER_LEN);
		dma_setTcdBiter(dmaHandle,esai_g.dmaChannel,ESAI_BUFFER_LEN);
		rc = dma_enableSource(dmaHandle,esai_g.dmaChannel,ESAI_DMA_SOURCE);
	}
	
	if(rc != EOK)
		main_printf(ATTR_ERROR,"dev: [_esai_enable] enabling dma channel failed\n");
	if((rc = dma_getError(dmaHandle)) != 0)
		main_printf(ATTR_ERROR,"dev: [_esai_enable] dma error=%x\n",rc);
	esai_g.base->RCR |= ESAI_RCR_RE0_MASK | ESAI_RCR_RE1_MASK;

	hal_pitSetupClocks(ESAI_PIT, 0xffffffff, NULL, NULL);
}

static u32 _citer2i(u32 citer)
{
	s32 i = ESAI_BUFFER_LEN - citer - ESAI_CITER_AHEAD;
	if( i < 0)
		i+=ESAI_BUFFER_LEN;
	assert(i>= 0);
	assert(i <  ESAI_BUFFER_LEN);
	return i;
}

void esai_readBufStart(void)
{
	proc_spinlockSet(&esai_g.lock);
	esai_g.readI = _citer2i(dma_getCiter(0, esai_g.dmaChannel))/ESAI_CHANNELS*ESAI_CHANNELS;
	esai_g.readCycles = hal_pitGetTime(ESAI_PIT);
	proc_spinlockClear(&esai_g.lock, sopGetCycles);

	esai_g.remain = 0;
	esai_g.balance = 0;
}


int esai_available(void)
{
	int begin = esai_g.readI; // first byte to read;
	int end = _citer2i(dma_getCiter(0, esai_g.dmaChannel));
	if( begin <= end)
		return (end - begin);
	else
		return (ESAI_BUFFER_LEN - begin + end); 
}

/** request to read samples.
 * After return buf points to data buffer
 * \return number of samplesSets that can (and must) be read from buffer. 
 *  */
#define DATA2DT (BUS_CLK_KHZ/48)
//BUS_CLK_KHZ = 66000 XXX
int esai_readBuf(int samples, s32 **buf)
{
	assert(samples);
	assert(samples % ESAI_CHANNELS == 0);
	int begin = esai_g.readI; // first byte to read;
	int end;
	int len;
	cycles_t now;
	int dt;


	proc_spinlockSet(&esai_g.lock);
	end = _citer2i(dma_getCiter(0, esai_g.dmaChannel));
	now =  hal_pitGetTime(ESAI_PIT);
	proc_spinlockClear(&esai_g.lock, sopGetCycles);

	dt = (int32_t)now - (int32_t)(esai_g.readCycles);
	if(dt<0)
		main_printf(ATTR_ERROR,"ESAI dt=%d < 0\n", dt);
	assert(dt>=0);
    int newData = (dt / DATA2DT) * ESAI_CHANNELS;
	if (! (esai_g.balance + newData < ESAI_BUFFER_LEN*7/8 - ESAI_CITER_AHEAD))
		main_printf(ATTR_ERROR,"ESAI low space in buffer: balance = %d, newData = %d dt=%d aval=%d\n", esai_g.balance, newData, dt, esai_available());
	if(!(esai_g.balance + newData > -64))
        main_printf(ATTR_ERROR,"ESAI !(balance + newData > -64):  balance = %d, newData = %d\n", esai_g.balance, newData);

	assert(esai_g.balance + newData < ESAI_BUFFER_LEN*7/8 - ESAI_CITER_AHEAD); // assert that at least 1/8 of buffer is free
    assert(esai_g.balance + newData > -64);
	assert(samples > 0);
	//esai_g.readCycles = now - (dt - (newData * DATA2DT)/ESAI_CHANNELS) ;
	esai_g.readCycles += (newData * DATA2DT)/ESAI_CHANNELS ;

	if( begin <= end)
		len = end - begin;
	else
		len  = ESAI_BUFFER_LEN - begin; // read to the end of buffer
	
	
    len = (len / ESAI_CHANNELS) * ESAI_CHANNELS; // allways read whole set of channels!
	if(len > samples)
		len = samples;

	esai_g.readI = (begin + len) % ESAI_BUFFER_LEN;
	*buf = esai_g.buf + begin;
	assert(*buf + len <=  esai_g.buf + ESAI_BUFFER_LEN);

    esai_g.balance += newData - len ;
	static int done = 0;
	done+=len;
	if(0 && done > 100000){
		main_printf(ATTR_INFO, "r: %d / %d \t(%d)\t %d   \n", len, newData, newData - len, esai_g.balance );
		done=0;
	}

#if DEBUG_CITER
	debugCiter();
#endif

	hal_cpuInvalCache(*buf, len * 4);
	return len;
}


#if DEBUG_CITER

void debugCiter(void)
{
	static int isInitialized = 0;
	static int lastCiter = 0;  
	static int lastPit = 0;
	static char msg[300];
	static int64_t totalCiter=0;
	static int64_t sumPit=0;
	static int64_t lastPrint = 0;
	int64_t totalSamplesPit;
	if(!isInitialized){
		isInitialized = 1;
		msg[0]=0;
		lastCiter = dma_getCiter(0, esai_g.dmaChannel);
		lastPit   = hal_pitGetTime(ESAI_PIT);
		return;
	}

	int citer;
	int pit;

	proc_spinlockSet(&esai_g.lock);
	pit =  hal_pitGetTime(ESAI_PIT);
	citer = dma_getCiter(0, esai_g.dmaChannel);
	proc_spinlockClear(&esai_g.lock, sopGetCycles);

	assert( citer != 0);
	citer --;

	int dpit = ((int32_t) pit - (int32_t) lastPit) * (CPU_CLK_KHZ / BUS_CLK_KHZ);
	int citerDiff=0;
	if( citer <= lastCiter)
		citerDiff = lastCiter - citer;
	else
		citerDiff = lastCiter + ESAI_BUFFER_LEN - citer;

	totalCiter += citerDiff;
	sumPit += ((int32_t) pit - (int32_t) lastPit);
	totalSamplesPit = sumPit *(CPU_CLK_KHZ / BUS_CLK_KHZ) *10/6875;
	if(lastPrint + 48000 *8 < totalSamplesPit){
        main_printf(ATTR_INFO, "SUM: pit %10u totalCiter %8lld, tootalSamplesPit: %8lld, diff: %5lld, balance: %4d, available: %d\n", pit, totalCiter, totalSamplesPit, totalCiter - totalSamplesPit, esai_g.balance, esai_available());
		lastPrint = totalSamplesPit;
	}
	int diffSamplesPit = dpit * 10 / 6875;
	if ( diffSamplesPit < citerDiff - 10 || diffSamplesPit > citerDiff + 10 ){
		main_printf(ATTR_INFO,"LAST:[ %s ]\n", msg);
        main_printf(ATTR_INFO,"NOW:   pit: %8d , totalSamplesPit: %8lld , totalCiter: %8lld , diffSamplesPit %8d , citerDiff: %8d , lastPit %8d  lastCiter %8d\n", pit, totalSamplesPit, totalCiter, diffSamplesPit, citerDiff, lastPit, lastCiter);
		
	}else{
        main_snprintf(msg, 300,"pit: %8d , totalSamplesPit: %8lld , totalCiter: %8lld , diffSamplesPit %8d , citerDiff: %8d , lastPit %8d  lastCiter %8d", pit, totalSamplesPit, totalCiter, diffSamplesPit, citerDiff, lastPit, lastCiter);
	}

	lastCiter = citer;
	lastPit = pit;
}
#endif


