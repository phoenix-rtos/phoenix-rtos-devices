#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <sys/interrupt.h>
#include <sys/mman.h>
#include <sys/threads.h>
#include <sys/types.h>

#include <phoenix/arch/imxrt.h>

#include <edma.h>

#include "../ad7779.h"

#define DMA_MAX_BUFFERS (16)

#define SAI1_RX_DMA_REQUEST (19)
#define SAI1_RX_DMA_CHANNEL (7)

#define TCD_CSR_INTMAJOR_BIT (1 << 1)
#define TCD_CSR_ESG_BIT      (1 << 4)

#define OCRAM2_BASE (0x20200000)
#define OCRAM2_END  (0x2027FFFF)

struct {
	struct {
		handle_t cond;
		handle_t lock;
		handle_t handle;
		volatile uint32_t cnt;
	} irq;
	size_t size;
	addr_t ocram_ptr;
	volatile struct edma_tcd_s tcds[DMA_MAX_BUFFERS];
} edma = {
	.ocram_ptr = OCRAM2_BASE,
};

static int edma_error_handler(unsigned int n, void *arg)
{
	/* TODO: Store some info for debugging? Notify about it somehow? */
	sai_rx_disable();
	edma_clear_error(SAI1_RX_DMA_CHANNEL);

	return 0;
}

static int edma_irq_handler(unsigned int n, void *arg)
{
	edma.irq.cnt += 1;

	edma_clear_interrupt(SAI1_RX_DMA_CHANNEL);
	return 0;
}

/* Allocating OCRAM2 from the end */
static addr_t ocram_alloc(size_t size)
{
	unsigned n = (size + _PAGE_SIZE - 1) / _PAGE_SIZE;
	addr_t paddr = 0;

	if (edma.ocram_ptr + n * _PAGE_SIZE <= OCRAM2_END) {
		paddr = edma.ocram_ptr;
		edma.ocram_ptr += n * _PAGE_SIZE;
	}

	return paddr;
}

static void *alloc_uncached(size_t size, addr_t *paddr, int ocram)
{
	uint32_t n = (size + _PAGE_SIZE - 1) / _PAGE_SIZE;
	oid_t *oid = OID_NULL;
	addr_t _paddr = 0;

	if (ocram) {
		oid = OID_PHYSMEM;
		_paddr = ocram_alloc(n * _PAGE_SIZE);
		if (!_paddr)
			return NULL;
	}

	void *vaddr = mmap(NULL, n * _PAGE_SIZE,
		PROT_READ | PROT_WRITE, MAP_UNCACHED, oid, _paddr);
	if (vaddr == MAP_FAILED)
		return NULL;

	if (!ocram)
		_paddr = va2pa(vaddr);

	if (paddr != NULL)
		*paddr = _paddr;

	return vaddr;
}

static int free_uncached(void *vaddr, size_t size)
{
	return munmap(vaddr, (size + _PAGE_SIZE - 1) / _PAGE_SIZE * _PAGE_SIZE);
}

static int edma_configure(size_t size, size_t count, addr_t *paddr)
{
	int res;

	if ((res = mutexCreate(&edma.irq.lock)) != EOK) {
		log_error("mutex resource creation failed");
		return res;
	}

	if ((res = condCreate(&edma.irq.cond)) != EOK) {
		log_error("conditional resource creation failed");
		resourceDestroy(edma.irq.lock);
		return res;
	}

	void *buf = alloc_uncached(size, paddr, 0);

	if (buf == NULL) {
		log_error("edma buffers allocation failed");
		return -ENOMEM;
	}

	memset(buf, 0, size);

	uint8_t xfer_size = sizeof(uint32_t);

	edma.tcds[0].soff = 0;
	edma.tcds[0].attr = (edma_get_tcd_attr_xsize(xfer_size) << 8) |
		edma_get_tcd_attr_xsize(xfer_size);

	/* Number of bytes per minor loop iteration */
	edma.tcds[0].nbytes_mlnoffno = xfer_size * AD7779_NUM_OF_CHANNELS;
	edma.tcds[0].slast = 0;
	edma.tcds[0].doff = xfer_size;
	edma.tcds[0].dlast_sga = (uint32_t)&edma.tcds[1];

	/* Number of major loop iterations */
	edma.tcds[0].biter_elinkno =
		size / edma.tcds[0].nbytes_mlnoffno / count;
	edma.tcds[0].citer_elinkno = edma.tcds[0].biter_elinkno;

	/* Set addrs for the TCD. */
	edma.tcds[0].saddr = sai_get_rx_fifo_ptr();
	edma.tcds[0].daddr = (uint32_t)buf;

	/* Enable major loop finish interrupt and scatter-gather */
	edma.tcds[0].csr = TCD_CSR_INTMAJOR_BIT | TCD_CSR_ESG_BIT;

	int i;

	for (i = 1; i < count; ++i) {
		edma_copy_tcd(&edma.tcds[0], &edma.tcds[i]);
		edma.tcds[i].daddr = (uint32_t)buf + i * size / count;
		edma.tcds[i].dlast_sga = (uint32_t)&edma.tcds[(i + 1) % count];
	}

	if ((res = edma_install_tcd(&edma.tcds[0], SAI1_RX_DMA_CHANNEL)) != 0) {
		free_uncached(buf, size);
		return res;
	}

	interrupt(EDMA_CHANNEL_IRQ(SAI1_RX_DMA_CHANNEL),
		edma_irq_handler, NULL, edma.irq.cond, &edma.irq.handle);

	dmamux_set_source(SAI1_RX_DMA_CHANNEL, SAI1_RX_DMA_REQUEST);
	dmamux_channel_enable(SAI1_RX_DMA_CHANNEL);
	edma_channel_enable(SAI1_RX_DMA_CHANNEL);

	edma.size = size;

	return 0;
}

void dma_enable(void)
{
	edma_channel_enable(SAI1_RX_DMA_CHANNEL);
}

void dma_disable(void)
{
	edma_channel_disable(SAI1_RX_DMA_CHANNEL);
}

int dma_read(void *data, size_t len)
{
	int res = EOK;

	if (data != NULL && len != sizeof(unsigned))
		return -EIO;

	mutexLock(edma.irq.lock);
	if ((res = condWait(edma.irq.cond, edma.irq.lock, 1000000)) == 0)
		*(uint32_t **)data = (uint32_t *)&edma.irq.cnt;
	mutexUnlock(edma.irq.lock);

	return res;
}

void dma_free(void)
{
	free_uncached((void *)edma.tcds[0].daddr, edma.size);
}

int dma_init(size_t size, size_t count, addr_t *phys_addr)
{
	int res = EOK;

	if ((res = edma_init(edma_error_handler)) != 0) {
		log_error("failed to initialize edma");
		return res;
	}

	if ((res = edma_configure(size, count, phys_addr)) != 0) {
		log_error("failed to configure edma");
		return res;
	}


	return res;
}
