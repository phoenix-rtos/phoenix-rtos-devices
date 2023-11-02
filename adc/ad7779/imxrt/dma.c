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


static struct {
	struct {
		handle_t cond;
		handle_t lock;
		handle_t handle;
		volatile uint32_t cnt;
	} irq;
	struct {
		size_t size;
		size_t count;
		void *ptr;
	} buffer;
	volatile struct edma_tcd_s tcds[AD7779_BUFFER_CNT];
} edma_common;


static int edma_error_handler(unsigned int n, void *arg)
{
	/* TODO: Store some info for debugging? Notify about it somehow? */
	const uint32_t mask = 1U << SAI1_RX_DMA_CHANNEL;

	if (edma_error_channel() & mask) {
		sai_rx_disable();
		edma_clear_error(SAI1_RX_DMA_CHANNEL);
	}

	return 0;
}


static int edma_irq_handler(unsigned int n, void *arg)
{
	edma_common.irq.cnt += 1;

	edma_clear_interrupt(SAI1_RX_DMA_CHANNEL);
	return 0;
}


static void *alloc_uncached(size_t size, addr_t *paddr)
{
	uint32_t n = (size + _PAGE_SIZE - 1) / _PAGE_SIZE;
	addr_t _paddr = 0;

	void *vaddr = mmap(NULL, n * _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_ANONYMOUS, -1, _paddr);
	if (vaddr == MAP_FAILED)
		return NULL;

	if (paddr != NULL)
		*paddr = va2pa(vaddr);

	return vaddr;
}


static int free_uncached(void *vaddr, size_t size)
{
	return munmap(vaddr, (size + _PAGE_SIZE - 1) / _PAGE_SIZE * _PAGE_SIZE);
}


static int edma_configure(size_t size, size_t count, addr_t *paddr)
{
	int res, i;
	void *buf;
	uint8_t xfer_size;

	if ((res = mutexCreate(&edma_common.irq.lock)) != EOK) {
		log_error("mutex resource creation failed");
		return res;
	}

	if ((res = condCreate(&edma_common.irq.cond)) != EOK) {
		log_error("conditional resource creation failed");
		resourceDestroy(edma_common.irq.lock);
		return res;
	}

	buf = alloc_uncached(size * count, paddr);
	if (buf == NULL) {
		log_error("edma buffers allocation failed");
		resourceDestroy(edma_common.irq.lock);
		resourceDestroy(edma_common.irq.cond);
		return -ENOMEM;
	}

	memset(buf, 0, size * count);

	xfer_size = sizeof(uint32_t);

	edma_common.tcds[0].soff = 0;
	edma_common.tcds[0].attr = (edma_get_tcd_attr_xsize(xfer_size) << 8) |
		edma_get_tcd_attr_xsize(xfer_size);

	/* Number of bytes per minor loop iteration */
	edma_common.tcds[0].nbytes_mlnoffno = xfer_size * AD7779_NUM_OF_CHANNELS;
	edma_common.tcds[0].slast = 0;
	edma_common.tcds[0].doff = xfer_size;
	edma_common.tcds[0].dlast_sga = (uint32_t)&edma_common.tcds[1];

	/* Number of major loop iterations */
	edma_common.tcds[0].biter_elinkno =
		size / edma_common.tcds[0].nbytes_mlnoffno;
	edma_common.tcds[0].citer_elinkno = edma_common.tcds[0].biter_elinkno;

	/* Set addrs for the TCD. */
	edma_common.tcds[0].saddr = sai_fifo_rx_ptr();
	edma_common.tcds[0].daddr = (uint32_t)buf;

	/* Enable major loop finish interrupt and scatter-gather */
	edma_common.tcds[0].csr = TCD_CSR_INTMAJOR_BIT | TCD_CSR_ESG_BIT;

	for (i = 1; i < count; ++i) {
		edma_copy_tcd(&edma_common.tcds[i], &edma_common.tcds[0]);
		edma_common.tcds[i].daddr = (uint32_t)buf + i * size;
		edma_common.tcds[i].dlast_sga = (uint32_t)&edma_common.tcds[(i + 1) % count];
	}

	if ((res = edma_install_tcd(&edma_common.tcds[0], SAI1_RX_DMA_CHANNEL)) != 0) {
		free_uncached(buf, size * count);
		resourceDestroy(edma_common.irq.lock);
		resourceDestroy(edma_common.irq.cond);
		return res;
	}

	interrupt(EDMA_CHANNEL_IRQ(SAI1_RX_DMA_CHANNEL),
		edma_irq_handler, NULL, edma_common.irq.cond, &edma_common.irq.handle);

	dmamux_set_source(SAI1_RX_DMA_CHANNEL, SAI1_RX_DMA_REQUEST);
	dmamux_channel_enable(SAI1_RX_DMA_CHANNEL);
	edma_channel_enable(SAI1_RX_DMA_CHANNEL);

	edma_common.buffer.size = size;
	edma_common.buffer.count = count;
	edma_common.buffer.ptr = buf;

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

	if (data != NULL && len != sizeof(uint32_t))
		return -EIO;

	mutexLock(edma_common.irq.lock);
	if ((res = condWait(edma_common.irq.cond, edma_common.irq.lock, 1000000)) == 0)
		*(uint32_t **)data = (uint32_t *)&edma_common.irq.cnt;
	mutexUnlock(edma_common.irq.lock);

	return res;
}


void dma_free(void)
{
	free_uncached(edma_common.buffer.ptr, edma_common.buffer.size * edma_common.buffer.count);
	edma_common.buffer.size = 0;
	edma_common.buffer.count = 0;
	edma_common.buffer.ptr = NULL;

	resourceDestroy(edma_common.irq.lock);
	resourceDestroy(edma_common.irq.cond);
	edma_common.irq.cnt = 0;
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
