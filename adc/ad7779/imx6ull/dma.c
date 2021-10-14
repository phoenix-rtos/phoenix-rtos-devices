#include <errno.h>
#include <string.h>
#include <unistd.h>

#include <sys/mman.h>
#include <sys/threads.h>
#include <sys/types.h>
#include <sys/interrupt.h>

#include <phoenix/arch/imx6ull.h>
#include <sdma.h>

#include "../ad7779.h"

#define SDMA_DEVICE_FILE_NAME ("/dev/sdma/ch07")

struct {
	sdma_t sdma;
	struct {
		size_t size;
		size_t count;
		void *ptr;
	} buffer;
	sdma_buffer_desc_t *bd;
} sdma;

static int sdma_configure(size_t size, size_t count, addr_t *phys_addr)
{
	int res;

	uint8_t event_transfer = 39; /* SAI3 RX FIFO */
	uint8_t event_channel = event_transfer;

	unsigned tries = 25;
	while ((res = sdma_open(&sdma.sdma, SDMA_DEVICE_FILE_NAME)) < 0) {
		usleep(100 * 1000);
		if (--tries == 0) {
			log_error("failed to open SDMA device file (%s)", SDMA_DEVICE_FILE_NAME);
			return -1;
		}
	}

	sdma.buffer.ptr = sdma_alloc_uncached(&sdma.sdma, size, phys_addr, 1);
	if (sdma.buffer.ptr == NULL) {
		log_error("failed to allocate buffers");
		return -1;
	}

	memset(sdma.buffer.ptr, 0, size);

	addr_t bd_paddr;
	sdma.bd = sdma_alloc_uncached(&sdma.sdma, count * sizeof(sdma_buffer_desc_t), &bd_paddr, 1);
	if (sdma.bd == NULL) {
		log_error("failed to allocate memory for buffer descriptors");
		return -1;
	}

	for (int i = 0; i < count; i++) {
		sdma.bd[i].count = _PAGE_SIZE;
		sdma.bd[i].flags = SDMA_BD_DONE | SDMA_BD_INTR | SDMA_BD_CONT;
		sdma.bd[i].command = SDMA_CMD_MODE_32_BIT;
		sdma.bd[i].buffer_addr = *phys_addr + i * _PAGE_SIZE;

		/* Last buffer descriptor must wrap */
		if (i == count - 1)
			sdma.bd[i].flags |= SDMA_BD_WRAP;
	}

	/* SDMA context setup */
	sdma_context_t sdma_context;
	sdma_context_init(&sdma_context);
	sdma_context_set_pc(&sdma_context, sdma_script__shp_2_mcu);
	if (event_transfer < 32) {
		sdma_context.gr[1] = 1 << event_transfer;
	}
	else {
		sdma_context.gr[0] = 1 << (event_transfer - 32); /* Event2_mask */
	}
	sdma_context.gr[6] = sai_get_rx_fifo_ptr();                 /* RX FIFO address */
	sdma_context.gr[7] = SAI_FIFO_WATERMARK * sizeof(uint32_t); /* Watermark level */

	/* Load channel context */
	sdma_context_set(&sdma.sdma, &sdma_context);

	sdma_channel_config_t cfg;
	cfg.bd_paddr = bd_paddr;
	cfg.bd_cnt = count;
	cfg.trig = sdma_trig__event;
	cfg.event = event_channel;
	cfg.priority = SDMA_CHANNEL_PRIORITY_MIN + 1;
	sdma_channel_configure(&sdma.sdma, &cfg);

	sdma.buffer.size = size;
	sdma.buffer.count = count;

	return 0;
}

void dma_free(void)
{
	sdma_free_uncached(sdma.buffer.ptr, sdma.buffer.size);
	sdma_free_uncached(sdma.bd, sdma.buffer.count * sizeof(sdma_buffer_desc_t));
}

void dma_enable(void)
{
	sdma_enable(&sdma.sdma);
}

void dma_disable(void)
{
	sdma_disable(&sdma.sdma);
}

int dma_read(void *data, size_t len)
{
	if (data != NULL && len != sizeof(unsigned))
		return -EIO;

	if (sdma_wait_for_intr(&sdma.sdma, data) < 0)
		return -EIO;

	return EOK;
}

int dma_init(size_t size, size_t count, addr_t *phys_addr)
{
	return sdma_configure(size, count, phys_addr);
}
