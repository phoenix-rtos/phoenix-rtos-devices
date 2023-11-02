/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL SDMA lib
 *
 * Copyright 2018 Phoenix Systems
 * Author: Krystian Wasik
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>

#include <sys/msg.h>
#include <sys/mman.h>

#include "sdma.h"

static int sdma_dev_ctl(sdma_t *s, sdma_dev_ctl_t *dev_ctl, void *data, size_t size)
{
	int res;
	msg_t msg;

	msg.type = mtDevCtl;

	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = data;
	msg.o.size = size;
	memcpy(msg.o.raw, dev_ctl, sizeof(sdma_dev_ctl_t));

	if ((res = msgSend(s->oid.port, &msg)) < 0) {
		fprintf(stderr, "msgSend failed (%d)\n\r", res);
		return -1;
	} else if (msg.o.io.err != EOK) {
		fprintf(stderr, "devctl failed (%d)\n\r", msg.o.io.err);
		return -2;
	}

	memcpy(dev_ctl, msg.o.raw, sizeof(sdma_dev_ctl_t));

	return 0;
}

int sdma_open(sdma_t *s, const char *dev_name)
{
	int fd, res;
	if (s == NULL)
		return -1;

	if ((fd = open(dev_name, O_RDWR)) < 0)
		return -2;

	if ((res = lookup(dev_name, NULL, &s->oid)) < 0)
		return -3;

	return 0;
}

int sdma_close(sdma_t *s)
{
	/* TODO: Implement sdma_channel_close */

	return 0;
}

int sdma_channel_configure(sdma_t *s, sdma_channel_config_t *cfg)
{
	sdma_dev_ctl_t dev_ctl;

	dev_ctl.oid = s->oid;
	dev_ctl.type = sdma_dev_ctl__channel_cfg;
	dev_ctl.cfg = *cfg;

	return sdma_dev_ctl(s, &dev_ctl, NULL, 0);
}

int sdma_data_mem_write(sdma_t *s, void *data, size_t size, addr_t addr)
{
	sdma_dev_ctl_t dev_ctl;

	dev_ctl.oid = s->oid;
	dev_ctl.type = sdma_dev_ctl__data_mem_write;
	dev_ctl.mem.addr = addr;
	dev_ctl.mem.len = size;

	return sdma_dev_ctl(s, &dev_ctl, data, size);
}

int sdma_data_mem_read(sdma_t *s, void *data, size_t size, addr_t addr)
{
	sdma_dev_ctl_t dev_ctl;

	dev_ctl.oid = s->oid;
	dev_ctl.type = sdma_dev_ctl__data_mem_read;
	dev_ctl.mem.addr = addr;
	dev_ctl.mem.len = size;

	return sdma_dev_ctl(s, &dev_ctl, data, size);
}

int sdma_context_dump(sdma_t *s, sdma_context_t *ctx)
{
	sdma_dev_ctl_t dev_ctl;

	dev_ctl.oid = s->oid;
	dev_ctl.type = sdma_dev_ctl__context_dump;

	return sdma_dev_ctl(s, &dev_ctl, (void*)ctx, sizeof(sdma_context_t));
}

int sdma_context_set(sdma_t *s, const sdma_context_t *ctx)
{
	sdma_dev_ctl_t dev_ctl;

	dev_ctl.oid = s->oid;
	dev_ctl.type = sdma_dev_ctl__context_set;

	return sdma_dev_ctl(s, &dev_ctl, (void*)ctx, sizeof(sdma_context_t));
}

int sdma_enable(sdma_t *s)
{
	sdma_dev_ctl_t dev_ctl;

	dev_ctl.oid = s->oid;
	dev_ctl.type = sdma_dev_ctl__enable;

	return sdma_dev_ctl(s, &dev_ctl, NULL, 0);
}

int sdma_disable(sdma_t *s)
{
	sdma_dev_ctl_t dev_ctl;

	dev_ctl.oid = s->oid;
	dev_ctl.type = sdma_dev_ctl__disable;

	return sdma_dev_ctl(s, &dev_ctl, NULL, 0);
}

int sdma_trigger(sdma_t *s)
{
	sdma_dev_ctl_t dev_ctl;

	dev_ctl.oid = s->oid;
	dev_ctl.type = sdma_dev_ctl__trigger;

	return sdma_dev_ctl(s, &dev_ctl, NULL, 0);
}

int sdma_wait_for_intr(sdma_t *s, uint32_t *cnt)
{
	int res;
	msg_t msg;

	msg.type = mtRead;
	msg.o.size = sizeof(uint32_t);
	msg.o.data = cnt;
	msg.i.size = 0;
	msg.i.data = NULL;
	msg.i.io.oid = s->oid;

	if ((res = msgSend(s->oid.port, &msg)) < 0) {
		fprintf(stderr, "msgSend failed (%d)\n\r", res);
		return -1;
	} else if (msg.o.io.err != EOK) {
		fprintf(stderr, "read failed (%d)\n\r", msg.o.io.err);
		return -2;
	}

	return 0;
}

addr_t sdma_ocram_alloc(sdma_t *s, size_t size)
{
	int res;
	sdma_dev_ctl_t dev_ctl;

	if (s == NULL)
		return 0;

	dev_ctl.oid = s->oid;
	dev_ctl.type = sdma_dev_ctl__ocram_alloc;
	dev_ctl.alloc.size = size;

	if ((res = sdma_dev_ctl(s, &dev_ctl, NULL, 0)) < 0)
		return 0;

	return dev_ctl.alloc.paddr;
}


void *sdma_alloc_uncached(sdma_t *s, size_t size, addr_t *paddr, int ocram)
{
	uint32_t n = (size + _PAGE_SIZE - 1)/_PAGE_SIZE;
	int flags = MAP_UNCACHED | MAP_ANONYMOUS;
	addr_t _paddr = 0;

	if (ocram) {
		flags |= MAP_PHYSMEM;
		_paddr = sdma_ocram_alloc(s, n*_PAGE_SIZE);
		if (!_paddr)
			return NULL;
	}

	void *vaddr = mmap(NULL, n * _PAGE_SIZE, PROT_READ | PROT_WRITE, flags, -1, _paddr);
	if (vaddr == MAP_FAILED)
		return NULL;

	if (!ocram)
		_paddr = va2pa(vaddr);

	if (paddr != NULL)
		*paddr = _paddr;

	return vaddr;
}


int sdma_free_uncached(void *vaddr, size_t size)
{
	uint32_t n = (size + _PAGE_SIZE - 1)/_PAGE_SIZE;

	return munmap(vaddr, n*_PAGE_SIZE);
}
