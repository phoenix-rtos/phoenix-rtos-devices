#include <sys/mman.h>
#include <sys/threads.h>
#include <sys/interrupt.h>

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "ehci.h"
#include "phy.h"
#include "dma.h"

#define USBSTS_AS  (1 << 15)
#define USBSTS_PS  (1 << 14)
#define USBSTS_RCL (1 << 13)
#define USBSTS_IAA (1 << 5)
#define USBSTS_SEI (1 << 4)
#define USBSTS_FRI (1 << 3)
#define USBSTS_PCI (1 << 2)
#define USBSTS_UEI (1 << 1)
#define USBSTS_UI  (1 << 0)

#define USBCMD_ASE (1 << 5)
#define USBCMD_IAA (1 << 6)

#define PORTSC_WKOC (1 << 22)
#define PORTSC_WKDSCNNT (1 << 21)
#define PORTSC_WKCNT (1 << 20)

#define PORTSC_PWR (1 << 12)
#define PORTSC_RST (1 << 8)
#define PORTSC_ENA (1 << 2)
#define PORTSC_CSC (1 << 1)
#define PORTSC_CCS (1 << 0)

#define USB_OTG2_IRQ (32 + 42)
#define USB_ADDR 0x02184000

enum {
	/* identification regs */
	id = 0x0, hwgeneral, hwhost, hwdevice, hwtxbuf, hwrxbuf,

	/* operational regs */
	gptimer0ld	= 0x20, gptimer0ctrl, gptimer1ld, gptimer1ctrl, sbuscfg,

	/* capability regs */
	caplength = 0x40, hciversion = 0x40, hcsparams, hccparams,
	dciversion = 0x48, dccparams,

	/* operational regs cont. */
	usbcmd = 0x50, usbsts, usbintr, frindex,
	periodiclistbase = 0x55, deviceaddr = 0x55, asynclistaddr = 0x56,
	endpointlistaddr = 0x56, burstsize = 0x58, txfilltunning, endptnak = 0x5E,
	endptnaken, configflag, portsc1, otgsc = 0x69, usbmode, endptsetupstat,
	endptprime, endptflush, endptstat, endptcomplete, endptctrl0, endptctrl1,
	endptctrl2, endptctrl3, endptctrl4, endptctrl5, endptctrl6, endptctrl7,

	otg1_ctrl = 0x100, otg2_ctrl,
};


enum { usb_otg1_ctrl = 0x200, usb_otg2_ctrl, usb_otg1_phy_ctrl = usb_otg2_ctrl + 5, usb_otg2_phy_ctrl };


struct {
	volatile unsigned *base;
	volatile unsigned *usb2;
	link_pointer_t *periodic_list;
	struct qh *async_head;

	handle_t irq_cond, irq_handle, irq_lock;
	unsigned status, port_change;
} ehci_common;


static int ehci_irqHandler(unsigned int n, void *data)
{
	ehci_common.status = *(ehci_common.usb2 + usbsts); // & 0x1f;//(USBSTS_SEI | USBSTS_PCI | USBSTS_UEI | USBSTS_UI);
	*(ehci_common.usb2 + usbsts) = ehci_common.status & 0x1f;

	if (ehci_common.status & USBSTS_RCL) {
		/* Async schedule is empty */

	}

	ehci_common.port_change = 0;

	if (ehci_common.status & USBSTS_PCI) {
		if (*(ehci_common.usb2 + portsc1) & PORTSC_CSC) {
			ehci_common.port_change = 1;
			*(ehci_common.usb2 + portsc1) = PORTSC_CSC;
		}
	}

	return -!ehci_common.status;
}


int ehci_await(int timeout)
{
	int err;
	mutexLock(ehci_common.irq_lock);
	err = condWait(ehci_common.irq_cond, ehci_common.irq_lock, timeout);
	mutexUnlock(ehci_common.irq_lock);
	return err;
}


void ehci_irqThread(void *arg)
{
	for (;;) {
		mutexLock(ehci_common.irq_lock);
		condWait(ehci_common.irq_cond, ehci_common.irq_lock, 0);
		mutexUnlock(ehci_common.irq_lock);

		((void (*)(int))arg)(ehci_common.port_change);
	}
}


void ehci_consQtd(struct qtd *qtd, struct qh *qh)
{
	link_pointer_t head = qh->transfer_overlay.next;
	qh->transfer_overlay.next.pointer = va2pa(qtd) >> 5;
	qh->transfer_overlay.next.terminate = 0;
	qtd->next = head;
}


struct qtd *ehci_allocQtd(int token, char *buffer, size_t *size, int datax)
{
	struct qtd *result = NULL;
	size_t initial_size;
	int ix, offset;

	if ((result = dma_alloc64()) == NULL)
		return NULL;

	result->data_toggle = datax;
	result->pid_code = token;

	result->next.terminate = 1;
	result->alt_next.terminate = 1;
	result->active = 1;

	if (buffer != NULL) {
		initial_size = *size;
		offset = (uintptr_t)buffer & (SIZE_PAGE - 1);
		result->offset = offset;

		for (ix = 0; ix < 5 && *size; ++ix) {
			result->buffers[ix].page = va2pa(buffer) >> 12;
			buffer += SIZE_PAGE - offset;

			if (*size > SIZE_PAGE - offset)
				*size -= SIZE_PAGE - offset;
			else
				*size = 0;

			offset = 0;
		}

		result->bytes_to_transfer = initial_size - *size;
	}

	return result;
}


void ehci_freeQtd(struct qtd *qtd)
{
	dma_free64(qtd);
}


struct qh *ehci_allocQh(int address, int endpoint, int transfer, int speed, int max_packet_len)
{
	struct qh *result = dma_alloc64();

	if (result == NULL)
		return NULL;

	result->horizontal.terminate = 1;

	result->device_addr = address;
	result->endpoint_speed = speed;
	result->data_toggle = transfer == transfer_control ? 1 : 0;

	result->current_qtd.terminate = 1;
	result->transfer_overlay.next.terminate = 1;

	result->max_packet_len = max_packet_len;
	result->endpoint = endpoint;
	result->control_endpoint = transfer == transfer_control && speed != high_speed;

	return result;
}


void ehci_freeQh(struct qh *qh)
{
	mutexLock(ehci_common.irq_lock);
	if (ehci_common.async_head != NULL) {
		*(ehci_common.usb2 + usbcmd) |= USBCMD_IAA;

		while (!(ehci_common.status & USBSTS_IAA))
			condWait(ehci_common.irq_cond, ehci_common.irq_lock, 0);

		ehci_common.status &= ~USBSTS_IAA;
	}
	mutexUnlock(ehci_common.irq_lock);

	dma_free64(qh);
}


void ehci_linkQh(struct qh *prev, struct qh *next)
{
	prev->horizontal.pointer = va2pa(next) >> 5;
	prev->horizontal.type = framelist_qh;
	prev->horizontal.terminate = 0;

	if (prev == next) {
		ehci_common.async_head = prev;
		prev->head_of_reclamation = 1;
		*(ehci_common.usb2 + asynclistaddr) = va2pa(prev);
		*(ehci_common.usb2 + usbcmd) |= USBCMD_ASE;
	}
}


void ehci_unlinkQh(struct qh *prev, struct qh *unlink, struct qh *next)
{
	if (unlink->head_of_reclamation)
		next->head_of_reclamation = 1;

	if (prev == next) {
		ehci_common.async_head = NULL;
		*(ehci_common.usb2 + usbcmd) &= ~USBCMD_ASE;
	}
	else if (unlink == ehci_common.async_head) {
		ehci_common.async_head = next;
		*(ehci_common.usb2 + asynclistaddr) = va2pa(next);
	}

	prev->horizontal = unlink->horizontal;
}


int ehci_deviceAttached(void)
{
	return *(ehci_common.usb2 + portsc1) & PORTSC_CCS;
}


void ehci_resetPort(void)
{
	/* Reset port */
	*(ehci_common.usb2 + portsc1) |= PORTSC_RST;
	*(ehci_common.usb2 + portsc1) &= ~PORTSC_ENA;
	while (*(ehci_common.usb2 + portsc1) & PORTSC_RST) ;
	*(ehci_common.usb2 + portsc1) |= PORTSC_ENA;

	/* Turn port power on */
	*(ehci_common.usb2 + portsc1) |= PORTSC_PWR;
	usleep(20 * 1000);

	/* Configure port */
	//	*(ehci_common.usb2 + portsc1) |= PORTSC_WKOC | PORTSC_WKCNT | PORTSC_WKDSCNNT;
}


void ehci_init(void *event_callback)
{
	int i;

	phy_init();
	condCreate(&ehci_common.irq_cond);
	mutexCreate(&ehci_common.irq_lock);

	ehci_common.periodic_list = mmap(NULL, SIZE_PAGE, PROT_WRITE | PROT_READ, MAP_ANONYMOUS | MAP_UNCACHED, OID_NULL, 0);

	for (i = 0; i < 1024; ++i)
		ehci_common.periodic_list[i] = (link_pointer_t) { .type = 0, .zero = 0, .pointer = 0, .terminate = 1 };

	ehci_common.base = mmap(NULL, 4 * SIZE_PAGE, PROT_WRITE | PROT_READ, MAP_DEVICE, OID_PHYSMEM, USB_ADDR);

	/* Offset into USB2 */
	ehci_common.usb2 = ehci_common.base + 128;

	beginthread(ehci_irqThread, 4, malloc(4000), 4000, event_callback);
	interrupt(USB_OTG2_IRQ, ehci_irqHandler, NULL, ehci_common.irq_cond, &ehci_common.irq_handle);

	/* Reset controller */
	*(ehci_common.usb2 + usbcmd) |= 2;
	while (*(ehci_common.usb2 + usbcmd) & 2) ;

	/* Set host mode */
	*(ehci_common.usb2 + usbmode) |= 3;

	/* Enable interrupts */
	*(ehci_common.usb2 + usbintr) = 0x1f & ~(1 << 3);

	/* Set periodic frame list */
	*(ehci_common.usb2 + periodiclistbase) = va2pa(ehci_common.periodic_list);

	/* Set interrupts threshold, frame list size, turn controller on */
	*(ehci_common.usb2 + usbcmd) |= (1 << 4) | 1;

	/* Route all ports to this host controller */
	*(ehci_common.usb2 + configflag) = 1;
}


void ehci_dumpRegisters(FILE *stream)
{
	fprintf(stream, "%18s: %08x", "id", *(ehci_common.usb2 + id));
	fprintf(stream, "%18s: %08x\n", "hwgeneral", *(ehci_common.usb2 + hwgeneral));
	fprintf(stream, "%18s: %08x", "hwhost", *(ehci_common.usb2 + hwhost));
	fprintf(stream, "%18s: %08x\n", "hwdevice", *(ehci_common.usb2 + hwdevice));
	fprintf(stream, "%18s: %08x", "hwtxbuf", *(ehci_common.usb2 + hwtxbuf));
	fprintf(stream, "%18s: %08x\n", "hwrxbuf", *(ehci_common.usb2 + hwrxbuf));
	fprintf(stream, "%18s: %08x", "gptimer0ld", *(ehci_common.usb2 + gptimer0ld));
	fprintf(stream, "%18s: %08x\n", "gptimer0ctrl", *(ehci_common.usb2 + gptimer0ctrl));
	fprintf(stream, "%18s: %08x", "gptimer1ld", *(ehci_common.usb2 + gptimer1ld));
	fprintf(stream, "%18s: %08x\n", "gptimer1ctrl", *(ehci_common.usb2 + gptimer1ctrl));
	fprintf(stream, "%18s: %08x", "sbuscfg", *(ehci_common.usb2 + sbuscfg));
	fprintf(stream, "%18s: %08x\n", "caplength", *(ehci_common.usb2 + caplength));
	fprintf(stream, "%18s: %08x", "hciversion", *(ehci_common.usb2 + hciversion));
	fprintf(stream, "%18s: %08x\n", "hcsparams", *(ehci_common.usb2 + hcsparams));
	fprintf(stream, "%18s: %08x", "hccparams", *(ehci_common.usb2 + hccparams));
	fprintf(stream, "%18s: %08x\n", "dciversion", *(ehci_common.usb2 + dciversion));
	fprintf(stream, "%18s: %08x", "dccparams", *(ehci_common.usb2 + dccparams));
	fprintf(stream, "%18s: %08x\n", "usbcmd", *(ehci_common.usb2 + usbcmd));
	fprintf(stream, "%18s: %08x", "usbsts", *(ehci_common.usb2 + usbsts));
	fprintf(stream, "%18s: %08x\n", "usbintr", *(ehci_common.usb2 + usbintr));
	fprintf(stream, "%18s: %08x", "frindex", *(ehci_common.usb2 + frindex));
	fprintf(stream, "%18s: %08x\n", "periodiclistbase", *(ehci_common.usb2 + periodiclistbase));
	fprintf(stream, "%18s: %08x", "deviceaddr", *(ehci_common.usb2 + deviceaddr));
	fprintf(stream, "%18s: %08x\n", "asynclistaddr", *(ehci_common.usb2 + asynclistaddr));
	fprintf(stream, "%18s: %08x", "endpointlistaddr", *(ehci_common.usb2 + endpointlistaddr));
	fprintf(stream, "%18s: %08x\n", "burstsize", *(ehci_common.usb2 + burstsize));
	fprintf(stream, "%18s: %08x", "txfilltunning", *(ehci_common.usb2 + txfilltunning));
	fprintf(stream, "%18s: %08x\n", "endptnak", *(ehci_common.usb2 + endptnak));
	fprintf(stream, "%18s: %08x", "endptnaken", *(ehci_common.usb2 + endptnaken));
	fprintf(stream, "%18s: %08x\n", "configflag", *(ehci_common.usb2 + configflag));
	fprintf(stream, "%18s: %08x", "portsc1", *(ehci_common.usb2 + portsc1));
	fprintf(stream, "%18s: %08x\n", "otgsc", *(ehci_common.usb2 + otgsc));
	fprintf(stream, "%18s: %08x", "usbmode", *(ehci_common.usb2 + usbmode));
	fprintf(stream, "%18s: %08x\n", "endptsetupstat", *(ehci_common.usb2 + endptsetupstat));
	fprintf(stream, "%18s: %08x", "endptprime", *(ehci_common.usb2 + endptprime));
	fprintf(stream, "%18s: %08x\n", "endptflush", *(ehci_common.usb2 + endptflush));
	fprintf(stream, "%18s: %08x", "endptstat", *(ehci_common.usb2 + endptstat));
	fprintf(stream, "%18s: %08x\n", "endptcomplete", *(ehci_common.usb2 + endptcomplete));
	fprintf(stream, "%18s: %08x", "endptctrl0", *(ehci_common.usb2 + endptctrl0));
	fprintf(stream, "%18s: %08x\n", "endptctrl1", *(ehci_common.usb2 + endptctrl1));
	fprintf(stream, "%18s: %08x", "endptctrl2", *(ehci_common.usb2 + endptctrl2));
	fprintf(stream, "%18s: %08x\n", "endptctrl3", *(ehci_common.usb2 + endptctrl3));
	fprintf(stream, "%18s: %08x", "endptctrl4", *(ehci_common.usb2 + endptctrl4));
	fprintf(stream, "%18s: %08x\n", "endptctrl5", *(ehci_common.usb2 + endptctrl5));
	fprintf(stream, "%18s: %08x", "endptctrl6", *(ehci_common.usb2 + endptctrl6));
	fprintf(stream, "%18s: %08x\n", "endptctrl7", *(ehci_common.usb2 + endptctrl7));
	fprintf(stream, "%18s: %08x", "otg1_ctrl", *(ehci_common.usb2 + otg1_ctrl));
	fprintf(stream, "%18s: %08x\n\n", "otg2_ctrl", *(ehci_common.usb2 + otg2_ctrl));
}
