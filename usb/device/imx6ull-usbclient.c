/*
 * Phoenix-RTOS
 *
 * usbclient - usb device controller driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Kamil Amanowicz, Bartosz Ciesla, Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "imxdevice.h"

#include <sys/interrupt.h>
#include <sys/threads.h>
#include <sys/debug.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <errno.h>

#define USB_ADDR 0x02184000

static usb_common_data_t usb_data;
static usb_dc_t dc = { 0 };


static int ctrlqh_init(void)
{
    u32 qh_addr;

    /* map queue head list */
    dc.endptqh = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ, MAP_UNCACHED, OID_NULL, 0);

    if (dc.endptqh == MAP_FAILED)
        return -ENOMEM;

    memset((void *)dc.endptqh, 0, 0x1000);

    qh_addr = ((u32)va2pa((void *)dc.endptqh)) & ~0xfff;

    dc.endptqh[0].caps =  0x40 << 16; /* max 64 bytes */
    dc.endptqh[0].caps |= 0x1 << 29;
    dc.endptqh[0].caps |=  0x1 << 15; /* ios */
    dc.endptqh[0].dtd_next = 0x1; /* invalid */

    dc.endptqh[1].caps =  0x40 << 16;
    dc.endptqh[1].caps |= 0x1 << 29;
    dc.endptqh[1].caps |=  0x1 << 15;
    dc.endptqh[1].dtd_next = 1;

    dc.endptqh[0].base = (((u32)va2pa(dc.endptqh)) & ~0xfff) + (32 * sizeof(dqh_t));
    dc.endptqh[0].size = 0x10;
    dc.endptqh[0].head = (dtd_t *)(dc.endptqh + 32);
    dc.endptqh[0].tail = (dtd_t *)(dc.endptqh + 32);

    dc.endptqh[1].base = (((u32)va2pa(dc.endptqh)) & ~0xfff) + (48 * sizeof(dqh_t));
    dc.endptqh[1].size = 0x10;
    dc.endptqh[1].head = (dtd_t *)(dc.endptqh + 48);
    dc.endptqh[1].tail = (dtd_t *)(dc.endptqh + 48);

    *(dc.base + endpointlistaddr) = qh_addr;
    *(dc.base + endptprime) |= 1;
    *(dc.base + endptprime) |= 1 << 16;

    return EOK;
}


static int dtd_init(int endpt)
{
    dtd_t *buff;
    int qh = endpt * 2;

    if (!endpt)
        return -EINVAL;

    buff = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ, MAP_UNCACHED, OID_NULL, 0);

    if (buff == MAP_FAILED)
        return -ENOMEM;

    memset(buff, 0, 0x1000);

    dc.endptqh[qh].base = (((u32)va2pa(buff)) & ~0xfff);
    dc.endptqh[qh].size = 0x40;
    dc.endptqh[qh].head = buff;
    dc.endptqh[qh].tail = buff;

    dc.endptqh[++qh].base = (((u32)va2pa(buff)) & ~0xfff) + (64 * sizeof(dtd_t));
    dc.endptqh[qh].size = 0x40;
    dc.endptqh[qh].head = buff + 64;
    dc.endptqh[qh].tail = buff + 64;

    return EOK;
}


static int endpt_init(int endpt, endpt_init_t *endpt_init)
{
    u32 setup = 0;
    int res;
    int qh_rx = endpt * 2 + USBCLIENT_ENDPT_DIR_OUT;
    int qh_tx = endpt * 2 + USBCLIENT_ENDPT_DIR_IN;

    if (endpt == 0)
        return -EINVAL;

    if ((res = dtd_init(endpt)) != EOK)
        return res;

    dc.endptqh[qh_rx].caps =  endpt_init->rx_caps.max_pkt_len << 16;
    dc.endptqh[qh_rx].caps |= endpt_init->rx_caps.ios << 15;
    dc.endptqh[qh_rx].caps |= endpt_init->rx_caps.zlt << 29;
    dc.endptqh[qh_rx].caps |= endpt_init->rx_caps.mult << 30;
    dc.endptqh[qh_rx].dtd_next = 1;

    dc.endptqh[qh_tx].caps =  endpt_init->tx_caps.max_pkt_len << 16;
    dc.endptqh[qh_tx].caps |= endpt_init->tx_caps.ios << 15;
    dc.endptqh[qh_tx].caps |= endpt_init->tx_caps.zlt << 29;
    dc.endptqh[qh_tx].caps |= endpt_init->tx_caps.mult << 30;
    dc.endptqh[qh_tx].dtd_next = 1;

    setup |= endpt_init->rx_ctrl.type << 2;
    setup |= endpt_init->tx_ctrl.type << 18;
    setup |= endpt_init->rx_ctrl.data_toggle << 6;
    setup |= endpt_init->tx_ctrl.data_toggle << 22;

    *(dc.base + endptctrl0 + endpt) = setup;
    *(dc.base + endptctrl0 + endpt) |= 1 << 7;
    *(dc.base + endptctrl0 + endpt) |= 1 << 23;

    return EOK;
}


static int dc_intr(unsigned int intr, void *data)
{
    dc_hf_intr();
    dc_lf_intr();

    return 0;
}


int usbclient_init(usbclient_conf_t *conf)
{
    int res = 0;
    /* Buffers init */
    usb_data.read_buffer.data = mmap(NULL, BUFFER_SIZE, PROT_WRITE | PROT_READ, MAP_UNCACHED, OID_NULL, 0);
    usb_data.pread_buffer = (((u32)va2pa(usb_data.read_buffer.data)) & ~0xfff) + ((u32)usb_data.read_buffer.data & 0xfff);
    usb_data.write_buffer.data = mmap(NULL, BUFFER_SIZE, PROT_WRITE | PROT_READ, MAP_UNCACHED, OID_NULL, 0);
    usb_data.pwrite_buffer = (((u32)va2pa(usb_data.write_buffer.data)) & ~0xfff) + ((u32)usb_data.write_buffer.data & 0xfff);

    usb_data.local_conf = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ, MAP_UNCACHED, OID_NULL, 0);
    dc.base = mmap(NULL, USB_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE, OID_PHYSMEM, USB_ADDR);

    if (dc.base == MAP_FAILED)
        return -ENOMEM;

    init_desc(conf, &usb_data, &dc);

    dc.lock = 0;
    dc.cond = 0;
    dc.dev_addr = 0;

    if (mutexCreate(&dc.lock) != EOK)
        return 0;
    if (condCreate(&dc.cond) != EOK)
        return 0;

    interrupt(75, dc_intr, NULL, dc.cond, &dc.inth);

    *(dc.base + endptflush) = 0xffffffff;
    /* Run/Stop bit */
    *(dc.base + usbcmd) &= ~1;
    /* Controller resets its internal pipelines, timers etc. */
    *(dc.base + usbcmd) |= 1 << 1;
    dc.status = DC_POWERED;
    /* Run/Stop register is set to 0 when the reset process is complete. */
    while (*(dc.base + usbcmd) & (1 << 1));

    /* set usb mode to device */
    *(dc.base + usbmode) |= 2;
    /* trip wire mode (setup lockout mode disabled) */
    *(dc.base + usbmode) |= 1 << 3;
    /* map queue heads list and init control endpoint */
    if ((res = ctrlqh_init()) != EOK)
        return res;

    *(dc.base + usbintr) |= 0x57;

    dc.status = DC_ATTACHED;
    *(dc.base + usbcmd) |= 1;

    while (dc.op != DC_OP_EXIT) {
        if (dc.op == DC_OP_INIT) {
            res = endpt_init(1, &usb_data.in_endpt); /* hardcode endpoint initialization */
            return res;
        }
    }

    return EOK;
}


int usbclient_destroy(void)
{
    /* stopping device controller */
    *(dc.base + usbintr) = 0;
    *(dc.base + usbcmd) &= ~1;

    munmap(usb_data.local_conf, 0x1000);
    munmap((void *)dc.base, 0x1000);
    munmap((void *)((u32)dc.endptqh[2].head & ~0xfff), 0x1000);
    munmap((void *)dc.endptqh, 0x1000);

    return 0;
}
