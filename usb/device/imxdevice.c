/*
 * Phoenix-RTOS
 *
 * usbclient - usb device controller driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Kamil Amanowicz, Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "imxdevice.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/msg.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/threads.h>
#include <sys/mman.h>

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

/* Physical addresses for USB controller */
addr_t pdev;
addr_t pconf;

addr_t pstr_0;
addr_t pstr_man;
addr_t pstr_prod;

addr_t phid_reports;

addr_t pIN;
addr_t pOUT;
u8 *IN;
u8 *OUT;

usb_common_data_t *usb_data;

usb_dc_t *dc;


void init_desc(usbclient_conf_t *conf, usb_common_data_t *usb_data_in, usb_dc_t *dc_in)
{
    usb_data = usb_data_in;
    dc = dc_in;

    usbclient_desc_dev_t *dev;
    usbclient_desc_conf_t *cfg;
    usbclient_desc_intf_t *intf;
    usbclient_desc_gen_t *hid;
    usbclient_desc_ep_t *endpt;

    usbclient_desc_str_zr_t *str_0;
    usbclient_desc_gen_t *str_man;
    usbclient_desc_gen_t *str_prod;

    usbclient_desc_gen_t *hid_reports;

    memset(usb_data->local_conf, 0, 0x1000);

    /* Virtual addresses offsets */
    dev = usb_data->local_conf;
    cfg = (usbclient_desc_conf_t*)(dev + 1);
    intf = (usbclient_desc_intf_t*)(cfg + 1);
    hid = (usbclient_desc_gen_t*)(intf + 1);
    endpt = (usbclient_desc_ep_t*)(((uint8_t*)hid) + 9);
    str_0 = (usbclient_desc_str_zr_t*)(endpt + 1);
    str_man = (usbclient_desc_gen_t*)(str_0 + 1);
    str_prod = (usbclient_desc_gen_t*)(((uint8_t*)str_man) + 56);
    hid_reports = (usbclient_desc_gen_t*)(((uint8_t*)str_prod) + 28);

    /* Physical addresses offsets */
#ifdef TARGET_IMX6ULL
    pdev = (((u32)va2pa(dev)) & ~0xfff) + ((u32)dev & 0xfff);
    pconf = (((u32)va2pa(cfg)) & ~0xfff) + ((u32)cfg & 0xfff);

    pstr_0 = (((u32)va2pa(str_0)) & ~0xfff) + ((u32)str_0 & 0xfff);
    pstr_man = (((u32)va2pa(str_man)) & ~0xfff) + ((u32)str_man & 0xfff);
    pstr_prod = (((u32)va2pa(str_prod)) & ~0xfff) + ((u32)str_prod & 0xfff);
    phid_reports = (((u32)va2pa(hid_reports)) & ~0xfff) + ((u32)hid_reports & 0xfff);

    /* Endpoints */
    IN = usb_data->local_conf + 0x500;
    OUT = usb_data->local_conf + 0x700;

    pIN = ((va2pa((void *)IN)) & ~0xfff) + ((u32)IN & 0xfff);
    pOUT = ((va2pa((void *)OUT)) & ~0xfff) + ((u32)OUT & 0xfff);
#endif


    /* i.MX RT works without MMU */
#ifdef TARGET_IMXRT
    pdev =  (u32) dev;
    pconf =  (u32) cfg;

    pstr_0 =  (u32) str_0;
    pstr_man =  (u32) str_man;
    pstr_prod =  (u32) str_prod;
    phid_reports =  (u32) hid_reports;

    /* Endpoints */
    IN = usb_data->local_conf + 0x500;
    OUT = usb_data->local_conf + 0x700;

    pIN =  (u32) IN;
    pOUT =  (u32) OUT;
#endif

    uint32_t string_desc_count = 0;
    /* Extract mandatory descriptors to mapped memory */
    usbclient_desc_list_t* it = conf->descriptors_head;
    for (; it != NULL; it = it->next) {
        switch(it->descriptors->desc_type) {
            case USBCLIENT_DESC_TYPE_DEV:
                memcpy(dev, &it->descriptors[0], sizeof(usbclient_desc_dev_t));
                break;
            case USBCLIENT_DESC_TYPE_CFG:
                memcpy(cfg, &it->descriptors[0], sizeof(usbclient_desc_conf_t));
                break;
            case USBCLIENT_DESC_TYPE_INTF:
                memcpy(intf, &it->descriptors[0], sizeof(usbclient_desc_intf_t));
                break;
            case USBCLIENT_DESC_TYPE_ENDPT:
                memcpy(endpt, &it->descriptors[0], sizeof(usbclient_desc_ep_t));
                /* Initialize endpoint */

                /* For now hardcode only one endpoint */
                usb_data->in_endpt.rx_caps.mult = 0;
                usb_data->in_endpt.rx_caps.zlt = 1;
                usb_data->in_endpt.rx_caps.max_pkt_len = endpt->max_pkt_sz;
                usb_data->in_endpt.rx_caps.ios = 0;

                usb_data->in_endpt.rx_ctrl.type = (endpt->attr_bmp & 0x03);
                usb_data->in_endpt.rx_ctrl.data_toggle = 1;
                usb_data->in_endpt.rx_ctrl.data_inhibit = 0;
                usb_data->in_endpt.rx_ctrl.stall = 0;

                usb_data->in_endpt.tx_caps.mult = 0;
                usb_data->in_endpt.tx_caps.zlt = 1;
                usb_data->in_endpt.tx_caps.max_pkt_len = endpt->max_pkt_sz;
                usb_data->in_endpt.tx_caps.ios = 0;

                usb_data->in_endpt.tx_ctrl.type = (endpt->attr_bmp & 0x03);
                usb_data->in_endpt.tx_ctrl.data_toggle = 1;
                usb_data->in_endpt.tx_ctrl.data_inhibit = 0;
                usb_data->in_endpt.tx_ctrl.stall = 0;
                break;
            case USBCLIENT_DESC_TYPE_HID:
                memcpy(hid, &it->descriptors[0], 9);
                break;
            case USBCLIENT_DESC_TYPE_HID_REPORT:
                /* Copy only data section, because HID report descriptor is sent raw */
                memcpy(hid_reports, &it->descriptors[0].data, it->descriptors[0].len - 2);
                break;
            case USBCLIENT_DESC_TYPE_STR:
                if (string_desc_count == 0) {
                    memcpy(str_0, &it->descriptors[0], sizeof(usbclient_desc_str_zr_t));
                } else if (string_desc_count == 1) {
                    memcpy(str_man, &it->descriptors[0], it->descriptors[0].len);
                } else if (string_desc_count == 2) {
                    memcpy(str_prod, &it->descriptors[0], it->descriptors[0].len);
                }
                string_desc_count++;
                break;
            case USBCLIENT_DESC_TYPE_DEV_QUAL:
            case USBCLIENT_DESC_TYPE_OTH_SPD_CFG:
            case USBCLIENT_DESC_TYPE_INTF_PWR:
            default:
                /* Not implemented yet */
                break;
        }
    }
}


static dtd_t *dtd_get(int endpt, int dir)
{
    int qh = endpt * 2 + dir;
    u32 base_addr;
    dtd_t *ret;

    base_addr = ((u32)dc->endptqh[qh].head & ~((dc->endptqh[qh].size * sizeof(dtd_t)) - 1));

    ret = dc->endptqh[qh].tail++;
    dc->endptqh[qh].tail = (dtd_t *)(base_addr | ((u32)dc->endptqh[qh].tail & (((dc->endptqh[qh].size) * sizeof(dtd_t)) - 1)));

    return ret;
}


static int dtd_build(dtd_t *dtd, u32 paddr, u32 size)
{
    if (size > 0x1000)
        return -EINVAL;

    dtd->dtd_next = 1;
    if (size)
        dtd->dtd_token = size << 16;
    else
        dtd->dtd_token = 0;

    dtd->dtd_token |= 1 << 7;
    dtd->buff_ptr[0] = paddr;

    return EOK;
}


static int dtd_exec(int endpt, u32 paddr, u32 sz, int dir)
{
    int shift;
    u32 offs;
    dtd_t *dtd;
    int qh = (endpt << 1) + dir;

    dtd = dtd_get(endpt, dir);

    dtd_build(dtd, paddr, sz);

    shift = endpt + ((qh & 1) ? 16 : 0);
    offs = (u32)dtd & (((dc->endptqh[qh].size) * sizeof(dtd_t)) - 1);

    dc->endptqh[qh].dtd_next = (dc->endptqh[qh].base + offs) & ~1;
    dc->endptqh[qh].dtd_token &= ~(1 << 6);
    dc->endptqh[qh].dtd_token &= ~(1 << 7);

    /* prime the endpoint and wait for it to prime */
    while ((*(dc->base + endptprime) & (1 << shift)));
    *(dc->base + endptprime) |= 1 << shift;
    while (!(*(dc->base + endptprime) & (1 << shift)) && (*(dc->base + endptstat) & (1 << shift)));

    while (!(*(dc->base + endptcomplete) & (1 << shift)));
    *(dc->base + endptcomplete) |= 1 << shift;

    while (*(dc->base + usbsts) & 1);
    *(dc->base + usbsts) |= 1;
    dc->endptqh[qh].head += 1;

    return EOK;
}


static int dc_setup(setup_packet_t *setup)
{
    if (EXTRACT_REQ_TYPE(setup->req_type) != REQ_TYPE_STANDARD) {
        return EOK;
    }

    int res = EOK;
    u32 fsz;

    switch (setup->req_code) {
        case REQ_SET_ADDR:
            if (setup->val) {
                dc->status = DC_ADDRESS;
                dc->dev_addr = setup->val << 25;
                dc->dev_addr |= 1 << 24;
                *(dc->base + deviceaddr) = dc->dev_addr;
                dc->op = DC_OP_INIT;
                dtd_exec(0, pIN, 0, USBCLIENT_ENDPT_DIR_IN);
            } else if (dc->status != DC_CONFIGURED)
                dc->status = DC_DEFAULT;
            break;

        case REQ_SET_CONFIG:
            if (dc->status == DC_ADDRESS) {
                dc->status = DC_CONFIGURED;
                dtd_exec(0, pIN, 0, USBCLIENT_ENDPT_DIR_IN);
            }
            break;

        case REQ_GET_DESC:
            if (setup->val >> 8 == USBCLIENT_DESC_TYPE_DEV)
                dtd_exec(0, pdev, sizeof(usbclient_desc_dev_t), USBCLIENT_ENDPT_DIR_IN);
            else if (setup->val >> 8 == USBCLIENT_DESC_TYPE_CFG)
                dtd_exec(0, pconf, setup->len, USBCLIENT_ENDPT_DIR_IN);
            else if (setup->val >> 8 == USBCLIENT_DESC_TYPE_STR) {

                if ((setup->val & 0xff) == 0) {
                    dtd_exec(0, pstr_0, MIN(sizeof(usbclient_desc_str_zr_t), setup->len), USBCLIENT_ENDPT_DIR_IN);
                } else if ((setup->val & 0xff) == 1) {
                    dtd_exec(0, pstr_man, MIN(56, setup->len), USBCLIENT_ENDPT_DIR_IN);
                } else if ((setup->val & 0xff) == 2) {
                    dtd_exec(0, pstr_prod, MIN(28, setup->len), USBCLIENT_ENDPT_DIR_IN);
                }
            } else if (setup->val >> 8 == USBCLIENT_DESC_TYPE_HID_REPORT) {
                dtd_exec(0, phid_reports, 76, USBCLIENT_ENDPT_DIR_IN);
            }
            dtd_exec(0, pOUT, 0x40, USBCLIENT_ENDPT_DIR_OUT);
            break;

        case REQ_CLR_FEAT:
        case REQ_GET_STS:
        case REQ_GET_INTF:
        case REQ_SET_INTF:
        case REQ_SET_FEAT:
        case REQ_SET_DESC:
        case REQ_SYNCH_FRAME:
            break;

        case REQ_GET_CONFIG:
            if (setup->val != 0 || setup->idx != 0 || setup->len != 1)
                return res;
            if (dc->status != DC_CONFIGURED)
                OUT[0] = 0;
            else
                OUT[1] = 1;

            dtd_exec(0, pOUT, setup->len, USBCLIENT_ENDPT_DIR_OUT);
            break;

        default:
            if (*(u32 *)setup == 0xdeadc0de)
                dc->op = DC_OP_EXIT;
            else {
                fsz = setup->val << 16;
                fsz |= setup->idx;
                dtd_exec(0, pOUT, setup->len, USBCLIENT_ENDPT_DIR_OUT);
                dtd_exec(0, pIN, 0, USBCLIENT_ENDPT_DIR_IN);
                //strcpy(dc->mods[dc->mods_cnt].name, (const char *)OUT);
                //dc->mods[dc->mods_cnt].size = fsz;
                OUT[0] = 0;
                dtd_exec(1, pOUT, 0x80, USBCLIENT_ENDPT_DIR_OUT);
                //strcpy(dc->mods[dc->mods_cnt].args, (const char *)OUT);
                dc->op = DC_OP_RECEIVE;
                //dc->mods_cnt++;
            }
            break;
    }

    return res;
}


static int dc_class_setup(setup_packet_t *setup)
{
    if (EXTRACT_REQ_TYPE(setup->req_type) != REQ_TYPE_CLASS) {
        return EOK;
    }

    int res = EOK;

    switch (setup->req_code) {
        case CLASS_REQ_SET_IDLE:
            dtd_exec(0, pIN, 0, USBCLIENT_ENDPT_DIR_IN);
            break;
        case CLASS_REQ_SET_REPORT:
            dtd_exec(0, pOUT, 64 + setup->len, USBCLIENT_ENDPT_DIR_OUT); /* read data to buffer with URB struct*/
            usb_data->read_buffer.length = setup->len;
            dc->op = DC_OP_RECEIVE; /* mark that data is ready */
            break;
        case CLASS_REQ_GET_IDLE:
        case CLASS_REQ_GET_PROTOCOL:
        case CLASS_REQ_GET_REPORT:
        case CLASS_REQ_SET_PROTOCOL:
        default:
            break;
    }

    return res;
}


/* high frequency interrupts */
int dc_hf_intr(void)
{
    setup_packet_t setup;
    u32 status;
    int endpt = 0;

    if ((status = *(dc->base + endptsetupstat)) & 0x1) {
        /* trip wire set */
        while (!((status >> endpt) & 1))
            endpt++;
        do {
            *(dc->base + usbcmd) |= 1 << 13;
            memcpy(&setup, dc->endptqh[endpt].setup_buff, sizeof(setup_packet_t));
        } while (!(*(dc->base + usbcmd) & 1 << 13));

        *(dc->base + endptsetupstat) |= 1 << endpt;
        *(dc->base + usbcmd) &= ~(1 << 13);
        *(dc->base + endptflush) |= 0xffffffff;
        *(dc->base + usbsts) |= 1;

        dc->endptqh[0].head = dc->endptqh[0].tail;
        dc->endptqh[1].head = dc->endptqh[1].tail;
        dc->endptqh[2].head = dc->endptqh[2].tail;

        while (*(dc->base + endptsetupstat) & 1);

        dc_setup(&setup);
        dc_class_setup(&setup);
    }

    return 1;
}


/* low frequency interrupts */
int dc_lf_intr(void)
{
    if ((*(dc->base + usbsts) & 1 << 6)) {

        *(dc->base + endptsetupstat) = *(dc->base + endptsetupstat);
        *(dc->base + endptcomplete) = *(dc->base + endptcomplete);

        while (*(dc->base + endptprime));

        *(dc->base + endptflush) = 0xffffffff;

        while(*(dc->base + portsc1) & 1 << 8);

        *(dc->base + usbsts) |= 1 << 6;
        dc->status = DC_DEFAULT;
    }

    return 1;
}


int usbclient_send(usbclient_ep_t *ep, const void *data, unsigned int len)
{
    if (len > PAGE_SIZE)
        return -1;

    if (ep->direction != USBCLIENT_ENDPT_DIR_IN)
        return -1;

    memcpy(IN, data, len);
    dtd_exec(ep->id, pIN, len, ep->direction);

    return len;
}


int usbclient_receive(usbclient_ep_t *ep, void *data, unsigned int len)
{
    int32_t result = -1;
    while (dc->op != DC_OP_EXIT) {
        mutexLock(dc->lock);
        while (dc->op == DC_OP_NONE)
            condWait(dc->cond, dc->lock, 0);
        mutexUnlock(dc->lock);

        if (dc->op == DC_OP_RECEIVE) {
            /* Copy data to buffer */
            /* TODO: take data len into account when copying */
            memcpy(data, (const char *)OUT, usb_data->read_buffer.length); /* copy data to buffer */
            result = usb_data->read_buffer.length;
            usb_data->read_buffer.length = 0;
            dc->op = DC_OP_NONE;
            dtd_exec(0, pIN, 0, USBCLIENT_ENDPT_DIR_IN); /* ACK */
            break;
        }
    }
    return result;
}
