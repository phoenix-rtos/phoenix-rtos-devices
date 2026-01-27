/*
 * Phoenix-RTOS
 * STM32N6 USB Descriptor Manager (DEBUG + NO VM_2_PHYM)
 */

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <phoenix/arch/armv8m/stm32/n6/stm32n6.h>

#include "client.h"


typedef struct {
	addr_t pmAddr;
	usb_string_desc_t *vmStruct;
} addr_string_desc_t;

static struct {
	uint8_t *dev;
	uint8_t *cfg;
	addr_string_desc_t str0;
	addr_string_desc_t strMan;
	addr_string_desc_t strProd;
	addr_t hidReports;
	usb_dc_t *dc;
	usb_common_data_t *data;
} __attribute__((aligned(4))) desc_common;


static void desc_endptInit(usb_endpoint_desc_t *endpt)
{
	// int endNb = endpt->bEndpointAddress & 0x7;
	// int dir = (endpt->bEndpointAddress & (1 << 7)) ? USB_ENDPT_DIR_IN : USB_ENDPT_DIR_OUT;
	// desc_common.data->endpts[endNb].max_pkt_len[dir] = endpt->wMaxPacketSize;
	// desc_common.data->endpts[endNb].type[dir] = endpt->bmAttributes & 0x03;
	// desc_common.data->endpts[endNb].data_toggle[dir] = 0;
}


static void desc_strInit(usb_desc_list_t *desList, int *localOffset, int strOrder)
{
	void *target = (void *)(desc_common.data->setupMem + *localOffset);
	addr_t physAddr = (addr_t)target;

	switch (strOrder) {
		case 0:
			desc_common.str0.vmStruct = (usb_string_desc_t *)target;
			desc_common.str0.pmAddr = physAddr;
			memcpy(desc_common.str0.vmStruct, desList->descriptor, sizeof(usb_string_desc_t));
			*localOffset += desList->descriptor->bFunctionLength;
			break;
		case 1:
			desc_common.strMan.vmStruct = (usb_string_desc_t *)target;
			desc_common.strMan.pmAddr = physAddr;
			memcpy(desc_common.strMan.vmStruct, desList->descriptor, desList->descriptor->bFunctionLength);
			*localOffset += desList->descriptor->bFunctionLength;
			break;
		case 2:
			desc_common.strProd.vmStruct = (usb_string_desc_t *)target;
			desc_common.strProd.pmAddr = physAddr;
			memcpy(desc_common.strProd.vmStruct, desList->descriptor, desList->descriptor->bFunctionLength);
			*localOffset += desList->descriptor->bFunctionLength;
			break;
	}
}


int desc_init(usb_desc_list_t *desList, usb_common_data_t *usb_data_in, usb_dc_t *dc_in)
{
	int localOffset = 0;
	uint32_t string_desc_count = 0;
	char *vrtAddr;

	desc_common.dc = dc_in;
	desc_common.data = usb_data_in;

	memset(desc_common.data->setupMem, 0, USB_BUFFER_SIZE);

	/* Extract mandatory descriptors to mapped memory */
	for (; desList != NULL; desList = desList->next) {

		if (localOffset > USB_BUFFER_SIZE) {
			return -ENOMEM;
		}


		switch (desList->descriptor->bDescriptorType) {
			case USB_DESC_DEVICE:
				vrtAddr = desc_common.data->setupMem + localOffset;
				desc_common.dev = vrtAddr;
				memcpy(vrtAddr, desList->descriptor, sizeof(usb_device_desc_t));
				localOffset += desList->descriptor->bFunctionLength;

				break;

			case USB_DESC_CONFIG:
				vrtAddr = desc_common.data->setupMem + localOffset;
				desc_common.cfg = vrtAddr;
				memcpy(vrtAddr, desList->descriptor, sizeof(usb_configuration_desc_t));
				localOffset += desList->descriptor->bFunctionLength;
				break;

			case USB_DESC_INTERFACE:
				memcpy(desc_common.data->setupMem + localOffset, desList->descriptor, desList->descriptor->bFunctionLength);
				localOffset += desList->descriptor->bFunctionLength;
				break;

			case USB_DESC_ENDPOINT:
				memcpy(desc_common.data->setupMem + localOffset, desList->descriptor, desList->descriptor->bFunctionLength);
				localOffset += desList->descriptor->bFunctionLength;
				desc_endptInit((usb_endpoint_desc_t *)desList->descriptor);
				break;

			case USB_DESC_TYPE_HID:
				memcpy(desc_common.data->setupMem + localOffset, desList->descriptor, desList->descriptor->bFunctionLength);
				localOffset += desList->descriptor->bFunctionLength;
				break;

			case USB_DESC_TYPE_CDC_CS_INTERFACE:
				memcpy(desc_common.data->setupMem + localOffset, desList->descriptor, desList->descriptor->bFunctionLength);
				localOffset += desList->descriptor->bFunctionLength;
				break;

			case USB_DESC_TYPE_HID_REPORT:
				vrtAddr = desc_common.data->setupMem + localOffset;
				// desc_common.hidReports = VM_2_PHYM(vrtAddr);
				memcpy(vrtAddr, &desList->descriptor->bDescriptorSubtype, desList->descriptor->bFunctionLength - 2);
				localOffset += desList->descriptor->bFunctionLength - 2;
				break;

			case USB_DESC_STRING:
				desc_strInit(desList, &localOffset, string_desc_count++);
				break;

			case USB_DESC_TYPE_DEV_QUAL:
			case USB_DESC_TYPE_OTH_SPD_CFG:
			case USB_DESC_TYPE_INTF_PWR:
				/* Not implemented yet */
				break;

			default:
				break;
		}
	}

	return EOK;
}


static void desc_ReqSetAddress(const usb_setup_packet_t *setup)
{
	/*Endpoint initialization on SetAddress command
	This section describes what the application must do when it receives a SetAddress
	command in a SETUP packet.
	3898/4691
	1.Program the OTG_DCFG register with the device address received in the SetAddress
	command
	2.Program the core to send out a status IN packet*/
	desc_common.dc->dev_addr = setup->wValue;

	/* setting XFERSIZ */
	desc_common.dc->base[DIEPTSIZ0] &= ~(0x7F);
	desc_common.dc->base[DIEPTSIZ0] |= (0x7F & 0);
	/* setting PKTCNT */
	desc_common.dc->base[DIEPTSIZ0] &= ~(0x180000);
	desc_common.dc->base[DIEPTSIZ0] |= (1 << 19);

	/* clear NAK */
	desc_common.dc->base[DIEPCTL0] |= (1 << 26);
	/* set EPENA */
	desc_common.dc->base[DIEPCTL0] |= (1 << 31);
}


static void desc_ReqSetConfig(void)
{
	/*Endpoint initialization on SetConfiguration/SetInterface command
This section describes what the application must do when it receives a SetConfiguration or
SetInterface command in a SETUP packet.
1.When a SetConfiguration command is received, the application must program the
endpoint registers to configure them with the characteristics of the valid endpoints in
the new configuration.
2.When a SetInterface command is received, the application must program the endpoint
registers of the endpoints affected by this command.
3.Some endpoints that were active in the prior configuration or alternate setting are not
valid in the new configuration or alternate setting. These invalid endpoints must be
deactivated.
4.Unmask the interrupt for each active endpoint and mask the interrupts for all inactive
endpoints in the OTG_DAINTMSK register.
5.Set up the data FIFO RAM for each FIFO.
6.After all required endpoints are configured; the application must program the core to
send a status IN packet.*/
	/*At this point, the device core is configured to receive and transmit any type of data packet.*/
}


static int desc_ReqGetConfig(const usb_setup_packet_t *setup)
{
	// if (setup->wValue != 0 || setup->wIndex != 0 || setup->wLength != 1)
	// 	return EOK;
	// uint8_t *txBuf = (uint8_t *)desc_common.data->endpts[0].buf[USB_ENDPT_DIR_IN].vBuffer;
	// if (desc_common.dc->status != DC_CONFIGURED)
	// 	txBuf[0] = 0;
	// else
	// 	txBuf[0] = 1;
	// ctrl_execTransfer(0, (uint32_t)txBuf, 1, USB_ENDPT_DIR_IN);
	return EOK;
}


static void desc_ReqGetDescriptor(const usb_setup_packet_t *setup)
{
	// if (setup->wValue >> 8 == USB_DESC_DEVICE) {
	// 	ctrl_execTransfer(0, desc_common.dev, sizeof(usb_device_desc_t), USB_ENDPT_DIR_IN);
	// }
	// else if (setup->wValue >> 8 == USB_DESC_CONFIG) {
	// 	ctrl_execTransfer(0, desc_common.cfg, setup->wLength, USB_ENDPT_DIR_IN);
	// }
	// else if (setup->wValue >> 8 == USB_DESC_STRING) {
	// 	if ((setup->wValue & 0xff) == 0) {
	// 		ctrl_execTransfer(0, desc_common.str0.pmAddr, MIN(desc_common.str0.vmStruct->bLength, setup->wLength), USB_ENDPT_DIR_IN);
	// 	}
	// 	else if ((setup->wValue & 0xff) == 1) {
	// 		ctrl_execTransfer(0, desc_common.strMan.pmAddr, MIN(desc_common.strMan.vmStruct->bLength, setup->wLength), USB_ENDPT_DIR_IN);
	// 	}
	// 	else if ((setup->wValue & 0xff) == 2) {
	// 		ctrl_execTransfer(0, desc_common.strProd.pmAddr, MIN(desc_common.strProd.vmStruct->bLength, setup->wLength), USB_ENDPT_DIR_IN);
	// 	}
	// 	else if ((setup->wValue & 0xff) == 4) {
	// 		ctrl_execTransfer(0, desc_common.data->endpts[0].buf[USB_ENDPT_DIR_IN].pBuffer, 0, USB_ENDPT_DIR_IN);
	// 		ctrl_execTransfer(0, desc_common.data->endpts[0].buf[USB_ENDPT_DIR_OUT].pBuffer, 71, USB_ENDPT_DIR_OUT);
	// 		return;
	// 	}
	// }
	// else if (setup->wValue >> 8 == USB_DESC_TYPE_HID_REPORT) {
	// 	ctrl_execTransfer(0, desc_common.hidReports, 76, USB_ENDPT_DIR_IN);
	// }

	uint32_t len = 0;
	uint8_t *pbuf = NULL;

	// 1. Wybierz odpowiedni bufor w zależności od zapytania
	if (setup->wValue >> 8 == USB_DESC_DEVICE) {
		pbuf = (uint8_t *)desc_common.dev;
		len = sizeof(usb_device_desc_t);
	}

	// 2. Jeśli znaleziono deskryptor
	if (pbuf != NULL && len != 0) {
		// 3. PRZYTNIJ długość do żądania Hosta (setup->wLength)
		// To chroni przed "Babble Condition" i Broken Pipe
		len = MIN(len, setup->wLength);
		// __asm__ volatile ("1: b 1b");

		// 4. Wywołaj właściwą funkcję
		clbc_epTransmit(0, pbuf, len);
	}
}


int desc_setup(const usb_setup_packet_t *setup)
{
	int res = EOK;

	if (EXTRACT_REQ_TYPE(setup->bmRequestType) != REQUEST_TYPE_STANDARD)
		return EOK;

	switch (setup->bRequest) {
		case REQ_SET_ADDRESS:
			desc_ReqSetAddress(setup);
			break;

		case REQ_SET_CONFIGURATION:
			desc_ReqSetConfig();
			break;

		case REQ_GET_DESCRIPTOR:
			desc_ReqGetDescriptor(setup);
			break;

		case REQ_CLEAR_FEATURE:
		case REQ_GET_STATUS:
		case REQ_GET_INTERFACE:
		case REQ_SET_INTERFACE:
		case REQ_SET_FEATURE:
		case REQ_SET_DESCRIPTOR:
		case REQ_SYNCH_FRAME:
			break;

		case REQ_GET_CONFIGURATION:
			desc_ReqGetConfig(setup);
			break;

		default:
			// desc_defaultSetup(setup);
			break;
	}

	return res;
}


int desc_classSetup(const usb_setup_packet_t *setup)
{

	return EOK;
}

int desc_epActivation(void)
{
	/*Endpoint activation
This section describes the steps required to activate a device endpoint or to configure an
existing device endpoint to a new type.
1.
2.
Program the characteristics of the required endpoint into the following fields of the
OTG_DIEPCTLx register (for IN or bidirectional endpoints) or the OTG_DOEPCTLx
register (for OUT or bidirectional endpoints).
–Maximum packet size
–USB active endpoint = 1
–Endpoint start data toggle (for interrupt and bulk endpoints)
–Endpoint type
–Tx FIFO number
Once the endpoint is activated, the core starts decoding the tokens addressed to that
endpoint and sends out a valid handshake for each valid token received for the
endpoint.*/
}

int desc_epDeactivation(void)
{
	/*Endpoint deactivation
This section describes the steps required to deactivate an existing endpoint.
Note:
1.In the endpoint to be deactivated, clear the USB active endpoint bit in the
OTG_DIEPCTLx register (for IN or bidirectional endpoints) or the OTG_DOEPCTLx
register (for OUT or bidirectional endpoints).
2.Once the endpoint is deactivated, the core ignores tokens addressed to that endpoint,
which results in a timeout on the USB.
The application must meet the following conditions to set up the device core to handle
traffic:
NPTXFEM and RXFLVLM in the OTG_GINTMSK register must be cleared.*/
}
