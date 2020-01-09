/*
 * Phoenix-RTOS
 *
 * USB Data
 *
 * Copyright 2018, 2019, 2020 Phoenix Systems
 * Author: Jan Sikorski, Kamil Amanowicz, Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _USB_DESCRIPTORS_H_
#define _USB_DESCRIPTORS_H_

#define REQUEST_DIR_HOST2DEV  (0 << 7)
#define REQUEST_DIR_DEV2HOST  (1 << 7)
#define REQUEST_DIR_MASK      (1 << 7)

#define REQUEST_TYPE_STANDARD  (0 << 5)
#define REQUEST_TYPE_CLASS     (1 << 5)
#define REQUEST_TYPE_VENDOR    (2 << 5)

#define REQUEST_RECIPIENT_DEVICE    0
#define REQUEST_RECIPIENT_INTERFACE 1
#define REQUEST_RECIPIENT_ENDPOINT  2
#define REQUEST_RECIPIENT_OTHER     3

#define EXTRACT_REQ_TYPE(req_type) ((0x3 << 5) & req_type)

/* request types */
#define REQ_GET_STATUS         0
#define REQ_CLEAR_FEATURE      1
#define REQ_SET_FEATURE        3
#define REQ_SET_ADDRESS        5
#define REQ_GET_DESCRIPTOR     6
#define REQ_SET_DESCRIPTOR     7
#define REQ_GET_CONFIGURATION  8
#define REQ_SET_CONFIGURATION  9
#define REQ_GET_INTERFACE      10
#define REQ_SET_INTERFACE      11
#define REQ_SYNCH_FRAME        12


/* class request codes */
#define CLASS_REQ_GET_REPORT   1
#define CLASS_REQ_GET_IDLE     2
#define CLASS_REQ_GET_PROTOCOL 3
#define CLASS_REQ_SET_REPORT   9
#define CLASS_REQ_SET_IDLE     10
#define CLASS_REQ_SET_PROTOCOL 11
#define CLASS_REQ_SET_LINE_CODING 0x20
#define CLASS_REQ_SET_CONTROL_LINE_STATE 0x22


/* descriptor types */
#define USB_DESC_DEVICE 1
#define USB_DESC_CONFIG 2
#define USB_DESC_STRING 3
#define USB_DESC_INTERFACE 4
#define USB_DESC_ENDPOINT 5
#define USB_DESC_TYPE_DEV_QUAL 6
#define USB_DESC_TYPE_OTH_SPD_CFG 7
#define USB_DESC_TYPE_INTF_PWR 8
#define USB_DESC_INTERFACE_ASSOCIATION 11
#define USB_DESC_TYPE_HID  0x21
#define USB_DESC_TYPE_HID_REPORT 0x22
#define USB_DESC_TYPE_CDC_CS_INTERFACE 0x24


/* endpoint types */
#define USB_ENDPT_TYPE_CONTROL 0
#define USB_ENDPT_TYPE_ISO     1
#define USB_ENDPT_TYPE_BULK    2
#define USB_ENDPT_TYPE_INTR    3


/* endpoint direction */
#define USB_ENDPT_DIR_OUT 0
#define USB_ENDPT_DIR_IN  1


/* endpoint feature */
#define USB_ENDPOINT_HALT 0

/* high speed device can have 15 IN and 15 OUT endpoints
 * in addition to control endpoint (mandatory, both in and out) */
#define USB_MAX_ENDPOINTS 31

/* class specific desctriptors */
#define USB_DESC_CS_INTERFACE 0x24
#define USB_DESC_CS_ENDPOINT 0x25


#define USB_TIMEOUT 5000000


enum { pid_out = 0xe1, pid_in = 0x69, pid_setup = 0x2d };

enum { out_token = 0, in_token, setup_token };


typedef struct usb_setup_packet {
	uint8_t  bmRequestType;
	uint8_t  bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} __attribute__((packed)) usb_setup_packet_t;


struct usb_desc_header {
	uint8_t bLength;                /* size of descriptor */
	uint8_t bDescriptorType;        /* descriptor type */
};


typedef struct usb_device_desc {
	uint8_t  bLength;               /* size of descriptor */
	uint8_t  bDescriptorType;       /* descriptor type */
	uint16_t bcdUSB;                /* usb specification in BCD */
	uint8_t  bDeviceClass;          /* device class code (USB-IF)*/
	uint8_t  bDeviceSubClass;       /* device subclass code (USB-IF)*/
	uint8_t  bDeviceProtocol;       /* protocol code  (USB-IF)*/
	uint8_t  bMaxPacketSize0;       /* max packet size for endpoint0 */
	uint16_t idVendor;              /* vendor id (USB-IF) */
	uint16_t idProduct;             /* product id */
	uint16_t bcdDevice;             /* device release number in BCD */
	uint8_t  iManufacturer;         /* manufacturer string index */
	uint8_t  iProduct;              /* product string index */
	uint8_t  iSerialNumber;         /* serial number string index */
	uint8_t  bNumConfigurations;    /* number of possible configurations */
} __attribute__((packed)) usb_device_desc_t;


typedef struct usb_configuration_desc {
	uint8_t  bLength;               /* size of descriptor */
	uint8_t  bDescriptorType;       /* descriptor type */
	uint16_t wTotalLength;          /* total bytes returned for this configuration */
	uint8_t  bNumInterfaces;        /* number of interfaces supported */
	uint8_t  bConfigurationValue;   /* value to use for SET_CONFIGURATION request */
	uint8_t  iConfiguration;        /* configuration string index */
	uint8_t  bmAttributes;          /* attributes bitmap */
	uint8_t  bMaxPower;             /* maximum power consumption */
} __attribute__((packed)) usb_configuration_desc_t;


typedef struct usb_interface_desc {
	uint8_t bLength;                /* size of descriptor */
	uint8_t bDescriptorType;        /* descriptor type */
	uint8_t bInterfaceNumber;       /* number of this interface */
	uint8_t bAlternateSetting;      /* value for the alternate setting */
	uint8_t bNumEndpoints;          /* number of endpoints */
	uint8_t bInterfaceClass;        /* interface class code */
	uint8_t bInterfaceSubClass;     /* interface subclass code */
	uint8_t bInterfaceProtocol;     /* interface protocol code */
	uint8_t iInterface;             /* interface string index */
} __attribute__((packed)) usb_interface_desc_t;


typedef struct usb_interface_association_desc {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bFirstInterface;
	uint8_t bInterfaceCount;
	uint8_t bFunctionClass;
	uint8_t bFunctionSubClass;
	uint8_t bFunctionProtocol;
	uint8_t iFunction;
} __attribute__((packed)) usb_interface_association_desc_t;


typedef struct usb_string_desc {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t wData[256];
} __attribute__((packed)) usb_string_desc_t;


typedef struct usb_endpoint_desc {
	uint8_t  bLength;               /* size of descriptor */
	uint8_t  bDescriptorType;       /* descriptor type */
	uint8_t  bEndpointAddress;      /* endpoint address */
	uint8_t  bmAttributes;          /* attributes bitmap */
	uint16_t wMaxPacketSize;        /* maximum packet size */
	uint8_t  bInterval;             /* polling interval for data transfers */
} __attribute__((packed)) usb_endpoint_desc_t;


/* generic descriptor
 * used when there is no defined descriptor (e.g. HID descriptor or Report descriptor) */
typedef struct usb_functional_desc {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
} __attribute__((packed)) usb_functional_desc_t;


#endif
