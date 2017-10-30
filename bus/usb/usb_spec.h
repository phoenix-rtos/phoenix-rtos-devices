#ifndef _USB_SPEC_H_
#define _USB_SPEC_H_

#define PACKED __attribute__ ((__packed__))

/**
 * Setup packet for control transfer
 */
typedef struct usb_ctrlreq_ {
    u8 bmRequestType;
    u8 bRequest;
    u16 wValue;
    u16 wIndex;
    u16 wLength;
} PACKED usb_ctrlreq_t;

/**
 * bmRequestType field contents macros
 */
#define USB_DIR_OUT 0 	 /**< from host to device */
#define USB_DIR_IN  0x80 /**< from device to host */

#define USB_REQ_TYPE_STANDARD 	0x00
#define USB_REQ_TYPE_CLASS 		0x20
#define USB_REQ_TYPE_VENDOR		0x40

#define USB_RECIP_DEV 	0
#define USB_RECIP_IF 	0x01
#define USB_RECIP_EP 	0x02
#define USB_RECIP_OTH 	0x03

/**
 * Convenient bmRequestType field definitions
 */
#define USB_READ_STD_DEV (USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_DEV)
#define USB_WRITE_STD_DEV (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_DEV)
#define USB_WRITE_STD_EP (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_EP)
#define USB_READ_CLASS_DEV (USB_DIR_IN|USB_RECIP_DEV|USB_REQ_TYPE_CLASS)
#define USB_WRITE_CLASS_DEV (USB_DIR_IN|USB_RECIP_DEV|USB_REQ_TYPE_CLASS)
#define USB_READ_CLASS_IF (USB_DIR_IN|USB_RECIP_IF|USB_REQ_TYPE_CLASS)
#define USB_WRITE_CLASS_IF (USB_DIR_OUT|USB_RECIP_IF|USB_REQ_TYPE_CLASS)
#define USB_READ_CLASS_OTH (USB_DIR_IN|USB_RECIP_OTH|USB_REQ_TYPE_CLASS)
#define USB_WRITE_CLASS_OTH (USB_DIR_OUT|USB_RECIP_OTH|USB_REQ_TYPE_CLASS)
#define USB_READ_VENDOR_DEV (USB_DIR_IN|USB_RECIP_DEV|USB_REQ_TYPE_VENDOR)
#define USB_WRITE_VENDOR_DEV (USB_DIR_OUT|USB_RECIP_DEV|USB_REQ_TYPE_VENDOR)

/**
 * bRequest field
 */
#define USB_GET_STAT	0x00
#define USB_CLEAR_FEAT 	0x01
#define USB_SET_FEAT 	0x03
#define USB_SET_ADDR 	0x05
#define USB_GET_DESC 	0x06
#define USB_SET_DESC 	0x07
#define USB_GET_CFG 	0x08
#define USB_SET_CFG 	0x09
#define USB_GET_IF 		0x0A
#define USB_SET_IF 		0x0B

/**
 * USB descriptor types
 */
#define USB_DESC_DEV 0x01
#define USB_DESC_CFG 0x02
#define USB_DESC_STR 0x03
#define USB_DESC_IF  0x04
#define USB_DESC_EP  0x05
#define USB_DESC_HUB 0x29

/* Feature selectors */
#define USB_FEAT_EP_HALT 0
#define USB_FEAT_WAKEUP 1
#define USB_FEAT_TEST 2
/**
 * USB descriptor definitions
 */

/**
 * Device descriptor
 */
typedef struct usb_devDesc_{
    u8 bLength;
    u8 bDescriptorType;
    u16 bcdUSB;
    u8 bDeviceClass;
    u8 bDeviceSubClass;
    u8 bDeviceProtocol;
    u8 bMaxPacketSize0;
    u16 idVendor;
    u16 idProduct;
    u16 bcdDevice;
    u8 iManufacturer;
    u8 iProduct;
    u8 iSerialNumber;
    u8 bNumConfigurations;
} PACKED usb_devDesc_t;

#define USB_CLASS_IN_INTERFACE  0
#define USB_CLASS_CDC			2
#define USB_CLASS_HID			3
#define USB_CLASS_MASS_STORAGE	8
#define USB_CLASS_HUB			9
#define USB_CLASS_VENDOR_SPEC   0xFF

#define USB_MIN_CTRL_PACKET_SIZE 8
#define USB_CHECK_DEV_TYPE(dev,class,subclass,protocol) ((dev)->bDeviceClass==(class) && (dev)->bDeviceSubClass==(subclass) && (dev)->bDeviceProtocol==(protocol))


typedef struct usb_cfgDesc_{
    u8 bLength;
    u8 bDescriptorType;
    u16 wTotalLength;
    u8 bNumInterfaces;
    u8 bConfigurationValue;
    u8 iConfiguration;
    u8 bmAttributes;
    u8 bMaxPower;
} PACKED usb_cfgDesc_t;


typedef struct usb_ifDesc_{
    u8 bLength;
    u8 bDescriptorType;
    u8 bInterfaceNumber;
    u8 bAlternateSetting;
    u8 bNumEndpoints;
    u8 bInterfaceClass;
    u8 bInterfaceSubClass;
    u8 bInterfaceProtocol;
    u8 iInterface;
} PACKED usb_ifDesc_t;

#define USB_CHECK_IF_TYPE(intf,class,subclass,protocol) ((intf)->bInterfaceClass==(class) && (intf)->bInterfaceSubClass==(subclass) && (intf)->bInterfaceProtocol==(protocol))


typedef struct usb_epDesc_ {
    u8 bLength;
    u8 bDescriptorType;
    u8 bEndpointAddress;
    u8 bmAttributes;
    u16 wMaxPacketSize;
    u8 bInterval;
} PACKED usb_epDesc_t;


/* bEndpointAddress */
#define USB_EP_DIR_MASK  0x80
#define USB_EP_NUM_MASK  0x0F
/* bmAttributes */
#define USB_EP_TYPE_MASK 0x03
#define USB_EP_CTRL 0
#define USB_EP_ISOC 1
#define USB_EP_BULK 2
#define USB_EP_INTR 3
#define USB_EP_MAX_PACKET_MASK 0x03FF
#define USB_EP_MULT_MASK 0x0C00
#define USB_EP_MULT_SHIFT 11


typedef struct usb_strDesc_ {
	u8 bLength;
	u8 bDescriptorType;
	u16 wData[1];
} PACKED usb_strDesc_t;


typedef struct usb_hubDesc_ {
	u8 bDescLength;
	u8 bDescriptorType;
	u8 bNbrPorts;
	u16 wHubCharacteristics;
	u8 bPwrOn2PwrGood;
	u8 bHubContrCurrent;
	u8 DeviceRemovable;
	u8 PortPwrCtrlMask;
} PACKED usb_hubDesc_t;

usb_ifDesc_t* usb_cfgIfDesc(usb_cfgDesc_t* cfg,u8 ifNum);
usb_epDesc_t* usb_cfgEpDesc(usb_cfgDesc_t* cfg,u8 ifNum,u8 epNum);
usb_epDesc_t* usb_cfgSkipEndpoints(usb_epDesc_t* ep,u8 num);


#define USB_TIMEOUT 500000


/* Hub specification definitions */
#define USB_PORT_RESET 4
#define USB_PORT_POWER 8
#define USB_C_PORT_CONN 16
#define USB_C_PORT_RESET 20

#define USB_PORT_STATUS_CONN_MASK 0x01
#define USB_PORT_STATUS_RESET_MASK 0x10
//#define USB_PORT_STATUS_LOW_SPEED_MASK 0x0200
//#define USB_PORT_STATUS_HIGH_SPEED_MASK 0x0400
#define USB_PORT_STATUS_SPEED_MASK 0x0600
#define USB_PORT_STATUS_SPEED_SHIFT 9


#endif
