#ifndef _USB_EHCI_H_
#define _USB_EHCI_H_

#include<lib/stdint.h>
#include "../hcd.h"

usb_dev_t* usb_ehciInitHost(usb_hcDesc_t* hd);

#endif
