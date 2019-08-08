#
# Phoenix-RTOS
#
# usbc - usb module loader
#
# Copyright 2019 Phoenix Systems
# Author: Kamil Amanowicz, Hubert Buczynski
#


ifneq (, $(findstring imx6ull, $(TARGET)))
CFLAGS += -DTARGET_IMX6ULL 

$(PREFIX_A)libusb.a: $(addprefix $(PREFIX_O)usb/device/, imxdevice.o imx6ull-usbclient.o)
	$(ARCH)
    
else ifneq (, $(findstring imxrt, $(TARGET)))
CFLAGS += -DTARGET_IMXRT

$(PREFIX_A)libusb.a: $(addprefix $(PREFIX_O)usb/device/, imxdevice.o imxrt-usbclient.o)
	$(ARCH)
endif


$(PREFIX_H)usbclient.h: usb/device/usbclient.h
	$(HEADER)

all: $(PREFIX_A)libusb.a $(PREFIX_H)usbclient.h
