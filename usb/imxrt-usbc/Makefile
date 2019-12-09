#
# Phoenix-RTOS
#
# usbc - usb module loader
#
# Copyright 2019 Phoenix Systems
# Author: Kamil Amanowicz, Hubert Buczynski
#

$(PREFIX_A)libusbc.a: $(addprefix $(PREFIX_O)usb/, imx-usbc/hid.o imx-usbc/cdc.o imx-usbc/imxdevice.o imxrt-usbc/imxrt-usbclient.o)
	$(ARCH)

$(PREFIX_H)usbclient.h: usb/imx-usbc/usbclient.h
	$(HEADER)

$(PREFIX_H)hid.h: usb/imx-usbc/hid.h
	$(HEADER)

$(PREFIX_H)cdc.h: usb/imx-usbc/cdc.h
	$(HEADER)

all: $(PREFIX_A)libusbc.a $(PREFIX_H)usbclient.h $(PREFIX_H)hid.h $(PREFIX_H)cdc.h
