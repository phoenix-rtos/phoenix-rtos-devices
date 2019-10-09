#
# Phoenix-RTOS
#
# usbc - usb module loader
#
# Copyright 2019 Phoenix Systems
# Author: Kamil Amanowicz, Hubert Buczynski
#

$(PREFIX_A)libusb.a: $(addprefix $(PREFIX_O)usb/, imx-usbc/imxdevice.o imxrt-usbc/imxrt-usbclient.o)
	$(ARCH)

$(PREFIX_H)usbclient.h: usb/imx-usbc/usbclient.h
	$(HEADER)

all: $(PREFIX_A)libusb.a $(PREFIX_H)usbclient.h
