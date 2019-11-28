#
# Phoenix-RTOS
#
# usbc - usb module loader
#
# Copyright 2018, 2019 Phoenix Systems
# Author: Kamil Amanowicz, Hubert Buczynski
#

$(PREFIX_A)libusbc.a: $(addprefix $(PREFIX_O)usb/, imx-usbc/imxdevice.o imx6ull-usbc/imx6ull-usbclient.o)
	$(ARCH)

$(PREFIX_H)usbclient.h: usb/imx-usbc/usbclient.h
	$(HEADER)

all: $(PREFIX_A)libusbc.a $(PREFIX_H)usbclient.h
