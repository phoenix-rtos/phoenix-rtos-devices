#
# Phoenix-RTOS
#
# imx6ull-usb-loader - imx6ull usb module loader
#
# Copyright 2018 Phoenix Systems
# Author: Kamil Amanowicz
#

$(PREFIX_A)libusb-imx6ull.a: $(addprefix $(PREFIX_O)usb/imx6ull-usbc/, usbclient.o)
	$(ARCH)

$(PREFIX_H)usbclient.h: usb/imx6ull-usbc/usbclient.h
	$(HEADER)

all: $(PREFIX_A)libusb-imx6ull.a $(PREFIX_H)usbclient.h
