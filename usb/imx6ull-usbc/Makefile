#
# Phoenix-RTOS
#
# usbc - usb module loader
#
# Copyright 2018, 2019 Phoenix Systems
# Author: Kamil Amanowicz, Hubert Buczynski
#

$(PREFIX_A)libusbclient.a: $(addprefix $(PREFIX_O)usb/, imx-usbc/imxdevice.o imx6ull-usbc/imx6ull-usbclient.o) $(PREFIX_A)libusb.a
	$(ARCH)

$(PREFIX_H)usbclient.h: usb/imx-usbc/usbclient.h
	$(HEADER)

all: $(PREFIX_A)libusbclient.a $(PREFIX_H)usbclient.h
