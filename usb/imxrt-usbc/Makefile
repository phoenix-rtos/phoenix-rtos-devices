#
# Phoenix-RTOS
#
# usbc - usb module loader
#
# Copyright 2019 Phoenix Systems
# Author: Kamil Amanowicz, Hubert Buczynski
#

$(PREFIX_A)libusbclient.a: $(addprefix $(PREFIX_O)usb/, imx-usbc/desc_manager.o imx-usbc/controller.o imx-usbc/client.o imxrt-usbc/phy.o) $(PREFIX_A)libusb.a 
	$(ARCH)

$(PREFIX_H)usbclient.h: usb/common/usbclient.h
	$(HEADER)

all: $(PREFIX_A)libusbclient.a $(PREFIX_H)usbclient.h 
