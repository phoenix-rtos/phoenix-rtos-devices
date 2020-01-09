#
# Makefile for Phoenix-RTOS imx6ull-ehci
#
# Copyright 2018, 2019 Phoenix Systems
#

$(PREFIX_A)libusbehci.a: $(addprefix $(PREFIX_O)usb/imx6ull-ehci/, ehci.o dma.o phy.o)
	$(ARCH)
	
$(PREFIX_H)dma.h: usb/imx6ull-ehci/dma.h
	$(HEADER)

$(PREFIX_H)ehci.h: usb/imx6ull-ehci/ehci.h
	$(HEADER)

all: $(PREFIX_A)libusbehci.a $(PREFIX_H)dma.h $(PREFIX_H)ehci.h
