#
# Makefile for Phoenix-RTOS psd (Phoenix Serial Downloader)
#
# Copyright 2019 Phoenix Systems
#

$(PREFIX_PROG)cdc-client: $(addprefix $(PREFIX_O)usb/, tests/cdc-client.o) $(PREFIX_A)libusbc.a
	$(LINK)

all: $(PREFIX_PROG_STRIPPED)cdc-client
