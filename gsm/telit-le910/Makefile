#
# Makefile for Phoenix-RTOS telit-le910 driver
#
# Copyright 2018, 2019 Phoenix Systems
#

$(PREFIX_PROG)telit-le910: $(PREFIX_O)gsm/telit-le910/telit-le910.o $(PREFIX_A)libusb.a
	$(LINK)

$(PREFIX_PROG)sms: $(PREFIX_O)gsm/telit-le910/sms.o
	$(LINK)

all: $(PREFIX_PROG_STRIPPED)telit-le910 $(PREFIX_PROG_STRIPPED)sms

