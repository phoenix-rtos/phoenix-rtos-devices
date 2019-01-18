#
# Makefile for Phoenix-RTOS imx6ull-flash driver
#
# Copyright 2018, 2019 Phoenix Systems
#

$(PREFIX_PROG)imx6ull-flash-test: $(PREFIX_O)storage/imx6ull-flash/imx6ull-flash-test.o $(PREFIX_A)libflashdrv.a
	$(LINK)

$(PREFIX_A)libflashdrv.a: $(PREFIX_O)storage/imx6ull-flash/flashdrv.o
	$(ARCH)

all: $(PREFIX_PROG)imx6ull-flash-test
