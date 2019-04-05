#
# Makefile for Phoenix-RTOS imx6ull-ecspi driver
#
# Copyright 2018, 2019 Phoenix Systems
#

$(PREFIX_A)libecspi.a: $(PREFIX_O)spi/imx6ull-ecspi/libecspi.o
	$(ARCH)

$(PREFIX_H)ecspi.h: spi/imx6ull-ecspi/ecspi.h
	$(HEADER)

all: $(PREFIX_A)libecspi.a $(PREFIX_H)ecspi.h
