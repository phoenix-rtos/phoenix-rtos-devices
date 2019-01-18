#
# Makefile for Phoenix-RTOS imx6ull-gpio driver
#
# Copyright 2018, 2019 Phoenix Systems
#

$(PREFIX_PROG)imx6ull-gpio: $(PREFIX_O)gpio/imx6ull-gpio/imx6ull-gpio.o
	$(LINK)

$(PREFIX_H)imx6ull-gpio.h: gpio/imx6ull-gpio/imx6ull-gpio.h
	$(HEADER)

all: $(PREFIX_PROG_STRIPPED)imx6ull-gpio $(PREFIX_H)imx6ull-gpio.h
