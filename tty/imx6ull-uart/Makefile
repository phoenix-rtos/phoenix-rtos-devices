#
# Makefile for Phoenix-RTOS imx6ull-uart driver
#
# Copyright 2018, 2019 Phoenix Systems
#

$(PREFIX_PROG)imx6ull-uart: $(PREFIX_O)tty/imx6ull-uart/imx6ull-uart.o $(PREFIX_A)libtty.a
	$(LINK)

# FIXME: should be generated automatically by gcc -M
$(PREFIX_O)tty/imx6ull-uart/imx6ull-uart.o: $(PREFIX_H)libtty.h

all: $(PREFIX_PROG_STRIPPED)imx6ull-uart
