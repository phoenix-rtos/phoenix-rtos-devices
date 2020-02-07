#
# Makefile for Phoenix-RTOS imxrt-multi
#
# Copyright 2019 Phoenix Systems
#


$(PREFIX_PROG)imxrt-multi: $(addprefix $(PREFIX_O)multi/imxrt-multi/, uart.o gpio.o common.o spi.o i2c.o\
	imxrt-multi.o) $(PREFIX_A)libtty.a
	$(LINK)

$(PREFIX_O)multi/imxrt-multi/imxrt-multi.o: $(PREFIX_H)libtty.h


$(PREFIX_PROG)multi-tests: $(addprefix $(PREFIX_O)multi/imxrt-multi/tests/, multi_tests.o spi_tests.o)
	$(LINK)

all: $(PREFIX_PROG_STRIPPED)imxrt-multi $(PREFIX_PROG_STRIPPED)multi-tests
