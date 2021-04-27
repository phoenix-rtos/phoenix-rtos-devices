#
# Makefile for Phoenix-RTOS imxrt117x-cm4 driver
#
# Copyright 2018, 2019 Phoenix Systems
#

$(PREFIX_PROG)imxrt117x-cm4: $(PREFIX_O)cpu/imxrt117x-cm4/imxrt117x-cm4.o
	$(LINK)

$(PREFIX_H)imxrt117x-cm4.h: cpu/imxrt117x-cm4/imxrt117x-cm4.h
	$(HEADER)

all: $(PREFIX_PROG_STRIPPED)imxrt117x-cm4 $(PREFIX_H)imxrt117x-cm4.h
