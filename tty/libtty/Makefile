#
# Makefile for Phoenix-RTOS libtty
#
# Copyright 2018, 2019 Phoenix Systems
#

$(PREFIX_A)libtty.a: $(addprefix $(PREFIX_O)tty/libtty/, libtty.o libtty_disc.o)
	$(ARCH)
	
$(PREFIX_H)libtty.h: tty/libtty/libtty.h
	$(HEADER)
	
all: $(PREFIX_A)libtty.a $(PREFIX_H)libtty.h
