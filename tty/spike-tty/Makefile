#
# Makefile for Phoenix-RTOS riscv-htiftty driver
#
# Copyright 2020 Phoenix Systems
#

$(PREFIX_PROG)spike-tty: $(PREFIX_O)tty/spike-tty/spike-tty.o $(PREFIX_A)libtty.a
	$(LINK)
#-Ttext 0x80000

all: $(PREFIX_PROG_STRIPPED)spike-tty
