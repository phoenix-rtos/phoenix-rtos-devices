#
# Makefile for Phoenix-RTOS pc-tty driver
#
# Copyright 2017, 2018, 2019 Phoenix Systems
# Copyright 2001, 2005-2006 Pawel Pisarczyk
#

$(PREFIX_PROG)pc-tty: $(addprefix $(PREFIX_O)tty/pc-tty/, ttypc.o ttypc_vt.o ttypc_vtf.o ttypc_vga.o ttypc_kbd.o ttypc_bioskbd.o) $(PREFIX_A)libtty.a
	$(LINK)

all: $(PREFIX_PROG_STRIPPED)pc-tty
