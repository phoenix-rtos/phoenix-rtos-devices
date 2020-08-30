#
# Makefile for Phoenix-RTOS pc-uart driver
#
# Copyright 2017, 2018 Phoenix Systems
# Copyright 2001, 2005-2006 Pawel Pisarczyk
#

ifeq ($(TARGET_FAMILY),riscv64)
    UARTLL = uarthw-riscv
else ifeq ($(TARGET_FAMILY),ia32)
    UARTLL = uarthw-pc
endif

$(PREFIX_PROG)uart16550: $(PREFIX_O)tty/uart16550/uart16550.o $(PREFIX_O)tty/uart16550/$(UARTLL).o $(PREFIX_A)libtty.a
	$(LINK)

all: $(PREFIX_PROG_STRIPPED)uart16550
