#
# Makefile for Phoenix-RTOS pc-ata driver
#
# Copyright 2018, 2019 Phoenix Systems
#

$(PREFIX_PROG)pc-ata: $(addprefix $(PREFIX_O)storage/pc-ata/, atadrv.o atasrv.o mbr.o) #$(PREFIX_A)libext2.a
	$(LINK)

$(PREFIX_A)libatadrv.a: $(PREFIX_O)storage/pc-ata/atadrv.o
	$(ARCH)

$(PREFIX_H)atasrv.h: storage/pc-ata/atasrv.h
	$(HEADER)

all: $(PREFIX_PROG_STRIPPED)pc-ata $(PREFIX_A)libatadrv.a $(PREFIX_H)atasrv.h
