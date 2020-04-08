#
# Makefile for Phoenix-RTOS pc-ata driver
#
# Copyright 2018, 2019 Phoenix Systems
#

$(PREFIX_PROG)pc-ata: $(addprefix $(PREFIX_O)storage/pc-ata/, ata.o atasrv.o mbr.o) $(PREFIX_A)libext2.a
	$(LINK)

$(PREFIX_A)libata.a: $(PREFIX_O)storage/pc-ata/ata.o
	$(ARCH)

all: $(PREFIX_PROG_STRIPPED)pc-ata $(PREFIX_A)libata.a
