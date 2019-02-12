#
# Makefile for Phoenix-RTOS imx6ull-flash driver
#
# Copyright 2018, 2019 Phoenix Systems
#

$(PREFIX_PROG)imx6ull-flash: $(addprefix $(PREFIX_O)storage/imx6ull-flash/, flashdrv.o flashsrv.o) $(PREFIX_A)libjffs2.a
	$(LINK)

$(PREFIX_A)libflashdrv.a: $(PREFIX_O)storage/imx6ull-flash/flashdrv.o
	$(ARCH)

all: $(PREFIX_PROG_STRIPPED)imx6ull-flash $(PREFIX_A)libflashdrv.a
