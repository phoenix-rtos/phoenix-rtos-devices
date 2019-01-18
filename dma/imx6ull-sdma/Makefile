#
# Makefile for Phoenix-RTOS imx6ull-sdma driver
#
# Copyright 2018 Phoenix Systems
#

$(PREFIX_PROG)imx6ull-sdma: $(PREFIX_O)dma/imx6ull-sdma/imx6ull-sdma.o $(PREFIX_A)libsdma.a
	$(LINK)
	
$(PREFIX_A)libsdma.a: $(PREFIX_O)dma/imx6ull-sdma/libsdma.o
	$(ARCH)
	
$(PREFIX_H)sdma.h: dma/imx6ull-sdma/sdma.h
	$(HEADER)
	
$(PREFIX_H)sdma-api.h: dma/imx6ull-sdma/sdma-api.h
	$(HEADER)

all: $(PREFIX_PROG_STRIPPED)imx6ull-sdma $(PREFIX_H)sdma.h $(PREFIX_H)sdma-api.h
