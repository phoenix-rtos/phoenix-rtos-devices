#
# Phoenix-RTOS
#
# flash driver
#
# Copyright 2019 Phoenix Systems
# Author: Hubert Buczynski
#

$(PREFIX_PROG)imxrt-flash: $(addprefix $(PREFIX_O)storage/imxrt-flash/, flashsrv.o flashdrv.o rom_api.o)
	$(LINK)

$(PREFIX_A)libflashdrv.a: $(addprefix $(PREFIX_O)storage/imxrt-flash/, flashdrv.o rom_api.o)
	$(ARCH)

$(PREFIX_H)flashsrv.h: storage/imxrt-flash/flashsrv.h
	$(HEADER)

all: $(PREFIX_PROG_STRIPPED)imxrt-flash $(PREFIX_A)libflashdrv.a $(PREFIX_H)flashsrv.h 

