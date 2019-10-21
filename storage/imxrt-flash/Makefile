#
# Phoenix-RTOS
#
# flash driver
#
# Copyright 2019 Phoenix Systems
# Author: Hubert Buczynski
#

$(PREFIX_A)libflash.a: $(addprefix $(PREFIX_O)storage/imxrt-flash/, flashdrv.o rom_api.o)
	$(ARCH)

$(PREFIX_H)flashdrv.h: storage/imxrt-flash/flashdrv.h
	$(HEADER)

$(PREFIX_H)rom_api.h: storage/imxrt-flash/rom_api.h
	$(HEADER)

all: $(PREFIX_A)libflash.a $(PREFIX_H)flashdrv.h $(PREFIX_H)rom_api.h
