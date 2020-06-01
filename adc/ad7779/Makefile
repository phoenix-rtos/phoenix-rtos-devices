#
# Makefile for AD7779 driver
#
# Copyright 2019, 2020 Phoenix Systems
#

$(PREFIX_PROG)ad7779-test: $(addprefix $(PREFIX_O)adc/ad7779/, ad7779-test.o)
	$(LINK)

$(PREFIX_PROG)ad7779-driver: $(addprefix $(PREFIX_O)adc/ad7779/, ad7779.o ad7779-driver.o) \
	$(PREFIX_O)dma/imxrt-edma/imxrt-edma.o
	$(LINK)

$(PREFIX_H)adc-api.h: adc/ad7779/adc-api.h
	$(HEADER)

all: $(PREFIX_PROG_STRIPPED)ad7779-test $(PREFIX_PROG_STRIPPED)ad7779-driver $(PREFIX_H)adc-api.h
