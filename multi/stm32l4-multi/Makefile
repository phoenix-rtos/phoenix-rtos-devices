#
# Makefile for Phoenix-RTOS stm32-multi
#
# Copyright 2018, 2020 Phoenix Systems
#

MULTI_PATH = multi/stm32l4-multi/
MULTIDRV_OBJS = stm32-multi.o rcc.o rtc.o uart.o gpio.o spi.o adc.o flash.o exti.o #i2c.o
MULTIDRV_LIBOBJS = $(addprefix lib/lib, uart.o dma.o spi.o)
SRCHEADERS := $(shell find $(MULTI_PATH)lib -name \*.h)

headers: $(SRCHEADERS)
	@echo "HEADERS $(PREFIX_H)stm32l4-multi/*"; \
	mkdir -p "$(PREFIX_H)stm32l4-multi/"; \
	cp -a $(SRCHEADERS) "$(PREFIX_H)stm32l4-multi/";

$(PREFIX_PROG)stm32-multi: $(addprefix $(PREFIX_O)multi/stm32l4-multi/, $(MULTIDRV_OBJS)) $(PREFIX_A)libstm32l4-multi.a $(PREFIX_A)libtty.a
	$(LINK)

$(PREFIX_A)libstm32l4-multi.a: $(addprefix $(PREFIX_O)multi/stm32l4-multi/, $(MULTIDRV_LIBOBJS)) $(PREFIX_H)libtty.h $(PREFIX_A)libtty.a
	$(ARCH)

$(PREFIX_H)stm32-multi.h: multi/stm32l4-multi/stm32-multi.h
	$(HEADER)

.PHONY: headers

all: $(PREFIX_A)libstm32l4-multi.a headers $(PREFIX_PROG_STRIPPED)stm32-multi $(PREFIX_H)stm32-multi.h
