#
# Makefile for Phoenix-RTOS stm32-multi
#
# Copyright 2018, 2020 Phoenix Systems
#

MULTI_PATH = multi/stm32l4-multi/
MULTIDRV_OBJS = stm32-multi.o rcc.o rtc.o gpio.o spi.o adc.o flash.o exti.o i2c.o tty.o uart.o
MULTIDRV_LIBOBJS := $(addprefix libmulti/lib, dma.o spi.o i2c.o aes.o uart.o)

MULTIDRV_OBJS := $(addprefix $(PREFIX_O)$(MULTI_PATH), $(MULTIDRV_OBJS))
MULTIDRV_LIBOBJS := $(addprefix $(PREFIX_O)$(MULTI_PATH), $(MULTIDRV_LIBOBJS))
SRCHEADERS := $(patsubst $(MULTI_PATH)%,$(PREFIX_H)%,$(wildcard $(MULTI_PATH)libmulti/*.h))

$(MULTIDRV_OBJS): $(SRCHEADERS) $(PREFIX_H)libtty.h

$(PREFIX_PROG)stm32-multi: $(MULTIDRV_OBJS) $(PREFIX_A)libstm32l4-multi.a $(PREFIX_A)libtty.a
	$(LINK)

$(PREFIX_A)libstm32l4-multi.a: $(MULTIDRV_LIBOBJS) $(PREFIX_A)libtty.a
	$(ARCH)

$(PREFIX_H)%.h: $(MULTI_PATH)%.h
	$(HEADER)

all: $(PREFIX_A)libstm32l4-multi.a $(SRCHEADERS) $(PREFIX_PROG_STRIPPED)stm32-multi $(PREFIX_H)stm32-multi.h
