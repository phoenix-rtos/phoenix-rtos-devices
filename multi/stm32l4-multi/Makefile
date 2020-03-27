#
# Makefile for Phoenix-RTOS stm32-multi
#
# Copyright 2018, 2020 Phoenix Systems
#

MULTIDRV_OBJS = stm32-multi.o uart.o rcc.o gpio.o spi.o adc.o rtc.o flash.o #i2c.o exti.o

$(PREFIX_PROG)stm32-multi: $(addprefix $(PREFIX_O)multi/stm32l4-multi/, $(MULTIDRV_OBJS))
	$(LINK)

$(PREFIX_H)stm32-multi.h: multi/stm32l4-multi/stm32-multi.h
	$(HEADER)

all: $(PREFIX_PROG_STRIPPED)stm32-multi $(PREFIX_H)stm32-multi.h
