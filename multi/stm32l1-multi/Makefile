#
# Makefile for Phoenix-RTOS stm32-multi
#
# Copyright 2018 Phoenix Systems
#

MULTIDRV_OBJS = stm32-multi.o uart.o rcc.o gpio.o adc.o i2c.o lcd.o rtc.o flash.o spi.o exti.o dma.o

$(PREFIX_PROG)stm32-multi: $(addprefix $(PREFIX_O)multi/stm32l1-multi/, $(MULTIDRV_OBJS))
	$(LINK)

$(PREFIX_H)stm32-multi.h: multi/stm32l1-multi/stm32-multi.h
	$(HEADER)

all: $(PREFIX_PROG_STRIPPED)stm32-multi $(PREFIX_H)stm32-multi.h
