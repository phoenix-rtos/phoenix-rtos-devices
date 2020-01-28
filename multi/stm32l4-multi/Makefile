#
# Makefile for Phoenix-RTOS stm32-multi
#
# Copyright 2018, 2020 Phoenix Systems
#

MULTIDRV_OBJS = stm32-multi.o uart.o rcc.o gpio.o #adc.o i2c.o rtc.o flash.o spi.o exti.o

$(PREFIX_PROG)stm32-multi: $(addprefix $(PREFIX_O)multi/stm32l4-multi/, $(MULTIDRV_OBJS))
	$(LINK)

all: $(PREFIX_PROG_STRIPPED)stm32-multi
