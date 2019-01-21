#
# Makefile for Phoenix-RTOS stm32-multi
#
# Copyright 2018 Phoenix Systems
#


$(PREFIX_PROG)stm32-multi: $(addprefix $(PREFIX_O)multi/stm32-multi/, adc.o gpio.o i2c.o lcd.o rcc.o rtc.o stm32-multi.o flash.o uart.o spi.o)
	$(LINK)

all: $(PREFIX_PROG_STRIPPED)stm32-multi
