#
# Makefile for Phoenix-RTOS stm32-multi
#
# Copyright 2018 Phoenix Systems
#

MULTIDRV_OBJS = stm32-multi.o uart.o rcc.o
ifneq (, $(findstring l1, $(TARGET)))
	MULTIDRV_OBJS += adc.o gpio.o i2c.o lcd.o rtc.o flash.o spi.o exti.o
endif

$(PREFIX_PROG)stm32-multi: $(addprefix $(PREFIX_O)multi/stm32-multi/, $(MULTIDRV_OBJS))
	$(LINK)

all: $(PREFIX_PROG_STRIPPED)stm32-multi
