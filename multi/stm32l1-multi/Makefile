#
# Makefile for Phoenix-RTOS stm32-multi
#
# Copyright 2018 Phoenix Systems
#

NAME := stm32l1-multi
LOCAL_SRCS = stm32l1-multi.c uart.c rcc.c gpio.c adc.c i2c.c lcd.c rtc.c flash.c spi.c exti.c dma.c
LOCAL_HEADERS := stm32l1-multi.h

# FIXME: adapt code to array-bounds checker 
LOCAL_CFLAGS := -Wno-array-bounds

include $(binary.mk)
