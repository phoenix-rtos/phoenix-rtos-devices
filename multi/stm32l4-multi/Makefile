#
# Makefile for Phoenix-RTOS stm32-multi
#
# Copyright 2018, 2020 Phoenix Systems
#

LOCAL_PATH := $(call my-dir)

NAME := stm32l4-multi

LOCAL_SRCS = adc.c fs.c gpio.c i2c.c posixsrv.c rng.c rtc.c spi.c stm32l4-multi.c tty.c uart.c
ifeq ($(TARGET_SUBFAMILY),stm32l4x6)
  LOCAL_SRCS += exti_l4.c flash.c rcc_l4.c
else ifeq ($(TARGET_SUBFAMILY),stm32n6)
  LOCAL_SRCS += clockdef_n6.c exti_n6.c rcc_n6.c
endif

LOCAL_HEADERS := stm32l4-multi.h
DEP_LIBS := libstm32l4-multi libtty libklog
LIBS := libdummyfs libklog libposixsrv

# FIXME: adapt code to array-bounds checker
LOCAL_CFLAGS := -Wno-array-bounds

include $(binary.mk)
