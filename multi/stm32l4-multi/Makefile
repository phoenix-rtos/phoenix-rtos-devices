#
# Makefile for Phoenix-RTOS stm32-multi
#
# Copyright 2018, 2020 Phoenix Systems
#

LOCAL_PATH := $(call my-dir)

NAME := stm32l4-multi

LOCAL_SRCS = exti.c fs.c gpio.c i2c.c posixsrv.c rcc/common.c rcc/$(TARGET_SUBFAMILY).c rng.c rtc.c spi.c stm32l4-multi.c tty.c uart.c
ifeq ($(TARGET_SUBFAMILY),stm32l4x6)
  LOCAL_CFLAGS += -DMULTIDRV_STACKSZ=640
  LOCAL_SRCS += adc_l4.c flash.c
else ifeq ($(TARGET_SUBFAMILY),stm32n6)
  LOCAL_CFLAGS += -DMULTIDRV_STACKSZ=2048
  LOCAL_SRCS += adc_n6.c pwm_n6.c rcc/clockdef.c xspi/xspi_n6.c xspi/mce.c
else ifeq ($(TARGET_SUBFAMILY),stm32u3)
  LOCAL_CFLAGS += -DMULTIDRV_STACKSZ=2048
  LOCAL_SRCS += adc_n6.c rcc/clockdef.c
endif

LOCAL_HEADERS := stm32l4-multi.h
DEP_LIBS := libstm32l4-multi libtty libklog i2c-common
LIBS := libdummyfs libklog libposixsrv

include $(binary.mk)
