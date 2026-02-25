#
# Makefile for Phoenix-RTOS stm32-multi
#
# Copyright 2018, 2020 Phoenix Systems
#

LOCAL_PATH := $(call my-dir)

NAME := stm32l4-multi

LOCAL_SRCS = fs.c gpio.c i2c.c posixsrv.c rng.c rtc.c spi.c stm32l4-multi.c tty.c uart.c coredumpsrv.c
ifeq ($(TARGET_SUBFAMILY),stm32l4x6)
  LOCAL_CFLAGS += -DMULTIDRV_STACKSZ=640
  LOCAL_SRCS += adc_l4.c exti_l4.c flash.c rcc_l4.c
else ifeq ($(TARGET_SUBFAMILY),stm32n6)
  LOCAL_CFLAGS += -DMULTIDRV_STACKSZ=2048
  LOCAL_SRCS += adc_n6.c clockdef_n6.c exti_n6.c rcc_n6.c pwm_n6.c
endif

LOCAL_HEADERS := stm32l4-multi.h
DEP_LIBS := libstm32l4-multi libtty libklog i2c-common
LIBS := libdummyfs libklog libposixsrv

ifeq ($(COREDUMP_DISABLE),1)
	LOCAL_CFLAGS += -DBUILTIN_COREDUMPSRV=0
else
	LIBS += libcoredumpsrv
endif

include $(binary.mk)
