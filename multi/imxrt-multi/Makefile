#
# Makefile for Phoenix-RTOS imxrt-multi
#
# Copyright 2019 Phoenix Systems
#

NAME := imxrt-multi

LOCAL_SRCS = imxrt-multi.c common.c uart.c gpio.c spi.c
ifneq ($(TARGET_SUBFAMILY), imxrt117x)
  LOCAL_SRCS += i2c.c trng.c
endif
DEP_LIBS := libtty
LOCAL_HEADERS := imxrt-multi.h
include $(binary.mk)
