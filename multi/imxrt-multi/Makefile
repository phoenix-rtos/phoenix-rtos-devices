#
# Makefile for Phoenix-RTOS imxrt-multi
#
# Copyright 2019 Phoenix Systems
#

NAME := imxrt-multi

LOCAL_PATH := $(call my-dir)

LOCAL_SRCS = imxrt-multi.c common.c uart.c gpio.c spi.c i2c.c fs.c posixsrv.c
ifneq ($(TARGET_SUBFAMILY), imxrt117x)
  LOCAL_SRCS += trng.c
else
  LOCAL_SRCS += cm4.c
endif
DEP_LIBS := libtty libklog libpseudodev i2c-common
LIBS := libdummyfs libklog libpseudodev libposixsrv
LOCAL_HEADERS := imxrt-multi.h

include $(binary.mk)
