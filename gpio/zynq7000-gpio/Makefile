#
# Makefile for Phoenix-RTOS zynq7000-gpio driver
#
# Copyright 2022 Phoenix Systems
#

# zynq7000 GPIO message API
NAME := libzynq7000-gpio-msg
LOCAL_HEADERS := zynq7000-gpio-msg.h
LOCAL_SRCS := libzynq7000-gpio-msg.c
include $(static-lib.mk)

# zynq7000 GPIO server
NAME := zynq7000-gpio
LOCAL_SRCS := gpio.c gpiosrv.c
include $(binary.mk)
