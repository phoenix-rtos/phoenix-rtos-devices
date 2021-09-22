#
# Makefile for Phoenix-RTOS rtc drivers
#
# Copyright 2021 Phoenix Systems
#

# common RTC API
NAME := rtc-common
LOCAL_HEADERS := rtc.h
include $(static-lib.mk)


# by default use i2c message implementation (can be overridden to use custom target i2c lib directly)
I2C_LIB ?= libi2c-msg

NAME := rtc-pcf85363a
LOCAL_SRCS := pcf85363a.c
DEPS := rtc-common
DEP_LIBS := $(I2C_LIB)
include $(binary.mk)
