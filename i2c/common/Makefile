#
# Makefile for Phoenix-RTOS i2c API
#
# Copyright 2021 Phoenix Systems
#

# i2c API
NAME := i2c-common
LOCAL_HEADERS := i2c.h i2c-msg.h
include $(static-lib.mk)


# i2c API message implementation
NAME := libi2c-msg
DEPS := i2c-common
LOCAL_SRCS := libi2c-msg.c
include $(static-lib.mk)
