#
# Makefile for Phoenix-RTOS simple sensor devices
#
# Copyright 2023 Phoenix Systems
#
#

# by default use i2c message implementation (can be overridden to use custom target i2c lib directly, eg. `make I2C_LIB=libimx6ull-i2c mag-als31300`)
I2C_LIB ?= libi2c-msg

NAME := mag-als31300
LOCAL_SRCS := mag-als31300.c
DEP_LIBS := $(I2C_LIB)
include $(binary.mk)
