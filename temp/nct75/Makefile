#
# Makefile for Phoenix-RTOS simple sensor devices
#
# Copyright 2021 Phoenix Systems
#
#

# by default use i2c message implementation (can be overridden to use custom target i2c lib directly, eg. `make I2C_LIB=libimx6ull-i2c temp-nct75`)
I2C_LIB ?= libi2c-msg

NAME := temp-nct75
LOCAL_SRCS := temp-nct75.c
DEP_LIBS := $(I2C_LIB)
include $(binary.mk)
