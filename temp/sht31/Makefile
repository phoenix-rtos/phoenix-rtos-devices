#
# Makefile for Phoenix-RTOS simple sensor devices
#
# Copyright 2021 Phoenix Systems
#
#

# by default use i2c message implementation (can be overridden to use custom target i2c lib directly, eg. `make I2C_LIB=libimx6ull-i2c temp-nct75`)
I2C_LIB ?= libi2c-msg

NAME := sht31
LOCAL_SRCS := sht31.c
DEP_LIBS := $(I2C_LIB)
include $(binary.mk)

NAME := sht31-tool
LOCAL_SRCS := sht31-tool.c
LOCAL_INSTALL_PATH := /usr/bin
include $(binary.mk)