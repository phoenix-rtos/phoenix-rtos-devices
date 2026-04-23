#
# Phoenix-RTOS
#
# SHT3x I2C Humidity and Temperature Sensor Makefile
#
# Copyright 2026 Phoenix Systems
# Author: Michal Woyke
#
# SPDX-License-Identifier: BSD-3-Clause
#

# by default use i2c message implementation (can be overridden to use custom target i2c lib directly, eg. `make I2C_LIB=libimx6ull-i2c sht3x`)
I2C_LIB ?= libi2c-msg
LOCAL_HEADERS := sht3x-api.h

NAME := sht3x
LOCAL_SRCS := sht3x.c
DEP_LIBS := $(I2C_LIB)
include $(binary.mk)

NAME := sht3x-tool
LOCAL_SRCS := sht3x-tool.c
LOCAL_INSTALL_PATH := /usr/bin
include $(binary.mk)
