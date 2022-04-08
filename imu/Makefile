#
# Makefile for Phoenix-RTOS imu server
#
# Copyright 2021 Phoenix Systems
#
#

# by default use i2c message implementation (can be overridden to use custom target i2c lib directly, eg. `make I2C_LIB=libimx6ull-i2c temp-nct75`)
I2C_LIB ?= libimx6ull-i2c

NAME := imu
LOCAL_SRCS := main.c communication.c icm20xxx.c ak099xx.c
LOCAL_HEADERS := communication.h icm20xxx.h ak099xx.h
DEP_LIBS := $(I2C_LIB)
include $(binary.mk)
