#
# Makefile for Phoenix-RTOS imu server
#
# Copyright 2021 Phoenix Systems
#
#

# by default uses imx6ull i2c implementation
I2C_LIB ?= libimx6ull-i2c

NAME := imx6ull-imu
LOCAL_PATH := $(call my-dir)
SRCS := $(wildcard $(LOCAL_PATH)*.c)
DEP_LIBS := $(I2C_LIB)
include $(binary.mk)

# TODO: make an API header installed so the message structure is not unknown
