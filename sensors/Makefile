#
# Makefile for Phoenix-RTOS sensor manager
#
# Copyright 2022 Phoenix Systems
#

LOCAL_PATH := $(call my-dir)

# Sensors Server API Library
NAME := libsensors
include $(static-lib.mk)

# FIXME: should be stored and compiled outside 
# Sensors SPI interface
NAME := libsensors-spi
LOCAL_HEADERS := sensors-spi.h
LOCAL_HEADERS_DIR := nothing  # do not reinstall libsensors/*.h
LOCAL_SRCS := libsensors-spi.c
DEPS := libzynq7000-gpio-msg libspi-msg
include $(static-lib.mk)


# Simulated sesnsors common library
NAME := libsimsensors_common
SRCS := $(wildcard $(LOCAL_PATH)simsensor_common/*.c)
LOCAL_HEADERS_DIR := nothing  # do not reinstall libsensors/*.h
DEP_LIBS := libsensors
include $(static-lib.mk)


# FUNCTION WRAPPER FOR DEPENDENCIES INJECTION
# Should be used in sensors part-Makefiles instead for sensor libraries "DEP_LIBS" setting
# Adds dependencies for both local sensor static library building and for "sensors" binary building
define SENSORS_ADD_DEPS
DEP_LIBS := $(1)
SENSORS_DEPS := $(sort $(SENSORS_DEPS) $(1))
endef

# spi dependencies bundle for spi driven sensors
SENSORS_SPI_DEPS := libsensors-spi libzynq7000-gpio-msg libspi-msg

# include all sensor types part-Makefiles
ALL_SENSOR_MAKES := $(shell find . -mindepth 2 -name sensors.mk -not -path '*/.*')
include $(ALL_SENSOR_MAKES)

NAME := sensors
LOCAL_HEADERS_DIR := nothing # do not reinstall libsensors/*.h
SRCS := $(LOCAL_PATH)sensors.c
DEP_LIBS := libsensors libsimsensors_common
DEP_LIBS += $(SENSORS_SPI_DEPS) $(SENSORS_DEPS) 
LIBS := $(DEVICE_SENSORS)
include $(binary.mk)
