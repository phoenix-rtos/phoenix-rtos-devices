#
# Makefile for Phoenix-RTOS sensor manager
#
# Copyright 2022 Phoenix Systems
#

LOCAL_PATH := $(call my-dir)

# Sensors Server Library
# FIXME: need platform specific dependency of libzynq7000-gpio-msg
NAME := libsensors
LOCAL_HEADERS_DIR := server-include
SRCS := $(wildcard $(LOCAL_PATH)common/*/*.c) $(LOCAL_PATH)locsensor.c
DEPS := libzynq7000-gpio-msg libspi-msg libsensorsclient
include $(static-lib.mk)

# Sensor Client API
# Separated for use on host applications
NAME := libsensorsclient
LOCAL_HEADERS_DIR := client-include
include $(static-lib.mk)

# Sensors application
SENSORHUB_COMMON_DEPS := libzynq7000-gpio-msg libspi-msg libsensors libsensorsclient
SENSORHUB_DRIVERS_ALL :=

ALL_SENSOR_MAKES := $(shell find . -mindepth 2 -name sensors.mk -not -path '*/.*')
include $(ALL_SENSOR_MAKES)

# This is the default sensors list. It is supposed to be overwritten on per-project basis.
SENSORHUB_DRIVERS_ENABLED ?= baro_sim imu_sim mag_sim gps_sim

SENSORHUB_DRIVERS_INTERNAL := $(filter $(SENSORHUB_DRIVERS_ENABLED), $(SENSORHUB_DRIVERS_ALL))
SENSORHUB_DRIVERS_EXTERNAL := $(filter-out $(SENSORHUB_DRIVERS_INTERNAL), $(SENSORHUB_DRIVERS_ENABLED))
$(info sensors enabled (internal [external]): $(SENSORHUB_DRIVERS_INTERNAL) [$(SENSORHUB_DRIVERS_EXTERNAL)])

# FIXME: need platform specific dependency of libzynq7000-gpio-msg
NAME := sensors
SRCS := $(LOCAL_PATH)sensors.c
DEP_LIBS := $(SENSORHUB_COMMON_DEPS) $(SENSORHUB_DRIVERS_INTERNAL)
LIBS := $(SENSORHUB_DRIVERS_EXTERNAL)
include $(binary.mk)
