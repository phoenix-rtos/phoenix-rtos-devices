#
# Makefile for Phoenix-RTOS sensor manager
#
# Copyright 2022 Phoenix Systems
#

LOCAL_PATH := $(call my-dir)

# Sensors Server API Library
NAME := libsensors
SRCS := $(wildcard $(LOCAL_PATH)common/*/*.c)
DEPS := libzynq7000-gpio-msg libspi-msg
include $(static-lib.mk)

# Sensors application
SENSORHUB_COMMON_DEPS := libzynq7000-gpio-msg libspi-msg libsensors
SENSORHUB_DRIVERS_ALL :=

ALL_SENSOR_MAKES := $(shell find . -mindepth 2 -name sensors.mk -not -path '*/.*')
include $(ALL_SENSOR_MAKES)

SENSORHUB_DRIVERS_ENABLED ?= baro_sim imu_sim mag_sim gps_sim
SENSORHUB_DRIVERS_INTERNAL := $(filter $(SENSORHUB_DRIVERS_ENABLED), $(SENSORHUB_DRIVERS_ALL))
SENSORHUB_DRIVERS_EXTERNAL := $(filter-out $(SENSORHUB_DRIVERS_INTERNAL), $(SENSORHUB_DRIVERS_ENABLED))
$(info sensors enabled (internal [external]): $(SENSORHUB_DRIVERS_INTERNAL) [$(SENSORHUB_DRIVERS_EXTERNAL)])

NAME := sensors
LOCAL_HEADERS_DIR := nothing # do not reinstall libsensors/*.h
SRCS := $(LOCAL_PATH)sensors.c
DEP_LIBS := $(SENSORHUB_COMMON_DEPS) $(SENSORHUB_DRIVERS_INTERNAL)
LIBS := $(SENSORHUB_DRIVERS_EXTERNAL)
include $(binary.mk)
