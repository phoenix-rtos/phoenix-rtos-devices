#
# Part of makefile for Phoenix-RTOS sensor manager
#
# Copyright 2025 Phoenix Systems
#

# Simulated magnetometer sensor
NAME := mag_sim
LOCAL_SRCS := mag_sim.c
LOCAL_HEADERS_DIR := nothing
$(eval $(call SENSORS_ADD_DEPS, libsensors))
include $(static-lib.mk)


# STM LIS2MDL sensor
NAME := lis2mdl
LOCAL_SRCS := lis2mdl.c
LOCAL_HEADERS_DIR := nothing
$(eval $(call SENSORS_ADD_DEPS, libsensors $(SENSORS_SPI_DEPS)))
include $(static-lib.mk)


# STM lsm9dsxx magnetometer part sensor
NAME := lsm9dsxx_mag
LOCAL_SRCS := lis2mdl.c
LOCAL_HEADERS_DIR := nothing
$(eval $(call SENSORS_ADD_DEPS, libsensors $(SENSORS_SPI_DEPS)))
include $(static-lib.mk)
