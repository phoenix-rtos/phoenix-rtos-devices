#
# Part of makefile for Phoenix-RTOS sensor manager
#
# Copyright 2025 Phoenix Systems
#

# Simulated baro sensor
NAME := baro_sim
LOCAL_SRCS := baro_sim.c
LOCAL_HEADERS_DIR := nothing
$(eval $(call SENSORS_ADD_DEPS, libsensors libsimsensors_common))
include $(static-lib.mk)


# TE Connectivity MS5611 barometer sensor
NAME := ms5611
LOCAL_SRCS := ms5611.c
LOCAL_HEADERS_DIR := nothing
$(eval $(call SENSORS_ADD_DEPS, libsensors $(SENSORS_SPI_DEPS)))
include $(static-lib.mk)


# STM LPS25xx barometer sensor
NAME := lps25xx
LOCAL_SRCS := lps25xx.c
LOCAL_HEADERS_DIR := nothing
$(eval $(call SENSORS_ADD_DEPS, libsensors $(SENSORS_SPI_DEPS)))
include $(static-lib.mk)
