#
# Part of makefile for Phoenix-RTOS sensor manager
#
# Copyright 2025 Phoenix Systems
#

# Simulated baro sensor
NAME := baro_sim
SENSORHUB_DRIVERS_ALL += $(NAME)
LOCAL_SRCS := baro_sim.c
DEPS := $(SENSORHUB_COMMON_DEPS)
include $(static-lib.mk)


# TE Connectivity MS5611 barometer sensor
NAME := ms5611
SENSORHUB_DRIVERS_ALL += $(NAME)
LOCAL_SRCS := ms5611.c
DEPS := $(SENSORHUB_COMMON_DEPS)
include $(static-lib.mk)


# STM LPS25xx barometer sensor
NAME := lps25xx
SENSORHUB_DRIVERS_ALL += $(NAME)
LOCAL_SRCS := lps25xx.c
DEPS := $(SENSORHUB_COMMON_DEPS)
include $(static-lib.mk)


# STM LPS22xx barometer sensor
NAME := lps22xx
SENSORHUB_DRIVERS_ALL += $(NAME)
LOCAL_SRCS := lps22xx.c
DEPS := $(SENSORHUB_COMMON_DEPS)
include $(static-lib.mk)
