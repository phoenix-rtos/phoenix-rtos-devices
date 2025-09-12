#
# Part of makefile for Phoenix-RTOS sensor manager
#
# Copyright 2025 Phoenix Systems
#

# Simulated magnetometer sensor
NAME := mag_sim
SENSORHUB_DRIVERS_ALL += $(NAME)
LOCAL_SRCS := mag_sim.c
DEPS := $(SENSORHUB_COMMON_DEPS)
include $(static-lib.mk)


# STM LIS2MDL sensor
NAME := lis2mdl
SENSORHUB_DRIVERS_ALL += $(NAME)
LOCAL_SRCS := lis2mdl.c
DEPS := $(SENSORHUB_COMMON_DEPS)
include $(static-lib.mk)

# STM LIS2MDL sensor
NAME := lis2mdl_multi
SENSORHUB_DRIVERS_ALL += $(NAME)
LOCAL_SRCS := lis2mdl_multi.c
DEPS := $(SENSORHUB_COMMON_DEPS)
include $(static-lib.mk)

# STM lsm9dsxx magnetometer part sensor
NAME := lsm9dsxx_mag
SENSORHUB_DRIVERS_ALL += $(NAME)
LOCAL_SRCS := lsm9dsxx.c
DEPS := $(SENSORHUB_COMMON_DEPS)
include $(static-lib.mk)
