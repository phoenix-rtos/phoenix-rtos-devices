#
# Part of makefile for Phoenix-RTOS sensor manager
#
# Copyright 2025 Phoenix Systems
#

# Simulated imu sensor
NAME := imu_sim
SENSORHUB_DRIVERS_ALL += $(NAME)
LOCAL_SRCS := imu_sim.c
DEPS := $(SENSORHUB_COMMON_DEPS)
include $(static-lib.mk)


# STM lsm9dsxx imu part sensor
NAME := lsm9dsxx_imu
SENSORHUB_DRIVERS_ALL += $(NAME)
LOCAL_SRCS := lsm9dsxx.c
DEPS := $(SENSORHUB_COMMON_DEPS)
include $(static-lib.mk)


# TDK IvenSense MPU6000 imu sensor
NAME := mpu6000
SENSORHUB_DRIVERS_ALL += $(NAME)
LOCAL_SRCS := mpu6000.c
DEPS := $(SENSORHUB_COMMON_DEPS)
include $(static-lib.mk)
