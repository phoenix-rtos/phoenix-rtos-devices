#
# Part of makefile for Phoenix-RTOS sensor manager
#
# Copyright 2025 Phoenix Systems
#

# Simulated imu sensor
NAME := imu_sim
LOCAL_SRCS := imu_sim.c
LOCAL_HEADERS_DIR := nothing
$(eval $(call SENSORS_ADD_DEPS, libsensors))
include $(static-lib.mk)


# STM lsm9dsxx imu part sensor
NAME := lsm9dsxx_imu
LOCAL_SRCS := lsm9dsxx.c
LOCAL_HEADERS_DIR := nothing
$(eval $(call SENSORS_ADD_DEPS, libsensors $(SENSORS_SPI_DEPS)))
include $(static-lib.mk)


# TDK IvenSense MPU6000 imu sensor
NAME := mpu6000
LOCAL_SRCS := mpu6000.c
LOCAL_HEADERS_DIR := nothing
$(eval $(call SENSORS_ADD_DEPS, libsensors $(SENSORS_SPI_DEPS)))
include $(static-lib.mk)
