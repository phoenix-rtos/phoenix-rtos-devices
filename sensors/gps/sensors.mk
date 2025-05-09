#
# Part of makefile for Phoenix-RTOS sensor manager
#
# Copyright 2025 Phoenix Systems
#

# Simulated gps sensor
NAME := gps_sim
SENSORHUB_DRIVERS_ALL += $(NAME)
LOCAL_SRCS := gps_sim.c
LOCAL_HEADERS_DIR := nothing
DEPS := $(SENSORHUB_COMMON_DEPS)
include $(static-lib.mk)


# PA6H gps sensor
NAME := pa6h
SENSORHUB_DRIVERS_ALL += $(NAME)
LOCAL_SRCS := pa6h.c
DEPS := $(SENSORHUB_COMMON_DEPS)
include $(static-lib.mk)


# UBLOX gps sensor
NAME := ubx
SENSORHUB_DRIVERS_ALL += $(NAME)
LOCAL_SRCS := ubx.c
DEPS := $(SENSORHUB_COMMON_DEPS)
include $(static-lib.mk)
