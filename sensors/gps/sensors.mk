#
# Part of makefile for Phoenix-RTOS sensor manager
#
# Copyright 2025 Phoenix Systems
#

# common gps library. Gps sensors` dependency
NAME := libgpscommon
LOCAL_SRCS := common.c nmea.c
LOCAL_HEADERS_DIR := nothing
DEP_LIBS := libsensors
SENSORS_DEPS += libgpscommon
include $(static-lib.mk)


# PA6H gps sensor
NAME := pa6h
LOCAL_SRCS := pa6h.c
LOCAL_HEADERS_DIR := nothing
$(eval $(call SENSORS_ADD_DEPS, libsensors libgpscommon))
include $(static-lib.mk)


# UBLOX gps sensor
NAME := ubx
LOCAL_SRCS := ubx.c
LOCAL_HEADERS_DIR := nothing
$(eval $(call SENSORS_ADD_DEPS, libsensors libgpscommon))
include $(static-lib.mk)
