#
# Makefile for Phoenix-RTOS sensor manager
#
# Copyright 2022 Phoenix Systems
#

# Sensors Server API Library
NAME := libsensors
include $(static-lib.mk)

# Sensors SPI interface
NAME := libsensors-spi
LOCAL_HEADERS := sensors-spi.h
LOCAL_HEADERS_DIR := nothing  # do not reinstall libsensors.h
LOCAL_SRCS := libsensors-spi.c
DEPS := libzynq7000-gpio-msg libspi-msg
include $(static-lib.mk)

# Sensors Server
NAME := sensors
LOCAL_HEADERS_DIR := nothing
LOCAL_SRCS := sensors.c
LOCAL_SRCS += imu/lsm9dsxx.c imu/imu_sim.c
LOCAL_SRCS += baro/lps25xx.c baro/ms5611.c baro/baro_sim.c
LOCAL_SRCS += gps/pa6h.c gps/nmea.c gps/gps_sim.c
LOCAL_SRCS += mag/lsm9dsxx.c mag/mag_sim.c
LOCAL_SRCS += simsensor_common/event_queue.c simsensor_common/simsensor_reader.c simsensor_common/simsensor_generic.c
DEP_LIBS := libsensors libsensors-spi libzynq7000-gpio-msg libspi-msg
include $(binary.mk)
