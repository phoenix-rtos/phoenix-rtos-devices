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
LOCAL_SRCS := libsensors-spi.c
DEPS := libzynq7000-gpio-msg libspi-msg
include $(static-lib.mk)

# Sensors Server
NAME := sensors
LOCAL_HEADERS_DIR := nothing
LOCAL_SRCS := sensors.c
LOCAL_SRCS += imu/lsm9dsxx.c
LOCAL_SRCS += baro/lps25xx.c
LOCAL_SRCS += gps/pa6h.c gps/nmea.c
LOCAL_SRCS += mag/lsm9dsxx.c
DEP_LIBS := libsensors libsensors-spi libzynq7000-gpio-msg libspi-msg
include $(binary.mk)
