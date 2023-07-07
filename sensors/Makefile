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
SENSORS_ALL := imu/lsm9dsxx imu/mpu6000 mag/lsm9dsxx mag/lis2mdl baro/lps25xx baro/ms5611 gps/pa6h gps/ubx
SENSORS_SIM := imu/imu_sim.c mag/mag_sim.c baro/baro_sim.c gps/gps_sim.c gps/gps_sim.c
SENSORS_LOCAL := $(SENSORS_SIM) $(addsuffix .c, $(filter $(SENSORS_ALL), $(DEVICE_SENSORS)))

NAME := sensors
LOCAL_HEADERS_DIR := nothing
LOCAL_SRCS += sensors.c $(SENSORS_LOCAL) $(SENSORS_SIM)
LOCAL_SRCS += gps/nmea.c gps/common.c simsensor_common/event_queue.c simsensor_common/simsensor_reader.c simsensor_common/simsensor_generic.c
DEP_LIBS := libsensors libsensors-spi libzynq7000-gpio-msg libspi-msg
include $(binary.mk)
