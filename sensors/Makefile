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
LOCAL_HEADERS_DIR := nothing  # do not reinstall libsensors/*.h
LOCAL_SRCS := libsensors-spi.c
DEPS := libzynq7000-gpio-msg libspi-msg
include $(static-lib.mk)

# Sensors Server

#Find all local sensor files
SENSOR_DIRS := imu/ mag/ baro/ gps/
SENSOR_SRCS := $(patsubst sensors/%, %, $(wildcard $(addsuffix *.c, $(addprefix sensors/, $(SENSOR_DIRS)))))

SENSORS_SIM := $(filter %sim.c, $(SENSOR_SRCS))
SENSORS_HW := $(filter-out %sim.c, $(SENSOR_SRCS))
SENSORS_LOCAL := $(SENSORS_SIM) $(addsuffix .c, $(filter $(patsubst %.c, %, $(SENSORS_HW)), $(DEVICE_SENSORS)))

NAME := sensors
LOCAL_HEADERS_DIR := nothing # do not reinstall libsensors/*.h
LIBS := $(SENSORS_PROJECT_DEPS) # Project-specific sensors .a files and their dependencies
LOCAL_SRCS += sensors.c $(SENSORS_LOCAL)
LOCAL_SRCS += gps/nmea.c gps/common.c simsensor_common/event_queue.c simsensor_common/simsensor_reader.c simsensor_common/simsensor_generic.c
DEP_LIBS := libsensors libsensors-spi libzynq7000-gpio-msg libspi-msg
include $(binary.mk)
