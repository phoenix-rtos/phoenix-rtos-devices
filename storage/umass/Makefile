#
# Makefile for Phoenix-RTOS USB Mass Storage driver
#
# Copyright 2020 Phoenix Systems
#

NAME := umass
LOCAL_SRCS := umass.c
LIBS := libusb

ifeq ($(TARGET_FAMILY),ia32)
	LIBS += libext2
	CFLAGS += -DUMASS_MOUNT_EXT2
endif

include $(binary.mk)
