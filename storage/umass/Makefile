#
# Makefile for Phoenix-RTOS USB Mass Storage driver
#
# Copyright 2020, 2024 Phoenix Systems
#

ifeq ($(TARGET_FAMILY),ia32)
	UMASS_LIBS := libext2
	UMASS_CFLAGS := -DUMASS_MOUNT_EXT2
endif

NAME := libusbdrv-umass
LOCAL_SRCS := umass.c
LOCAL_CFLAGS += $(UMASS_CFLAGS)
include $(static-lib.mk)

NAME := umass
LOCAL_SRCS := umass.c srv.c
LIBS := libusb $(UMASS_LIBS)
LOCAL_CFLAGS += $(UMASS_CFLAGS)
include $(binary.mk)
