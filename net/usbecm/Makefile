#
# Makefile for Phoenix-RTOS USB CDC ECM driver
#
# Copyright 2025 Phoenix Systems
#

NAME := libusbdrv-usbecm
LOCAL_SRCS := usbecm.c
LOCAL_HEADERS := usbecm.h
include $(static-lib.mk)

NAME := usbecm
LOCAL_SRCS := usbecm.c srv.c
LIBS := libusb
include $(binary.mk)
