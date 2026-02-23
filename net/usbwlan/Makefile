#
# Makefile for Phoenix-RTOS USB WLAN driver
#
# Copyright 2025 Phoenix Systems
#

NAME := libusbdrv-usbwlan
LOCAL_SRCS := usbwlan.c
LOCAL_HEADERS := usbwlan.h
include $(static-lib.mk)

NAME := usbwlan
LOCAL_SRCS := usbwlan.c srv.c
LIBS := libusb
include $(binary.mk)
