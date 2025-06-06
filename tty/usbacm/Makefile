#
# Makefile for Phoenix-RTOS USB CDC ACM driver
#
# Copyright 2021, 2024 Phoenix Systems
#

NAME := libusbdrv-usbacm
LOCAL_SRCS := usbacm.c
include $(static-lib.mk)

NAME := usbacm
LOCAL_SRCS := usbacm.c srv.c
LIBS := libusb
include $(binary.mk)
