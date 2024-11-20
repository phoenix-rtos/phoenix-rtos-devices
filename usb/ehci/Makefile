#
# Makefile for Phoenix-RTOS imx6ull-ehci
#
# Copyright 2018, 2019 Phoenix Systems
#

# FIXME: rename usb host component
NAME := libusbehci
# TODO use TARGET_FAMILY-TARGET_SUBFAMILY
LOCAL_SRCS := ehci.c ehci-hub.c phy-$(TARGET_SUBFAMILY).c

ifneq (,$(findstring imx,$(TARGET_SUBFAMILY)))
 CFLAGS += -DEHCI_IMX
endif

include $(static-lib.mk)
