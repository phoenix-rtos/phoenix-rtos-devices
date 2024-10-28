#
# Makefile for Phoenix-RTOS ehci
#
# Copyright 2018, 2019 Phoenix Systems
#

# FIXME: rename usb host component
NAME := libusbehci
LOCAL_SRCS := ehci.c ehci-hub.c phy-$(TARGET_FAMILY)-$(TARGET_SUBFAMILY).c

ifneq (,$(findstring imx,$(TARGET_SUBFAMILY)))
 LOCAL_CFLAGS += -DEHCI_IMX
endif

include $(static-lib.mk)
