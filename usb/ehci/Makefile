#
# Makefile for Phoenix-RTOS ehci
#
# Copyright 2018, 2019 Phoenix Systems
#

# FIXME: rename usb host component
NAME := libusbehci
LOCAL_SRCS := ehci.c ehci-hub.c phy-$(TARGET_FAMILY)-$(TARGET_SUBFAMILY).c

ifneq (,$(findstring imx,$(TARGET_SUBFAMILY)))
 CFLAGS += -DEHCI_IMX
endif

ifeq ($(TARGET_FAMILY),ia32)
 DEPS += libacpi
endif

include $(static-lib.mk)
