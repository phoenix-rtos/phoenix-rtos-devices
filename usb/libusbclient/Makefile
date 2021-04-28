#
# Phoenix-RTOS
#
# usbc - usb module loader
#
# Copyright 2019 Phoenix Systems
# Author: Kamil Amanowicz, Hubert Buczynski

NAME := libusbclient
SRCS := $(wildcard usb/libusbclient/imx-usbc/*.c)
LOCAL_PATH := $(call my-dir)

ifneq (, $(findstring imxrt, $(TARGET)))
  SRCS += $(wildcard $(LOCAL_PATH)/imxrt-usbc/*.c)
else ifneq (, $(findstring imx6ull, $(TARGET)))
  SRCS += $(wildcard $(LOCAL_PATH)/imx6ull-usbc/*.c)
else
$(NAME): ; $(error $(NAME) for $(TARGET) not implemented)
endif


SRCS += $(wildcard usb/libusbclient/$(USBC_PHY_TYPE)-usbc/*.c)
LOCAL_HEADERS := usbclient.h
DEPS := usb

include $(static-lib.mk)


