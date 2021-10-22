#
# Phoenix-RTOS
#
# usbc - usb module loader
#
# Copyright 2019 Phoenix Systems
# Author: Kamil Amanowicz, Hubert Buczynski

NAME := libusbclient
LOCAL_PATH := $(call my-dir)
SRCS := $(wildcard $(LOCAL_PATH)/imx-usbc/*.c)

ifneq (, $(findstring imxrt, $(TARGET)))
  SRCS += $(wildcard $(LOCAL_PATH)/imxrt-usbc/*.c)
else ifneq (, $(findstring imx6ull, $(TARGET)))
  SRCS += $(wildcard $(LOCAL_PATH)/imx6ull-usbc/*.c)
else
$(NAME): ; $(error $(NAME) for $(TARGET) not implemented)
endif

DEPS := usb

include $(static-lib.mk)


