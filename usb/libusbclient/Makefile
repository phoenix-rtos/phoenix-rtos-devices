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
else ifneq (, $(findstring stm32n6, $(TARGET)))
  SRCS := $(wildcard $(LOCAL_PATH)/stm32n6-usbc/phy.c)
else
$(NAME): ; $(error $(NAME) for $(TARGET) not implemented)
endif

DEPS := usb

include $(static-lib.mk)


NAME := radek-usb
LOCAL_SRCS := stm32n6-usbc/main.c
LOCAL_HEADERS := stm32n6-usbc/phy.h
DEP_LIBS := libusbclient
include $(binary.mk)
