#
# Phoenix-RTOS
#
# MCP33131/21/11-XX adc Makefile
#
# Copyright 2026 Phoenix Systems
# Author: Michal Woyke
#
# SPDX-License-Identifier: BSD-3-Clause
#

NAME := mcp331
LOCAL_PATH := $(call my-dir)
SRCS := $(addprefix $(LOCAL_PATH), mcp331.c)
LOCAL_HEADERS := mcp331-api.h

ifneq (, $(findstring imx6ull, $(TARGET)))
  SRCS += $(wildcard $(LOCAL_PATH)imx6ull/*.c)
  DEP_LIBS := libimx6ull-ecspi
  DEPS := imx6ull-gpio
else
$(NAME): ; $(error $(NAME) for $(TARGET) not implemented)
endif

include $(binary.mk)

NAME := mcp331-tool
SRCS := $(addprefix $(LOCAL_PATH), mcp331-tool.c)
DEPS := mcp331
include $(binary.mk)
