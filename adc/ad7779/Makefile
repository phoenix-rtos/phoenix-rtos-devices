#
# Makefile for AD7779 driver
#
# Copyright 2019, 2020 Phoenix Systems
#

NAME := ad7779
LOCAL_PATH := $(call my-dir)
SRCS := $(addprefix $(LOCAL_PATH), ad7779.c ad7779-driver.c)
LOCAL_HEADERS := adc-api-ad7779.h

ifneq (, $(findstring imxrt, $(TARGET)))
  SRCS += $(wildcard $(LOCAL_PATH)imxrt/*.c)
  DEP_LIBS := libimxrt-edma
  DEPS := imxrt-multi
  CFLAGS += -DAD7779_BUFFER_CNT=4 -DAD7779_BUFFER_SIZE=2048
else ifneq (, $(findstring imx6ull, $(TARGET)))
  SRCS += $(wildcard $(LOCAL_PATH)imx6ull/*.c)
  DEP_LIBS := libsdma libimx6ull-ecspi
  DEPS := imx6ull-gpio
  CFLAGS += -DAD7779_BUFFER_CNT=2 -DAD7779_BUFFER_SIZE=4096
else
$(NAME): ; $(error $(NAME) for $(TARGET) not implemented)
endif

include $(binary.mk)

NAME := ad7779-tool
SRCS := $(addprefix $(LOCAL_PATH), ad7779-tool.c)
DEPS := ad7779
include $(binary.mk)
