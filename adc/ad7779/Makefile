#
# Makefile for AD7779 driver
#
# Copyright 2019, 2020 Phoenix Systems
#

NAME := ad7779
LOCAL_PATH := $(call my-dir)
SRCS := $(addprefix $(LOCAL_PATH), ad7779.c ad7779-driver.c)
LOCAL_HEADERS := adc-api.h

ifneq (, $(findstring imxrt, $(TARGET)))
  SRCS += $(wildcard $(LOCAL_PATH)imxrt/*.c)
  DEP_LIBS := libimxrt-edma
  DEPS := imxrt-multi
  CFLAGS += -DAD7779_BUFFER_CNT=32 -DAD7779_BUFFER_SIZE=2048
else
$(NAME): ; $(error $(NAME) for $(TARGET) not implemented)
endif

include $(binary.mk)
