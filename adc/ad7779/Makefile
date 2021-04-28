#
# Makefile for AD7779 driver
#
# Copyright 2019, 2020 Phoenix Systems
#

NAME := ad7779
LOCAL_SRCS := ad7779.c ad7779-driver.c
LOCAL_HEADERS := adc-api.h
DEP_LIBS := libimxrt-edma
DEPS := imxrt-multi

include $(binary.mk)

NAME := ad7779-test
LOCAL_SRCS := ad7779-test.c
DEPS := ad7779

include $(binary.mk)
