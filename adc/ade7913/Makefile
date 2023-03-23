#
# Makefile for ade7913
#
# Copyright 2021 Phoenix Systems
# Author: Marcin Baran
#
# %LICENSE%
#

NAME := ade7913-test
LOCAL_SRCS := gpio.c ade7913.c ade7913-test.c
DEPS := imxrt-multi

include $(binary.mk)

NAME := ade7913-driver
LOCAL_SRCS := gpio.c ade7913.c ade7913-driver.c flexpwm.c
LOCAL_HEADERS := adc-api-ade7913.h
DEPS := imxrt-multi
DEP_LIBS := libimxrt-edma

include $(binary.mk)
