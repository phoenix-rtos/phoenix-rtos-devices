#
# Makefile for Phoenix-RTOS zynq-pwm driver
#
# Copyright 2022 Phoenix Systems
#

NAME := zynq-pwm
LOCAL_SRCS := pwm.c

include $(binary.mk)

NAME := libzynqpwm
LOCAL_SRCS := zynq-pwm-msg.c
LOCAL_HEADERS := zynq-pwm-msg.h

include $(static-lib.mk)
