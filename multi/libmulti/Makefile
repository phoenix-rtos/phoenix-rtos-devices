NAME := libmulti

LOCAL_PATH := $(call my-dir)

LOCAL_HEADERS := $(wildcard include/*.h)

LOCAL_CFLAGS := -I$(LOCAL_PATH)/include

LOCAL_SRCS := libmulti.c gpio.c # TODO: i2c.c spi.c

include $(static-lib.mk)
