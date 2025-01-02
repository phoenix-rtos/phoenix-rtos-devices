NAME := libuacpi
LOCAL_PATH := $(call my-dir)

# don't install headers internal to uacpi
LOCAL_HEADERS_DIR := nothing
CFLAGS += -I$(LOCAL_PATH)/include

SRCS := $(wildcard $(LOCAL_PATH)/source/*.c)
SRCS += $(wildcard $(LOCAL_PATH)/phoenix/*.c) 
CFLAGS += -DUACPI_PHYS_ADDR_IS_32BITS

include $(static-lib.mk)
