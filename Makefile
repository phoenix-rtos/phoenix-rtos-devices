#
# Makefile for phoenix-rtos-devices
#
# Copyright 2018 Phoenix Systems
#
# %LICENSE%
#

SIL ?= @
MAKEFLAGS += --no-print-directory

TARGET ?= ia32-qemu
#TARGET ?= armv7-stm32
#TARGET ?= arm-imx

TOPDIR := $(CURDIR)
BUILD_PREFIX ?= ../build/$(TARGET)
BUILD_PREFIX := $(abspath $(BUILD_PREFIX))
BUILD_DIR ?= $(BUILD_PREFIX)/$(notdir $(TOPDIR))
BUILD_DIR := $(abspath $(BUILD_DIR))

# Compliation options for various architectures
TARGET_FAMILY = $(firstword $(subst -, ,$(TARGET)-))
include Makefile.$(TARGET_FAMILY)

export TOPDIR BUILD_PREFIX BUILD_DIR SIL TARGET CC CFLAGS AR ARFLAGS LD LDFLAGS LDLIBS OBJDUMP STRIP

# allow taking subdirs as targets
ifneq ($(filter $(SUBDIRS),$(MAKECMDGOALS)),)
SUBDIRS := $(filter $(SUBDIRS),$(MAKECMDGOALS))
MAKECMDGOALS := $(filter-out $(SUBDIRS),$(MAKECMDGOALS))
else
CLEAN_ALL:=yes
endif

all: $(SUBDIRS)

$(SUBDIRS): .FORCE
	@echo "\033[1;32mCOMPILE $@\033[0m";
	@$(MAKE) -C "$@" $(MAKECMDGOALS)

.FORCE:

clean:
ifeq ($(CLEAN_ALL),yes)
	@rm -rf $(BUILD_DIR)
endif

.PHONY: clean
