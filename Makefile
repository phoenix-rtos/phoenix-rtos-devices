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

VERSION = 0.2
TOPDIR := $(CURDIR)
BUILD_DIR ?= build/$(TARGET)
BUILD_DIR := $(abspath $(BUILD_DIR))

# Compliation options for various architectures
TARGET_FAMILY = $(firstword $(subst -, ,$(TARGET)-))
include Makefile.$(TARGET_FAMILY)

export TOPDIR BUILD_DIR SIL TARGET CC CFLAGS MKDEP MKDEPFLAGS AR ARFLAGS LD LDFLAGS LDLIBS OBJDUMP STRIP

# allow taking subdirs as targets
ifneq ($(filter $(SUBDIRS),$(MAKECMDGOALS)),)
SUBDIRS := $(filter $(SUBDIRS),$(MAKECMDGOALS))
MAKECMDGOALS := $(filter-out $(SUBDIRS),$(MAKECMDGOALS))
endif

all: $(SUBDIRS)

$(SUBDIRS): .FORCE
	@echo "\033[1;32mCOMPILE $@\033[0m";
	@$(MAKE) -C "$@" $(MAKECMDGOALS)

.FORCE:

clean:
	@rm -rf $(BUILD_DIR)


.PHONY: clean
