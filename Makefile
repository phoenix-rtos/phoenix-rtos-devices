#
# Makefile for phoenix-rtos-devices
#
# Copyright 2018, 2019 Phoenix Systems
#
# %LICENSE%
#

SIL ?= @
MAKEFLAGS += --no-print-directory

#TARGET ?= ia32-qemu
#TARGET ?= armv7-stm32
TARGET ?= arm-imx6ull

TOPDIR := $(CURDIR)
PREFIX_BUILD ?= ../_build/$(TARGET)
PREFIX_BUILD := $(abspath $(PREFIX_BUILD))
BUILD_DIR ?= $(PREFIX_BUILD)/$(notdir $(TOPDIR))
BUILD_DIR := $(abspath $(BUILD_DIR))

# Compliation options for various architectures
TARGET_FAMILY = $(firstword $(subst -, ,$(TARGET)-))
include Makefile.$(TARGET_FAMILY)

# build artifacts dir
CURR_SUFFIX := $(patsubst $(TOPDIR)/%,%,$(abspath $(CURDIR))/)
PREFIX_O := $(BUILD_DIR)/$(CURR_SUFFIX)

# target install paths, can be provided exterally
PREFIX_A ?= $(PREFIX_BUILD)/lib/
PREFIX_H ?= $(PREFIX_BUILD)/include/
PREFIX_PROG ?= $(PREFIX_BUILD)/prog/
PREFIX_PROG_STRIPPED ?= $(PREFIX_BUILD)/prog.stripped/

CFLAGS += -I"$(PREFIX_H)"
LDFLAGS += -L"$(PREFIX_A)"

# add include path for auto-generated files
CFLAGS += -I"$(BUILD_DIR)/$(CURR_SUFFIX)"

ARCH =  $(SIL)@mkdir -p $(@D); \
	(printf "AR  %-24s\n" "$(@F)"); \
	$(AR) $(ARFLAGS) $@ $^ 2>/dev/null

LINK = $(SIL)mkdir -p $(@D); \
	(printf "LD  %-24s\n" "$(@F)"); \
	$(LD) $(LDFLAGS) -o "$@"  $^ $(LDLIBS)
	
HEADER = $(SIL)mkdir -p $(@D); \
	(printf "HEADER %-24s\n" "$<"); \
	cp -pR "$<" "$@"

$(PREFIX_O)%.o: %.c
	@mkdir -p $(@D)
	$(SIL)(printf "CC  %-24s\n" "$<")
	$(SIL)$(CC) -c $(CFLAGS) "$<" -o "$@"
	$(SIL)$(CC) -M  -MD -MP -MF $(PREFIX_O)$*.c.d -MT "$@" $(CFLAGS) $<

$(PREFIX_O)%.o: %.S
	@mkdir -p $(@D)
	$(SIL)(printf "ASM %s/%-24s\n" "$(notdir $(@D))" "$<")
	$(SIL)$(CC) -c $(CFLAGS) "$<" -o "$@"
	$(SIL)$(CC) -M  -MD -MP -MF $(PREFIX_O)$*.S.d -MT "$@" $(CFLAGS) $<
	
$(PREFIX_PROG_STRIPPED)%: $(PREFIX_PROG)%
	@mkdir -p $(@D)
	@(printf "STR %-24s\n" "$(@F)")
	$(SIL)$(STRIP) -o $@ $<

.PHONY: clean
clean:
	@echo "rm -rf $(BUILD_DIR)"

ifneq ($(filter clean,$(MAKECMDGOALS)),)
	$(shell rm -rf $(BUILD_DIR))
endif

T1 := $(filter-out clean all,$(MAKECMDGOALS))
ifneq ($(T1),)
	include $(T1)/Makefile
.PHONY: $(T1)
$(T1):
	@echo >/dev/null
else
	include Makefile.$(TARGET)
endif
