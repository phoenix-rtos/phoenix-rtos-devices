#
# Makefile for Phoenix-RTOS stm32-multi
#
# Copyright 2018 Phoenix Systems
#

SRCS = adc.c gpio.c i2c.c lcd.c rcc.c rtc.c stm32-multi.c flash.c uart.c
OBJS = $(SRCS:.c=.o)
BIN = stm32-multi


all: check $(BIN)


check:
	@(if test -z "$(TARGET)"; then\
		echo "Error: Only main Makefile can be used for compilation!";\
		exit 1;\
	fi;)


.c.o:
	@(printf " CC    %-24s  \n" "$<"; $(CC) -c $(CFLAGS) $<)

-include .depend


$(BIN): $(OBJS)
	@echo " LD    $@"
	$(SIL)$(LD) $(LDFLAGS) -o $(BIN) $(OBJS) $(LDLIBS)
	@cp $(BIN) $(BIN)-unstripped
	@echo " STRIP $@"
	$(SIL)$(STRIP) $(BIN) 2>/dev/null


depend: check
	$(SIL)$(MKDEP) $(MKDEPFLAGS) $(SRCS) >.depend


clean: check
	$(SIL)rm -f core *.o $(ARCH)


.PHONY: clean
# DO NOT DELETE
