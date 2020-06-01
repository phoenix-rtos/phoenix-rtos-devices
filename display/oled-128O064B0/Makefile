#
# Makefile for BeSmart oled driver
#
# Copyright 2020 Phoenix Systems
# Author: Hubert Buczynski, Marcin Baran
#

OLED_CORE = oled-phy.o
OLED_GRAPHIC = oled-graphic.o fonts/font.o fonts/5x7.o

$(PREFIX_PROG)oled-test: $(addprefix $(PREFIX_O)display/oled-128O064B0/, oled-test.o $(OLED_CORE))
	$(LINK)

$(PREFIX_PROG)oled-driver: $(addprefix $(PREFIX_O)display/oled-128O064B0/, oled-dev.o $(OLED_GRAPHIC) $(OLED_CORE))
	$(LINK)

$(PREFIX_H)oled-api.h: display/oled-128O064B0/oled-api.h
	$(HEADER)

all: $(addprefix $(PREFIX_PROG_STRIPPED), oled-test oled-driver) $(PREFIX_H)oled-api.h
