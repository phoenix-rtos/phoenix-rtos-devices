#
# Makefile for Phoenix-RTOS virtio-blk driver
#
# Copyright 2020 Phoenix Systems
#

$(PREFIX_PROG)virtio-blk: $(addprefix $(PREFIX_O)storage/virtio-blk/, virtio.o virtio-mmio.o virtio-blk.o) $(PREFIX_A)libext2.a
	$(LINK)

all: $(PREFIX_PROG_STRIPPED)virtio-blk
