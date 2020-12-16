#
# Makefile for Phoenix-RTOS virtio-blk driver
#
# Copyright 2020 Phoenix Systems
#

$(PREFIX_PROG)virtio-blk: $(addprefix $(PREFIX_O)storage/virtio-blk/, virtio-blk.o) $(PREFIX_A)libvirtio.a
	$(LINK)

all: $(addprefix $(PREFIX_PROG_STRIPPED), virtio-blk)
