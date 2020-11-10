#
# Makefile for Phoenix-RTOS virtio-blk driver
#
# Copyright 2020 Phoenix Systems
#

$(PREFIX_PROG)virtio-blk: $(addprefix $(PREFIX_O)storage/virtio-blk/, mbr.o virtio.o virtio-mmio.o virtio-blk.o virtio-blksrv.o) $(PREFIX_A)libext2.a
	$(LINK)

$(PREFIX_A)libvirtioblk.a: $(PREFIX_O)storage/virtio-blk/virtio-blk.o
	$(ARCH)

all: $(PREFIX_PROG_STRIPPED)virtio-blk $(PREFIX_A)libvirtioblk.a
