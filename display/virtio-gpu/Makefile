#
# Makefile for Phoenix-RTOS virtio-gpu driver
#
# Copyright 2020 Phoenix Systems
#

$(PREFIX_PROG)virtio-gpu: $(addprefix $(PREFIX_O)display/virtio-gpu/, virtio-gpu.o) $(PREFIX_A)libvirtio.a
	$(LINK)

all: $(addprefix $(PREFIX_PROG_STRIPPED), virtio-gpu)
