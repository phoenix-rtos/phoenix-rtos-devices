#
# Phoenix-RTOS
#
#  i.MX RT storage tests
#
# Copyright 2020 Phoenix Systems
# Author: Hubert Buczynski
#

$(PREFIX_PROG)flash-tests: $(addprefix $(PREFIX_O)storage/imxrt-flash/tests/, tests.o flashsrv_mfs_tests.o flashsrv_raw_tests.o flashdrv_tests.o) $(PREFIX_A)libflashdrv.a $(PREFIX_A)libptable.a $(PREFIX_A)libmeterfs.a
	$(LINK)

$(PREFIX_O)storage/ imxrt-flash/tests/flash-tests.o: $(PREFIX_H)libflashdrv.h

all: $(PREFIX_PROG_STRIPPED)flash-tests

