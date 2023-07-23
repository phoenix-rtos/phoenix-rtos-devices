#!/bin/bash

set -e

export TARGET=armv7m4-imxrt117x
export PREFIX_BUILD=../../../../../_build/${TARGET}

make -C ../lib -f Makefile.ext clean all
make -C ../example -f Makefile.ext clean all

pushd ${PREFIX_BUILD}/prog.stripped/

xxd -ps -c1 blinky.bin | awk '{printf("0x%s,",$0);if (NR % 16 == 0) printf("\n")}' > blinky.hex

popd
