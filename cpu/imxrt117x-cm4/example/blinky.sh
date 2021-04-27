#!/bin/bash

set -e

cd ../lib
make clean all
cd ../example
make clean all

pushd ../../../../_build/armv7m4-imxrt117x/prog.stripped/

xxd -ps -c1 blinky.bin | awk '{printf("0x%s,",$0);if (NR % 16 == 0) printf("\n")}' > blinky.h

popd
