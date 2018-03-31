# phoenix-rtos-devices
This repository contains the source for the Phoenix-RTOS device drivers. Device drivers work on the user-level and are written
using libphoenix standard library. Most of drivers are imported into this repository directly from Phoenix-RTOS 2 kernel and need to be reimplemnted.
These drivers are marked with _ prefix.

To compile driver enter into the driver directory and type:

	$ make clean
	$ make

This work is licensed under a BSD license. See the LICENSE file for details.
