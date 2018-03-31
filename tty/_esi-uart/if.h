/* 
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * eSi-UART serial port driver
 *
 * Copyright 2012 Phoenix Systems
 *
 * Author: Jacek Popko
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _ESI_UART_IF_H_
#define _ESI_UART_IF_H_

#define BPS_9600	9600
#define BPS_19200   19200
#define BPS_28800   28800
#define BPS_38400   38400
#define BPS_57600   57600
#define BPS_115200  115200

#define SIZE_SERIALS       3
#define SIZE_SERIAL_CHUNK  256


extern int _serial_init(unsigned int speed);


#endif
