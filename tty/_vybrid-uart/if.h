/* 
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * Vybrid-UART serial port driver
 *
 * Copyright 2014 Phoenix Systems
 *
 * Author: Horacio Mijail Anton Quiles
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _VYBRID_UART_IF_H_
#define _VYBRID_UART_IF_H_

#define BPS_300	    300
#define BPS_600	    600
#define BPS_9600	9600
#define BPS_19200   19200
#define BPS_28800   28800
#define BPS_38400   38400
#define BPS_57600   57600
#define BPS_115200  115200
#define BPS_460800  460800


extern int _serial_init(unsigned int speed);


#endif
