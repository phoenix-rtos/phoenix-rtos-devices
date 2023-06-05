/*
 * Phoenix-RTOS
 *
 * i.MX RT117x M4 cpu core driver
 *
 * Copyright 2021, 2023 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * %LICENSE%
 */

#ifndef IMXRT117X_CM4_H
#define IMXRT117X_CM4_H


#include <sys/msg.h>


void cm4_handleMsg(msg_t *msg);


void cm4_init(void);


#endif
