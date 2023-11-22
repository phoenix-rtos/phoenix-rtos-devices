/*
 * Phoenix-RTOS
 *
 * Phoenix-RTOS klog driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Maciej Purski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */
#ifndef _LIBKLOG_H_
#define _LIBKLOG_H_


#include <stddef.h>
#include <stdint.h>
#include <sys/msg.h>


typedef void (*libklog_write_t)(const char *buf, size_t size);


int libklog_ctrlHandle(uint32_t port, msg_t *msg, msg_rid_t rid);


int libklog_ctrlRegister(oid_t *oid);


int libklog_init(libklog_write_t clbk);


#endif /* _LIBKLOG_H_ */
