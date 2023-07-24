/*
 * Phoenix-RTOS
 *
 * GR716 ADC driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _MULTI_ADC_H_
#define _MULTI_ADC_H_


#include <sys/msg.h>


void adc_handleMsg(msg_t *msg, int dev);


int adc_init(void);


#endif
