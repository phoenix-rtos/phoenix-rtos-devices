/*
 * Phoenix-RTOS
 *
 * STM32L4 ADC driver
 *
 * Copyright 2020 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef ADC_H_
#define ADC_H_


unsigned short adc_conversion(int adc, char chan);


int adc_init(void);


#endif
