/*
 * Phoenix-RTOS
 *
 * STM32L4/N6/U3 ADC driver
 *
 * Copyright 2020 Phoenix Systems
 * Copyright 2026 Apator Metrix
 * Author: Aleksander Kaminski, Mateusz Karcz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef ADC_H_
#define ADC_H_


int adc_conversion(int adc, char chan, unsigned int *out);


int adc_init(void);


#endif
