/*
 * Phoenix-RTOS
 *
 * i.MX FlexPWM
 *
 * Copyright 2021-2023 Phoenix Systems
 * Author: Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef FLEXPWM_H
#define FLEXPWM_H


/* Flag for use in board config */
#define FLEXPWM_BOARDCONFIG


/* Device selector */
#define FLEXPWM_PWM1 0u
#define FLEXPWM_PWM2 1u
#define FLEXPWM_PWM3 2u
#define FLEXPWM_PWM4 3u


/* Input capture edge setup */
#define FLEXPWM_CAP_DISABLED     0u
#define FLEXPWM_CAP_EDGE_FALLING 1u
#define FLEXPWM_CAP_EDGE_RISING  2u
#define FLEXPWM_CAP_EDGE_ANY     3u


/* Submodule */
#define FLEXPWM_SM0 0u
#define FLEXPWM_SM1 1u
#define FLEXPWM_SM2 2u
#define FLEXPWM_SM3 3u


/* PWM pin */
#define FLEXPWM_PWMX 0u
#define FLEXPWM_PWMB 1u
#define FLEXPWM_PWMA 2u


/* DMA Requests read/write */
#define FLEXPWM_DMA_RREQ(dev, sm) (85u + (unsigned)(dev)*8u + (unsigned)(sm) + 0u)
#define FLEXPWM_DMA_WREQ(dev, sm) (85u + (unsigned)(dev)*8u + (unsigned)(sm) + 4u)


#ifdef __cplusplus
extern "C" {
#endif


/* Initialize base address using dev index */
int flexpwm_init(unsigned int dev);


/* Configure input capture mode */
int flexpwm_input_capture(unsigned int sm, unsigned int pwm, unsigned int cap0_edge, unsigned int cap1_edge);


#ifdef __cplusplus
};
#endif


#endif /* end of FLEXPWM_H */
