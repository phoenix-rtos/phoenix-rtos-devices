/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32L1 helper functions
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _COMMON_H_
#define _COMMON_H_


#include "../../../phoenix-rtos-kernel/include/arch/stm32l1.h"


#ifdef NDEBUG
#define DEBUG(format, ...)
#else
#define DEBUG(format, ...) printf("%s:"format, drvname, __VA_ARGS__)
#endif


extern const char *drvname;


/* STM32L1 interrupts numbers */
enum { wwdq_irq = 16, pvd_irq, tamper_stamp_irq, rtc_wkup_irq, flash_irq, rcc_irq,
	exti0_irq, exti1_irq, exti2_irq, exti3_irq, exti4_irq, dma1ch1_irq, dma1ch2_irq,
	dma1ch3_irq, dma1ch4_irq, dma1ch5_irq, dma1ch6_irq, dma1ch7_irq, adc1_irq,
	usbhp_irq, usblp_irq, dac_irq, comp_irq, exti9_5_irq, lcd_irq, tim9_irq, tim10_irq,
	tim11_irq, tim2_irq, tim3_irq, tim4_irq, i2c1_ev_irq, i2c1_er_irq, i2c2_ev_irq,
	i2c2_er_irq, spi1_irq, spi2_irq, usart1_irq, usart2_irq, usart3_irq, exti15_10_irq,
	rtc_alrm_irq, usb_fs_wkup_irq, tim6_irq, tim7_irq, sdio_irq, tim5_irq, spi3_irq,
	uart4_irq, uart5_irq, dma2ch1_irq, dma2ch2_irq, dma2ch3_irq, dma2ch4_irq, dma2ch5_irq,
	comp_acq_irq = 72 };


static inline dataBarier(void)
{
	__asm__ volatile ("dmb");
}

#endif
