/*
 * Phoenix-RTOS
 *
 * IMX6ULL periphery registers for PWM, GPIO and ECSPI
 *
 * NOTE: This file was generated from datasheet using internal tools. See
 * (non-public) devtools repo for details. To be determined what is the
 * correct way to upstream those tools and source data into public repo.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jan Wiśniewski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


/* clang-format off */


#include <stdint.h>

typedef const uint32_t reg32_ro_t;
typedef uint32_t reg32_rw_t;
typedef uint32_t reg32_wo_t;
typedef uint32_t reg32_w1c_t;

/* PWM */
struct imx6ull_regs_pwm {
    reg32_rw_t PWMCR;  /* 0x0000 - Control */
    reg32_w1c_t PWMSR;  /* 0x0004 - Status */
    reg32_rw_t PWMIR;  /* 0x0008 - Interrupt */
    reg32_rw_t PWMSAR;  /* 0x000C - Sample */
    reg32_rw_t PWMPR;  /* 0x0010 - Period */
    reg32_ro_t PWMCNR;  /* 0x0014 - Counter */
};
_Static_assert(offsetof(struct imx6ull_regs_pwm, PWMCNR) == 0x0014, "wrong offset");

#define IMX_PWM3_BASE  0x02088000U


/* GPIO */
struct imx6ull_regs_gpio {
    reg32_rw_t DR;  /* 0x0000 - data register */
    reg32_rw_t GDIR;  /* 0x0004 - direction register */
    reg32_ro_t PSR;  /* 0x0008 - pad status register */
    reg32_rw_t ICR1;  /* 0x000C - interrupt configuration register1 */
    reg32_rw_t ICR2;  /* 0x0010 - interrupt configuration register2 */
    reg32_rw_t IMR;  /* 0x0014 - interrupt mask register */
    reg32_w1c_t ISR;  /* 0x0018 - interrupt status register */
    reg32_rw_t EDGE_SEL;  /* 0x001C - edge select register */
};
_Static_assert(offsetof(struct imx6ull_regs_gpio, EDGE_SEL) == 0x001C, "wrong offset");


#define IMX_GPIO1_BASE  0x0209C000U
#define IMX_GPIO2_BASE  0x020A0000U
#define IMX_GPIO3_BASE  0x020A4000U
#define IMX_GPIO4_BASE  0x020A8000U
#define IMX_GPIO5_BASE  0x020AC000U


/* ECSPI */
struct imx6ull_regs_ecspi {
    reg32_ro_t RXDATA;  /* 0x0000 - Receive Data */
    reg32_wo_t TXDATA;  /* 0x0004 - Transmit Data */
    reg32_rw_t CONREG;  /* 0x0008 - Control */
    reg32_rw_t CONFIGREG;  /* 0x000C - Config */
    reg32_rw_t INTREG;  /* 0x0010 - Interrupt Control */
    reg32_rw_t DMAREG;  /* 0x0014 - DMA Control */
    reg32_rw_t STATREG;  /* 0x0018 - Status */
    reg32_rw_t PERIODREG;  /* 0x001C - Sample Period Control */
    reg32_rw_t TESTREG;  /* 0x0020 - Test Control */
    reg32_ro_t _reserved0[7];  /* 0x0024 */
    reg32_wo_t MSGDATA;  /* 0x0040 - Message Data */
};
_Static_assert(offsetof(struct imx6ull_regs_ecspi, MSGDATA) == 0x0040, "wrong offset");

#define IMX_ECSPI1_BASE  0x02008000U
#define IMX_ECSPI1  ((volatile struct imx6ull_regs_ecspi *)IMX_ECSPI1_BASE)

/* clang-format: on */
