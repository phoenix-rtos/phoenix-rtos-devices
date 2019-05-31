/*
 * Phoenix-RTOS
 *
 * iMX RT multi driver config.
 *
 * Copyright 2019 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _CONFIG_H_
#define _CONFIG_H_

/* UART */

#ifndef UART1
#define UART1 1
#endif

#ifndef UART2
#define UART2 0
#endif

#ifndef UART3
#define UART3 0
#endif

#ifndef UART4
#define UART4 0
#endif

#ifndef UART5
#define UART5 0
#endif

#ifndef UART6
#define UART6 0
#endif

#ifndef UART7
#define UART7 0
#endif

#ifndef UART8
#define UART8 0
#endif

#ifndef UART_CONSOLE
#define UART_CONSOLE 1
#endif

#define UART1_TX_PIN ad_b0_12
#define UART1_RX_PIN ad_b0_13
#define UART1_RTS_PIN ad_b0_15
#define UART1_CTS_PIN ad_b0_14

#ifndef UART2_TX_PIN
#define UART2_TX_PIN ad_b1_02
//#define UART2_TX_PIN sd_b1_11
#endif
#ifndef UART2_RX_PIN
#define UART2_RX_PIN ad_b1_03
//#define UART2_RX_PIN sd_b1_10
#endif
#define UART2_RTS_PIN ad_b1_01
#define UART2_CTS_PIN ad_b1_00

#ifndef UART3_TX_PIN
#define UART3_TX_PIN emc_13
//#define UART3_TX_PIN ad_b1_06
//#define UART3_TX_PIN b0_08
#endif
#ifndef UART3_RX_PIN
#define UART3_RX_PIN emc_14
//#define UART3_RX_PIN ad_b1_07
//#define UART3_RX_PIN b0_09
#endif
#ifndef UART3_RTS_PIN
#define UART3_RTS_PIN emc_16
//#define UART3_RTS_PIN ad_b1_05
#endif
#ifndef UART3_CTS_PIN
#define UART3_CTS_PIN emc_15
//#define UART3_CTS_PIN ad_b1_04
#endif

#ifndef UART4_TX_PIN
#define UART4_TX_PIN emc_19
//#define UART4_TX_PIN b1_00
//#define UART4_TX_PIN sd_b1_00
#endif
#ifndef UART4_RX_PIN
#define UART4_RX_PIN emc_20
//#define UART4_RX_PIN b1_01
//#define UART4_RX_PIN sd_b1_01
#endif
#define UART4_RTS_PIN emc_18
#define UART4_CTS_PIN emc_17

#ifndef UART5_TX_PIN
#define UART5_TX_PIN emc_23
//#define UART5_TX_PIN b1_12
#endif
#ifndef UART5_RX_PIN
#define UART5_RX_PIN emc_24
//#define UART5_RX_PIN b1_13
#endif
#define UART5_RTS_PIN emc_27
#define UART5_CTS_PIN emc_28

#ifndef UART6_TX_PIN
#define UART6_TX_PIN emc_25
//#define UART6_TX_PIN ad_b0_12
#endif
#ifndef UART6_RX_PIN
#define UART6_RX_PIN emc_26
//#define UART6_RX_PIN ad_b0_03
#endif
#define UART6_RTS_PIN emc_29
#define UART6_CTS_PIN emc_30

#ifndef UART7_TX_PIN
#define UART7_TX_PIN emc_31
//#define UART7_TX_PIN sd_b1_08
#endif
#ifndef UART7_RX_PIN
#define UART7_RX_PIN emc_32
//#define UART7_RX_PIN sd_b1_09
#endif
#define UART7_RTS_PIN sd_b1_07
#define UART7_CTS_PIN sd_b1_06

#ifndef UART8_TX_PIN
#define UART8_TX_PIN emc_38
//#define UART8_TX_PIN ad_b1_10
//#define UART8_TX_PIN sd_b0_04
#endif
#ifndef UART8_RX_PIN
#define UART8_RX_PIN emc_39
//#define UART8_RX_PIN ad_b1_11
//#define UART8_RX_PIN sd_b0_05
#endif
#define UART8_RTS_PIN sd_b0_03
#define UART8_CTS_PIN sd_b0_02

#endif
