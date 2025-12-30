/*
 * Phoenix-RTOS
 *
 * i.MX6ULL UART driver
 *
 * Default pin definitions
 *
 * Copyright 2025 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _IMX6ULL_UART_DEF_H_
#define _IMX6ULL_UART_DEF_H_

#ifndef CONFIG_UART1_CTS_MUX_PAD
#define CONFIG_UART1_CTS_MUX_PAD mux_uart1_cts
#endif

#ifndef CONFIG_UART1_CTS_MUX_VAL
#define CONFIG_UART1_CTS_MUX_VAL 0
#endif

#ifndef CONFIG_UART1_RTS_MUX_PAD
#define CONFIG_UART1_RTS_MUX_PAD mux_uart1_rts
#endif

#ifndef CONFIG_UART1_RTS_MUX_VAL
#define CONFIG_UART1_RTS_MUX_VAL 0
#endif

#ifndef CONFIG_UART1_RX_MUX_PAD
#define CONFIG_UART1_RX_MUX_PAD mux_uart1_rx
#endif

#ifndef CONFIG_UART1_RX_MUX_VAL
#define CONFIG_UART1_RX_MUX_VAL 0
#endif

#ifndef CONFIG_UART1_TX_MUX_PAD
#define CONFIG_UART1_TX_MUX_PAD mux_uart1_tx
#endif

#ifndef CONFIG_UART1_TX_MUX_VAL
#define CONFIG_UART1_TX_MUX_VAL 0
#endif

#ifndef CONFIG_UART2_CTS_MUX_PAD
#define CONFIG_UART2_CTS_MUX_PAD mux_uart2_cts
#endif

#ifndef CONFIG_UART2_CTS_MUX_VAL
#define CONFIG_UART2_CTS_MUX_VAL 0
#endif

#ifndef CONFIG_UART2_RTS_MUX_PAD
#define CONFIG_UART2_RTS_MUX_PAD mux_uart2_rts
#endif

#ifndef CONFIG_UART2_RTS_MUX_VAL
#define CONFIG_UART2_RTS_MUX_VAL 0
#endif

#ifndef CONFIG_UART2_RX_MUX_PAD
#define CONFIG_UART2_RX_MUX_PAD mux_uart2_rx
#endif

#ifndef CONFIG_UART2_RX_MUX_VAL
#define CONFIG_UART2_RX_MUX_VAL 0
#endif

#ifndef CONFIG_UART2_TX_MUX_PAD
#define CONFIG_UART2_TX_MUX_PAD mux_uart2_tx
#endif

#ifndef CONFIG_UART2_TX_MUX_VAL
#define CONFIG_UART2_TX_MUX_VAL 0
#endif

#ifndef CONFIG_UART3_CTS_MUX_PAD
#define CONFIG_UART3_CTS_MUX_PAD mux_uart3_cts
#endif

#ifndef CONFIG_UART3_CTS_MUX_VAL
#define CONFIG_UART3_CTS_MUX_VAL 0
#endif

#ifndef CONFIG_UART3_RTS_MUX_PAD
#define CONFIG_UART3_RTS_MUX_PAD mux_uart3_rts
#endif

#ifndef CONFIG_UART3_RTS_MUX_VAL
#define CONFIG_UART3_RTS_MUX_VAL 0
#endif

#ifndef CONFIG_UART3_RX_MUX_PAD
#define CONFIG_UART3_RX_MUX_PAD mux_uart3_rx
#endif

#ifndef CONFIG_UART3_RX_MUX_VAL
#define CONFIG_UART3_RX_MUX_VAL 0
#endif

#ifndef CONFIG_UART3_TX_MUX_PAD
#define CONFIG_UART3_TX_MUX_PAD mux_uart3_tx
#endif

#ifndef CONFIG_UART3_TX_MUX_VAL
#define CONFIG_UART3_TX_MUX_VAL 0
#endif

#ifndef CONFIG_UART4_CTS_MUX_PAD
#define CONFIG_UART4_CTS_MUX_PAD mux_lcd_hsync
#endif

#ifndef CONFIG_UART4_CTS_MUX_VAL
#define CONFIG_UART4_CTS_MUX_VAL 2
#endif

#ifndef CONFIG_UART4_RTS_MUX_PAD
#define CONFIG_UART4_RTS_MUX_PAD mux_lcd_vsync
#endif

#ifndef CONFIG_UART4_RTS_MUX_VAL
#define CONFIG_UART4_RTS_MUX_VAL 2
#endif

#ifndef CONFIG_UART4_RX_MUX_PAD
#define CONFIG_UART4_RX_MUX_PAD mux_uart4_rx
#endif

#ifndef CONFIG_UART4_RX_MUX_VAL
#define CONFIG_UART4_RX_MUX_VAL 0
#endif

#ifndef CONFIG_UART4_TX_MUX_PAD
#define CONFIG_UART4_TX_MUX_PAD mux_uart4_tx
#endif

#ifndef CONFIG_UART4_TX_MUX_VAL
#define CONFIG_UART4_TX_MUX_VAL 0
#endif

#ifndef CONFIG_UART5_CTS_MUX_PAD
#define CONFIG_UART5_CTS_MUX_PAD mux_gpio1_09
#endif

#ifndef CONFIG_UART5_CTS_MUX_VAL
#define CONFIG_UART5_CTS_MUX_VAL 8
#endif

#ifndef CONFIG_UART5_RTS_MUX_PAD
#define CONFIG_UART5_RTS_MUX_PAD mux_gpio1_08
#endif

#ifndef CONFIG_UART5_RTS_MUX_VAL
#define CONFIG_UART5_RTS_MUX_VAL 8
#endif

#ifndef CONFIG_UART5_RX_MUX_PAD
#define CONFIG_UART5_RX_MUX_PAD mux_uart5_rx
#endif

#ifndef CONFIG_UART5_RX_MUX_VAL
#define CONFIG_UART5_RX_MUX_VAL 0
#endif

#ifndef CONFIG_UART5_TX_MUX_PAD
#define CONFIG_UART5_TX_MUX_PAD mux_uart5_tx
#endif

#ifndef CONFIG_UART5_TX_MUX_VAL
#define CONFIG_UART5_TX_MUX_VAL 0
#endif

#ifndef CONFIG_UART6_CTS_MUX_PAD
#define CONFIG_UART6_CTS_MUX_PAD mux_enet1_tx1
#endif

#ifndef CONFIG_UART6_CTS_MUX_VAL
#define CONFIG_UART6_CTS_MUX_VAL 1
#endif

#ifndef CONFIG_UART6_RTS_MUX_PAD
#define CONFIG_UART6_RTS_MUX_PAD mux_enet1_txen
#endif

#ifndef CONFIG_UART6_RTS_MUX_VAL
#define CONFIG_UART6_RTS_MUX_VAL 1
#endif

#ifndef CONFIG_UART6_RX_MUX_PAD
#define CONFIG_UART6_RX_MUX_PAD mux_enet2_rx1
#endif

#ifndef CONFIG_UART6_RX_MUX_VAL
#define CONFIG_UART6_RX_MUX_VAL 1
#endif

#ifndef CONFIG_UART6_TX_MUX_PAD
#define CONFIG_UART6_TX_MUX_PAD mux_enet2_rx0
#endif

#ifndef CONFIG_UART6_TX_MUX_VAL
#define CONFIG_UART6_TX_MUX_VAL 1
#endif

#ifndef CONFIG_UART7_CTS_MUX_PAD
#define CONFIG_UART7_CTS_MUX_PAD mux_lcd_d6
#endif

#ifndef CONFIG_UART7_CTS_MUX_VAL
#define CONFIG_UART7_CTS_MUX_VAL 1
#endif

#ifndef CONFIG_UART7_RTS_MUX_PAD
#define CONFIG_UART7_RTS_MUX_PAD mux_lcd_d7
#endif

#ifndef CONFIG_UART7_RTS_MUX_VAL
#define CONFIG_UART7_RTS_MUX_VAL 1
#endif

#ifndef CONFIG_UART7_RX_MUX_PAD
#define CONFIG_UART7_RX_MUX_PAD mux_lcd_d17
#endif

#ifndef CONFIG_UART7_RX_MUX_VAL
#define CONFIG_UART7_RX_MUX_VAL 1
#endif

#ifndef CONFIG_UART7_TX_MUX_PAD
#define CONFIG_UART7_TX_MUX_PAD mux_lcd_d16
#endif

#ifndef CONFIG_UART7_TX_MUX_VAL
#define CONFIG_UART7_TX_MUX_VAL 1
#endif

#ifndef CONFIG_UART8_CTS_MUX_PAD
#define CONFIG_UART8_CTS_MUX_PAD mux_lcd_d4
#endif

#ifndef CONFIG_UART8_CTS_MUX_VAL
#define CONFIG_UART8_CTS_MUX_VAL 1
#endif

#ifndef CONFIG_UART8_RTS_MUX_PAD
#define CONFIG_UART8_RTS_MUX_PAD mux_lcd_d5
#endif

#ifndef CONFIG_UART8_RTS_MUX_VAL
#define CONFIG_UART8_RTS_MUX_VAL 1
#endif

#ifndef CONFIG_UART8_RX_MUX_PAD
#define CONFIG_UART8_RX_MUX_PAD mux_lcd_d21
#endif

#ifndef CONFIG_UART8_RX_MUX_VAL
#define CONFIG_UART8_RX_MUX_VAL 1
#endif

#ifndef CONFIG_UART8_TX_MUX_PAD
#define CONFIG_UART8_TX_MUX_PAD mux_lcd_d20
#endif

#ifndef CONFIG_UART8_TX_MUX_VAL
#define CONFIG_UART8_TX_MUX_VAL 1
#endif


#ifndef CONFIG_UART1_RTS_ISEL
#define CONFIG_UART1_RTS_ISEL 3
#endif

#ifndef CONFIG_UART1_RX_ISEL
#define CONFIG_UART1_RX_ISEL 3
#endif

#ifndef CONFIG_UART2_RTS_ISEL
#define CONFIG_UART2_RTS_ISEL 1
#endif

#ifndef CONFIG_UART2_RX_ISEL
#define CONFIG_UART2_RX_ISEL 1
#endif

#ifndef CONFIG_UART3_RTS_ISEL
#define CONFIG_UART3_RTS_ISEL 1
#endif

#ifndef CONFIG_UART3_RX_ISEL
#define CONFIG_UART3_RX_ISEL 1
#endif

#ifndef CONFIG_UART4_RTS_ISEL
#define CONFIG_UART4_RTS_ISEL 3
#endif

#ifndef CONFIG_UART4_RX_ISEL
#define CONFIG_UART4_RX_ISEL 1
#endif

#ifndef CONFIG_UART5_RTS_ISEL
#define CONFIG_UART5_RTS_ISEL 1
#endif

#ifndef CONFIG_UART5_RX_ISEL
#define CONFIG_UART5_RX_ISEL 7
#endif

#ifndef CONFIG_UART6_RTS_ISEL
#define CONFIG_UART6_RTS_ISEL 3
#endif

#ifndef CONFIG_UART6_RX_ISEL
#define CONFIG_UART6_RX_ISEL 2
#endif

#ifndef CONFIG_UART7_RTS_ISEL
#define CONFIG_UART7_RTS_ISEL 3
#endif

#ifndef CONFIG_UART7_RX_ISEL
#define CONFIG_UART7_RX_ISEL 3
#endif

#ifndef CONFIG_UART8_RTS_ISEL
#define CONFIG_UART8_RTS_ISEL 3
#endif

#ifndef CONFIG_UART8_RX_ISEL
#define CONFIG_UART8_RX_ISEL 3
#endif

#endif /* _IMX6ULL_UART_DEF_H_ */
