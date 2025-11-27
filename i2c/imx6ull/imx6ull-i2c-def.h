/*
 * Phoenix-RTOS
 *
 * i.MX6ULL I2C driver
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

#ifndef _IMX6ULL_I2C_DEF_H_
#define _IMX6ULL_I2C_DEF_H_

#ifndef CONFIG_I2C1_SCL_MUX_PAD
#define CONFIG_I2C1_SCL_MUX_PAD mux_gpio1_02
#endif
#ifndef CONFIG_I2C1_SCL_MUX_VAL
#define CONFIG_I2C1_SCL_MUX_VAL 0
#endif
#ifndef CONFIG_I2C1_SDA_MUX_PAD
#define CONFIG_I2C1_SDA_MUX_PAD mux_gpio1_03
#endif
#ifndef CONFIG_I2C1_SDA_MUX_VAL
#define CONFIG_I2C1_SDA_MUX_VAL 0
#endif

#ifndef CONFIG_I2C2_SCL_MUX_PAD
#define CONFIG_I2C2_SCL_MUX_PAD mux_gpio1_00
#endif
#ifndef CONFIG_I2C2_SCL_MUX_VAL
#define CONFIG_I2C2_SCL_MUX_VAL 0
#endif
#ifndef CONFIG_I2C2_SDA_MUX_PAD
#define CONFIG_I2C2_SDA_MUX_PAD mux_gpio1_01
#endif
#ifndef CONFIG_I2C2_SDA_MUX_VAL
#define CONFIG_I2C2_SDA_MUX_VAL 0
#endif

#ifndef CONFIG_I2C3_SCL_MUX_PAD
#define CONFIG_I2C3_SCL_MUX_PAD mux_lcd_d1
#endif
#ifndef CONFIG_I2C3_SCL_MUX_VAL
#define CONFIG_I2C3_SCL_MUX_VAL 4
#endif
#ifndef CONFIG_I2C3_SDA_MUX_PAD
#define CONFIG_I2C3_SDA_MUX_PAD mux_lcd_d0
#endif
#ifndef CONFIG_I2C3_SDA_MUX_VAL
#define CONFIG_I2C3_SDA_MUX_VAL 4
#endif

#ifndef CONFIG_I2C4_SCL_MUX_PAD
#define CONFIG_I2C4_SCL_MUX_PAD mux_lcd_d3
#endif
#ifndef CONFIG_I2C4_SCL_MUX_VAL
#define CONFIG_I2C4_SCL_MUX_VAL 4
#endif
#ifndef CONFIG_I2C4_SDA_MUX_PAD
#define CONFIG_I2C4_SDA_MUX_PAD mux_lcd_d2
#endif
#ifndef CONFIG_I2C4_SDA_MUX_VAL
#define CONFIG_I2C4_SDA_MUX_VAL 4
#endif

#ifndef CONFIG_I2C1_SCL_ISEL
#define CONFIG_I2C1_SCL_ISEL 0
#endif
#ifndef CONFIG_I2C1_SDA_ISEL
#define CONFIG_I2C1_SDA_ISEL 1
#endif
#ifndef CONFIG_I2C2_SCL_ISEL
#define CONFIG_I2C2_SCL_ISEL 1
#endif
#ifndef CONFIG_I2C2_SDA_ISEL
#define CONFIG_I2C2_SDA_ISEL 1
#endif
#ifndef CONFIG_I2C3_SCL_ISEL
#define CONFIG_I2C3_SCL_ISEL 2
#endif
#ifndef CONFIG_I2C3_SDA_ISEL
#define CONFIG_I2C3_SDA_ISEL 2
#endif
#ifndef CONFIG_I2C4_SCL_ISEL
#define CONFIG_I2C4_SCL_ISEL 2
#endif
#ifndef CONFIG_I2C4_SDA_ISEL
#define CONFIG_I2C4_SDA_ISEL 2
#endif

#endif /* _IMX6ULL_I2C_DEF_H_ */
