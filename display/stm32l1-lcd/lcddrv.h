/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * Tiramisu LCD driver implementation
 *
 * Copyright 2017 Phoenix Systems
 * Author: Adrian Kepka
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */
#ifndef _LCDDRV_H_
#define _LCDDRV_H_

#define LCDDRV_CHAR_DEGREE 0xf8

#define LCDSYM_SMALL_DOT0  (1 <<  0)
#define LCDSYM_SMALL_DOT1  (1 <<  1)
#define LCDSYM_SMALL_DOT2  (1 <<  2)
#define LCDSYM_SMALL_DOT3  (1 <<  3)
#define LCDSYM_SMALL_DOT4  (1 <<  4)
#define LCDSYM_PLUS_Q      (1 <<  5)
#define LCDSYM_PLUS_P      (1 <<  6)
#define LCDSYM_MINUS_Q     (1 <<  7)
#define LCDSYM_MINUS_P     (1 <<  8)
#define LCDSYM_DOT0        (1 <<  9)
#define LCDSYM_DOT1        (1 << 10)
#define LCDSYM_TRIANGLE8   (1 << 11)
#define LCDSYM_DOT3        (1 << 12)
#define LCDSYM_TRIANGLE9   (1 << 13)
#define LCDSYM_DOT2_CENTER (1 << 14)
#define LCDSYM_DOT2        (1 << 15)
#define LCDSYM_LETTER_A    (1 << 16)
#define LCDSYM_LETTER_B    (1 << 17)
#define LCDSYM_LETTER_C    (1 << 18)
#define LCDSYM_LETTER_D    (1 << 19)
#define LCDSYM_SWITCH      (1 << 20)
#define LCDSYM_Hz          (1 << 21)
#define LCDSYM_TRIANGLE2   (1 << 22)
#define LCDSYM_TRIANGLE3   (1 << 23)
#define LCDSYM_TRIANGLE4   (1 << 24)
#define LCDSYM_TRIANGLE5   (1 << 25)
#define LCDSYM_TRIANGLE6   (1 << 26)
#define LCDSYM_TRIANGLE7   (1 << 27)
#define LCDSYM_TRIANGLE0   (1 << 28)
#define LCDSYM_TRIANGLE1   (1 << 29)
#define LCDSYM_ARROWS      (1 << 30)
#define LCDSYM_BAT         (1 << 31)
#define LCDSYM_L1     ((u64)1 << 32)
#define LCDSYM_L2     ((u64)1 << 33)
#define LCDSYM_L3     ((u64)1 << 34)
#define LCDSYM_N      ((u64)1 << 35)
#define LCDSYM_k      ((u64)1 << 36)
#define LCDSYM_M      ((u64)1 << 37)
#define LCDSYM_r_TO_h ((u64)1 << 38)
#define LCDSYM_OO     ((u64)1 << 39)
#define LCDSYM_A      ((u64)1 << 40)
#define LCDSYM_V      ((u64)1 << 41)
#define LCDSYM_V_TO_W ((u64)1 << 42)
#define LCDSYM_Err    ((u64)1 << 43)
#define LCDSYM_h      ((u64)1 << 44)
#define LCDSYM_r      ((u64)1 << 45)

#define LCDSYM_ALL    ((u64)0x3FFFFFFFFFFF)
#define LCDSYM_TOTAL  (14 + 4 * 8)

typedef struct _lcddrv_msg_t {
	enum { LCD_GET, LCD_SET } type;

	char str[8], str_small[6];

	u64 sym_state;
	u64 sym_mask;

	char backlight, on;
} __attribute__((packed)) lcddrv_msg_t;


#endif
