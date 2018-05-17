/*
 * Phoenix-RTOS
 *
 * ttypc - keyboard handler (map derived from BSD 4.4 Lite kernel).
 *
 * Copyright 2012, 2017 Phoenix Systems
 * Copyright 2001, 2007-2008 Pawel Pisarczyk
 * Author: Pawel Pisarczyk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/threads.h>
#include <sys/interrupt.h>

#include "ttypc.h"
#include "ttypc_kbd.h"
#include "ttypc_vga.h"


/* U.S 101 keys keyboard map */
keymap_t scan_codes[] = {
	/*  type     unshift   shift     control   altgr     shift_altgr scancode */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 0 unused */
	{ KB_ASCII,  "\033",   "\033",   "\033",   "",       "" }, /* 1 ESCape */
	{ KB_ASCII,  "1",      "!",      "\033[k", "",       "" }, /* 2 1 */
	{ KB_ASCII,  "2",      "@",      "\033[l", "",       "" }, /* 3 2 */
	{ KB_ASCII,  "3",      "#",      "\033[m", "",       "" }, /* 4 3 */
	{ KB_ASCII,  "4",      "$",      "\033[n", "",       "" }, /* 5 4 */
	{ KB_ASCII,  "5",      "%",      "%",      "",       "" }, /* 6 5 */
	{ KB_ASCII,  "6",      "^",      "\036",   "",       "" }, /* 7 6 */
	{ KB_ASCII,  "7",      "&",      "&",      "",       "" }, /* 8 7 */
	{ KB_ASCII,  "8",      "*",      "\010",   "",       "" }, /* 9 8 */
	{ KB_ASCII,  "9",      "(",      "(",      "",       "" }, /* 10 9 */
	{ KB_ASCII,  "0",      ")",      ")",      "",       "" }, /* 11 0 */
	{ KB_ASCII,  "-",      "_",      "\037",   "",       "" }, /* 12 - */
	{ KB_ASCII,  "=",      "+",      "+",      "",       "" }, /* 13 = */
	{ KB_ASCII,  "\b \b",  "\177",   "\010",   "",       "" }, /* 14 backspace */
	{ KB_ASCII,  "\t",     "\t",     "\t",     "",       "" }, /* 15 tab */
	{ KB_ASCII,  "q",      "Q",      "\021",   "",       "" }, /* 16 q */
	{ KB_ASCII,  "w",      "W",      "\027",   "",       "" }, /* 17 w */
	{ KB_ASCII,  "e",      "E",      "\005",   "",       "" }, /* 18 e */
	{ KB_ASCII,  "r",      "R",      "\022",   "",       "" }, /* 19 r */
	{ KB_ASCII,  "t",      "T",      "\024",   "",       "" }, /* 20 t */
	{ KB_ASCII,  "y",      "Y",      "\031",   "",       "" }, /* 21 y */
	{ KB_ASCII,  "u",      "U",      "\025",   "",       "" }, /* 22 u */
	{ KB_ASCII,  "i",      "I",      "\011",   "",       "" }, /* 23 i */
	{ KB_ASCII,  "o",      "O",      "\017",   "",       "" }, /* 24 o */
	{ KB_ASCII,  "p",      "P",      "\020",   "",       "" }, /* 25 p */
	{ KB_ASCII,  "[",      "{",      "\033",   "",       "" }, /* 26 [ */
	{ KB_ASCII,  "]",      "}",      "\035",   "",       "" }, /* 27 ] */
	{ KB_ASCII,  "\n",     "\n",     "\r",     "",       "" }, /* 28 return */
	{ KB_CTL,    "",       "",       "",       "",       "" }, /* 29 control */
	{ KB_ASCII,  "a",      "A",      "\001",   "",       "" }, /* 30 a */
	{ KB_ASCII,  "s",      "S",      "\023",   "",       "" }, /* 31 s */
	{ KB_ASCII,  "d",      "D",      "\004",   "",       "" }, /* 32 d */
	{ KB_ASCII,  "f",      "F",      "\006",   "",       "" }, /* 33 f */
	{ KB_ASCII,  "g",      "G",      "\007",   "",       "" }, /* 34 g */
	{ KB_ASCII,  "h",      "H",      "\010",   "",       "" }, /* 35 h */
	{ KB_ASCII,  "j",      "J",      "\n",     "",       "" }, /* 36 j */
	{ KB_ASCII,  "k",      "K",      "\013",   "",       "" }, /* 37 k */
	{ KB_ASCII,  "l",      "L",      "\014",   "",       "" }, /* 38 l */
	{ KB_ASCII,  ";",      ":",      ";",      "",       "" }, /* 39 ; */
	{ KB_ASCII,  "'",      "\"",     "'",      "",       "" }, /* 40 ' */
	{ KB_ASCII,  "`",      "~",      "`",      "",       "" }, /* 41 ` */
	{ KB_SHIFT,  "\001",   "",       "",       "",       "" }, /* 42 shift */
	{ KB_ASCII,  "\\",     "|",      "\034",   "",       "" }, /* 43 \ */
	{ KB_ASCII,  "z",      "Z",      "\032",   "",       "" }, /* 44 z */
	{ KB_ASCII,  "x",      "X",      "\030",   "",       "" }, /* 45 x */
	{ KB_ASCII,  "c",      "C",      "\003",   "",       "" }, /* 46 c */
	{ KB_ASCII,  "v",      "V",      "\026",   "",       "" }, /* 47 v */
	{ KB_ASCII,  "b",      "B",      "\002",   "",       "" }, /* 48 b */
	{ KB_ASCII,  "n",      "N",      "\016",   "",       "" }, /* 49 n */
	{ KB_ASCII,  "m",      "M",      "\r",     "",       "" }, /* 50 m */
	{ KB_ASCII,  ",",      "<",      "<",      "",       "" }, /* 51 , */
	{ KB_ASCII,  ".",      ">",      ">",      "",       "" }, /* 52 . */
	{ KB_ASCII,  "/",      "?",      "\037",   "",       "" }, /* 53 / */
	{ KB_SHIFT,  "\002",   "",       "",       "",       "" }, /* 54 shift */
	{ KB_KP,     "*",      "*",      "*",      "",       "" }, /* 55 kp * */
	{ KB_ALT,    "",       "",       "",       "",       "" }, /* 56 alt */
	{ KB_ASCII,  " ",      " ",      "\000",   "",       "" }, /* 57 space */
	{ KB_CAPS,   "",       "",       "",       "",       "" }, /* 58 caps */
	{ KB_FUNC,   "\033[M", "\033[Y", "\033[k", "",       "" }, /* 59 f1 */
	{ KB_FUNC,   "\033[N", "\033[Z", "\033[l", "",       "" }, /* 60 f2 */
	{ KB_FUNC,   "\033[O", "\033[a", "\033[m", "",       "" }, /* 61 f3 */
	{ KB_FUNC,   "\033[P", "\033[b", "\033[n", "",       "" }, /* 62 f4 */
	{ KB_FUNC,   "\033[Q", "\033[c", "\033[o", "",       "" }, /* 63 f5 */
	{ KB_FUNC,   "\033[R", "\033[d", "\033[p", "",       "" }, /* 64 f6 */
	{ KB_FUNC,   "\033[S", "\033[e", "\033[q", "",       "" }, /* 65 f7 */
	{ KB_FUNC,   "\033[T", "\033[f", "\033[r", "",       "" }, /* 66 f8 */
	{ KB_FUNC,   "\033[U", "\033[g", "\033[s", "",       "" }, /* 67 f9 */
	{ KB_FUNC,   "\033[V", "\033[h", "\033[t", "",       "" }, /* 68 f10 */
	{ KB_NUM,    "",       "",       "",       "",       "" }, /* 69 num lock */
	{ KB_SCROLL, "",       "",       "",       "",       "" }, /* 70 scroll lock */
	{ KB_KP,     "7",      "\033[H", "7",      "",       "" }, /* 71 kp 7 */
	{ KB_KP,     "8",      "\033[A", "8",      "",       "" }, /* 72 kp 8 */
	{ KB_KP,     "9",      "\033[I", "9",      "",       "" }, /* 73 kp 9 */
	{ KB_KP,     "-",      "-",      "-",      "",       "" }, /* 74 kp - */
	{ KB_KP,     "4",      "\033[D", "4",      "",       "" }, /* 75 kp 4 */
	{ KB_KP,     "5",      "\033[E", "5",      "",       "" }, /* 76 kp 5 */
	{ KB_KP,     "6",      "\033[C", "6",      "",       "" }, /* 77 kp 6 */
	{ KB_KP,     "+",      "+",      "+",      "",       "" }, /* 78 kp + */
	{ KB_KP,     "1",      "\033[F", "1",      "",       "" }, /* 79 kp 1 */
	{ KB_KP,     "2",      "\033[B", "2",      "",       "" }, /* 80 kp 2 */
	{ KB_KP,     "3",      "\033[G", "3",      "",       "" }, /* 81 kp 3 */
	{ KB_KP,     "0",      "\033[L", "0",      "",       "" }, /* 82 kp 0 */
	{ KB_KP,     ",",      "\177",   ",",      "",       "" }, /* 83 kp . */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 84 0 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 85 0 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 86 0 */
	{ KB_FUNC,   "\033[W", "\033[i", "\033[u", "",       "" }, /* 87 f11 */
	{ KB_FUNC,   "\033[X", "\033[j", "\033[v", "",       "" }, /* 88 f12 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 89 0 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 90 0 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 91 0 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 92 0 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 93 0 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 94 0 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 95 0 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 96 0 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 97 0 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 98 0 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 99 0 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 100 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 101 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 102 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 103 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 104 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 105 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 106 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 107 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 108 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 109 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 110 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 111 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 112 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 113 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 114 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 115 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 116 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 117 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 118 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 119 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 120 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 121 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 122 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 123 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 124 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 125 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 126 */
	{ KB_NONE,   "",       "",       "",       "",       "" }, /* 127 */
};


/* Function gets characters from keyboard */
u8 *_ttypc_kbd_get(ttypc_t *ttypc)
{
	u8 dt;
	char *more = NULL;

	dt = inb(ttypc->inp_base);

	/* Key is released */
	if (dt & 0x80) {
		dt &= 0x7f;

		switch (scan_codes[dt].type) {
		case KB_NUM:
			ttypc->shiftst &= ~KB_NUM;
			break;
		case KB_CAPS:
			ttypc->shiftst &= ~KB_CAPS;
			break;
		case KB_SCROLL:
			ttypc->shiftst &= ~KB_SCROLL;
			break;
		case KB_SHIFT:
			ttypc->shiftst &= ~KB_SHIFT;
			break;
		case KB_ALT:
			if (ttypc->extended)
				ttypc->shiftst &= ~KB_ALTGR;
			else
				ttypc->shiftst &= ~KB_ALT;
			break;
		case KB_CTL:
			ttypc->shiftst &= ~KB_CTL;
			break;
		}
	}
	
	/* Key is pressed */
	else {
		switch (scan_codes[dt].type) {

		/* Locking keys - Caps, Scroll, Num */
		case KB_NUM:
			if (ttypc->shiftst & KB_NUM)
				break;
			ttypc->shiftst |= KB_NUM;
			ttypc->lockst ^= KB_NUM;
			break;

		case KB_CAPS:
			if (ttypc->shiftst & KB_CAPS)
				break;
			ttypc->shiftst |= KB_CAPS;
			ttypc->lockst ^= KB_CAPS;
			break;

		case KB_SCROLL:
			if (ttypc->shiftst & KB_SCROLL)
				break;
			ttypc->shiftst |= KB_SCROLL;
			ttypc->lockst ^= KB_SCROLL;
			break;

		/* Special no locking keys */
		case KB_SHIFT:
			ttypc->shiftst |= KB_SHIFT;
			break;

		case KB_ALT:
			if (ttypc->extended)
				ttypc->shiftst |= KB_ALTGR;
			else
				ttypc->shiftst |= KB_ALT;
			break;

		case KB_CTL:
			ttypc->shiftst |= KB_CTL;
			break;
		
		/* Regular ASCII */
		case KB_ASCII:

			/* Control is pressed */
			if (ttypc->shiftst & KB_CTL)
				more = scan_codes[dt].ctl;
			
			/* Right alt and right alt with shift */
			else if (ttypc->shiftst & KB_ALTGR) {
				if (ttypc->shiftst & KB_SHIFT)
					more = scan_codes[dt].shift_altgr;
				else
					more = scan_codes[dt].altgr;								
			}
			
			/* Shift */
			else if (ttypc->shiftst & KB_SHIFT)
				more = scan_codes[dt].shift;
			else
				more = scan_codes[dt].unshift;

			/* CAPS LOCK */
			if ((ttypc->lockst & KB_CAPS) /*&& (*scan_codes[dt].unshift >= 'a') && (*scan_codes[dt].unshift <= 'z')*/) {

				if (ttypc->shiftst & KB_SHIFT)
					more = scan_codes[dt].unshift;
				else
					more = scan_codes[dt].shift;
			}

			ttypc->extended = 0;	
			break;
		
		/* Key without meaning */	
		case KB_NONE:
			break;
			
		/* Function key */	
		case KB_FUNC: {
			if (ttypc->shiftst & KB_SHIFT)
				more = scan_codes[dt].shift;
			else if (ttypc->shiftst & KB_CTL)
				more = scan_codes[dt].ctl;
			else
				more = scan_codes[dt].unshift;
			
			ttypc->extended = 0;
			break;
		}
		
		/* Keypad */
		case KB_KP:
			
			/* Reboot sequence */
						
			/*
			if ((ttypcv->shiftst & KB_ALT) && (ttypcv->shiftst & KB_CTL) && (dt == 83)) {
				return ttypcv->capchar;
			}*/
			
			if (ttypc->shiftst & (KB_SHIFT | KB_CTL) || (ttypc->lockst & KB_NUM) == 0 || ttypc->extended)
				more = scan_codes[dt].shift;
			else
				more = scan_codes[dt].unshift;
			ttypc->extended = 0;
			break;
		}
	}
	ttypc->extended = 0;

	return (u8 *)more;
}


/* Keyboard interrupt handler */


static int ttypc_kbd_interrupt(unsigned int n, void *arg)
{
	ttypc_t *ttypc = (ttypc_t *)arg;

	return ttypc->rcond;
}


void ttypc_kbd_ctlthr(void *arg)
{
	ttypc_t *ttypc = (ttypc_t *)arg;
	u8 *s;

	for (;;) {
		mutexLock(ttypc->rlock);

		while (inb(ttypc->inp_base + 4) != 0x1d) {
			condWait(ttypc->rcond, ttypc->rlock, 0);
		}

		/* Put characters received from keyboard to fifo */
		if ((s = _ttypc_kbd_get(ttypc)) == NULL) {
			mutexUnlock(ttypc->rlock);
			continue;
		}

		mutexUnlock(ttypc->rlock);

		/* Put data into virtual terminal input buffer */
		if (s != NULL) {

			if (!strcmp((char *)s, "\033[k"))
				ttypc_vga_switch(&ttypc->virtuals[0]);
			else if (!strcmp((char *)s, "\033[l"))
				ttypc_vga_switch(&ttypc->virtuals[1]);
			else if (!strcmp((char *)s, "\033[m"))
				ttypc_vga_switch(&ttypc->virtuals[2]);
			else if (!strcmp((char *)s, "\033[n"))
				ttypc_vga_switch(&ttypc->virtuals[3]);
			else
				ttypc_virt_sadd(ttypc->cv, s, strlen((char *)s));
		}
	}
	return;
}


int _ttypc_kbd_init(ttypc_t *ttypc)
{
	void *stack;

	ttypc->extended = 0;
	ttypc->lockst = 0;
	ttypc->shiftst = 0;

	mutexCreate(&ttypc->rlock);
	condCreate(&ttypc->rcond);

	/* Allocate memory for character buffer */
	if ((ttypc->rbuff = (u8 **)malloc(sizeof(u8 *) * 128)) == NULL)
		return -ENOMEM;

	ttypc->rbuffsz = 128;
	ttypc->rb = 0;
	ttypc->rp = 0;

	/* Attach interrupt and launch interrupt thread */
	if ((stack = malloc(2048)) == NULL)
		return -ENOMEM;
	beginthread(ttypc_kbd_ctlthr, 1, stack, 2048, (void *)ttypc);

	interrupt(1, ttypc_kbd_interrupt, ttypc, ttypc->rcond, &ttypc->inth);

	/* Read byte from controller (reset is neccessary) */
	inb(ttypc->inp_base);

	return EOK;
}
