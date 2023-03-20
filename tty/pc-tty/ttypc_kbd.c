/*
 * Phoenix-RTOS
 *
 * PS/2 101-key US keyboard (based on FreeBSD 4.4 pcvt)
 *
 * Copyright 2001, 2007-2008 Pawel Pisarczyk
 * Copyright 2012, 2017, 2019, 2020 Phoenix Systems
 * Author: Pawel Pisarczyk, Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include <sys/interrupt.h>
#include <sys/io.h>
#include <sys/threads.h>
#include <sys/reboot.h>

#include "ttypc_kbd.h"
#include "ttypc_vga.h"


/* Keyboard key map entry */
typedef struct {
	unsigned int type;
	char *unshift;
	char *shift;
	char *ctl;
	char *altgr;
	char *shift_altgr;
} ttypc_kbd_keymap_t;


/* U.S 101 keys keyboard map */
static const ttypc_kbd_keymap_t scodes[] = {
	/*type       unshift    shift      ctl        altgr      shift_altgr scancode */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 0 unused */
	{ KB_ASCII,  "\033",    "\033",    "\033",    "",        "" },    /* 1 ESCape */
	{ KB_ASCII,  "1",       "!",       "\033[m",  "",        "" },    /* 2 1 */
	{ KB_ASCII,  "2",       "@",       "\033[n",  "",        "" },    /* 3 2 */
	{ KB_ASCII,  "3",       "#",       "\033[o",  "",        "" },    /* 4 3 */
	{ KB_ASCII,  "4",       "$",       "\033[p",  "",        "" },    /* 5 4 */
	{ KB_ASCII,  "5",       "%",       "\035",    "",        "" },    /* 6 5 */
	{ KB_ASCII,  "6",       "^",       "\036",    "",        "" },    /* 7 6 */
	{ KB_ASCII,  "7",       "&",       "\037",    "",        "" },    /* 8 7 */
	{ KB_ASCII,  "8",       "*",       "\010",    "",        "" },    /* 9 8 */
	{ KB_ASCII,  "9",       "(",       "9",       "",        "" },    /* 10 9 */
	{ KB_ASCII,  "0",       ")",       "0",       "",        "" },    /* 11 0 */
	{ KB_ASCII,  "-",       "_",       "-",       "",        "" },    /* 12 - */
	{ KB_ASCII,  "=",       "+",       "=",       "",        "" },    /* 13 = */
	{ KB_ASCII,  "\b \b",   "\177",    "\010",    "",        "" },    /* 14 backspace */
	{ KB_ASCII,  "\t",      "\t",      "\t",      "",        "" },    /* 15 tab */
	{ KB_ASCII,  "q",       "Q",       "\021",    "",        "" },    /* 16 q */
	{ KB_ASCII,  "w",       "W",       "\027",    "",        "" },    /* 17 w */
	{ KB_ASCII,  "e",       "E",       "\005",    "",        "" },    /* 18 e */
	{ KB_ASCII,  "r",       "R",       "\022",    "",        "" },    /* 19 r */
	{ KB_ASCII,  "t",       "T",       "\024",    "",        "" },    /* 20 t */
	{ KB_ASCII,  "y",       "Y",       "\031",    "",        "" },    /* 21 y */
	{ KB_ASCII,  "u",       "U",       "\025",    "",        "" },    /* 22 u */
	{ KB_ASCII,  "i",       "I",       "\011",    "",        "" },    /* 23 i */
	{ KB_ASCII,  "o",       "O",       "\017",    "",        "" },    /* 24 o */
	{ KB_ASCII,  "p",       "P",       "\020",    "",        "" },    /* 25 p */
	{ KB_ASCII,  "[",       "{",       "\033",    "",        "" },    /* 26 [ */
	{ KB_ASCII,  "]",       "}",       "\035",    "",        "" },    /* 27 ] */
	{ KB_ASCII,  "\n",      "\n",      "\r",      "",        "" },    /* 28 return */
	{ KB_CTL,    "",        "",        "",        "",        "" },    /* 29 control */
	{ KB_ASCII,  "a",       "A",       "\001",    "",        "" },    /* 30 a */
	{ KB_ASCII,  "s",       "S",       "\023",    "",        "" },    /* 31 s */
	{ KB_ASCII,  "d",       "D",       "\004",    "",        "" },    /* 32 d */
	{ KB_ASCII,  "f",       "F",       "\006",    "",        "" },    /* 33 f */
	{ KB_ASCII,  "g",       "G",       "\007",    "",        "" },    /* 34 g */
	{ KB_ASCII,  "h",       "H",       "\010",    "",        "" },    /* 35 h */
	{ KB_ASCII,  "j",       "J",       "\n",      "",        "" },    /* 36 j */
	{ KB_ASCII,  "k",       "K",       "\013",    "",        "" },    /* 37 k */
	{ KB_ASCII,  "l",       "L",       "\014",    "",        "" },    /* 38 l */
	{ KB_ASCII,  ";",       ":",       ";",       "",        "" },    /* 39 ; */
	{ KB_ASCII,  "'",       "\"",      "'",       "",        "" },    /* 40 ' */
	{ KB_ASCII,  "`",       "~",       "\000",    "",        "" },    /* 41 ` */
	{ KB_SHIFT,  "\001",    "",        "",        "",        "" },    /* 42 shift */
	{ KB_ASCII,  "\\",      "|",       "\034",    "",        "" },    /* 43 \ */
	{ KB_ASCII,  "z",       "Z",       "\032",    "",        "" },    /* 44 z */
	{ KB_ASCII,  "x",       "X",       "\030",    "",        "" },    /* 45 x */
	{ KB_ASCII,  "c",       "C",       "\003",    "",        "" },    /* 46 c */
	{ KB_ASCII,  "v",       "V",       "\026",    "",        "" },    /* 47 v */
	{ KB_ASCII,  "b",       "B",       "\002",    "",        "" },    /* 48 b */
	{ KB_ASCII,  "n",       "N",       "\016",    "",        "" },    /* 49 n */
	{ KB_ASCII,  "m",       "M",       "\r",      "",        "" },    /* 50 m */
	{ KB_ASCII,  ",",       "<",       ",",       "",        "" },    /* 51 , */
	{ KB_ASCII,  ".",       ">",       ".",       "",        "" },    /* 52 . */
	{ KB_ASCII,  "/",       "?",       "\037",    "",        "" },    /* 53 / */
	{ KB_SHIFT,  "\002",    "",        "",        "",        "" },    /* 54 shift */
	{ KB_KP,     "*",       "*",       "*",       "",        "" },    /* 55 kp * */
	{ KB_ALT,    "",        "",        "",        "",        "" },    /* 56 alt */
	{ KB_ASCII,  " ",       " ",       "\000",    "",        "" },    /* 57 space */
	{ KB_CAPS,   "",        "",        "",        "",        "" },    /* 58 caps */
	{ KB_FUNC,   "\033[OP", "\033[a",  "\033[m",  "",        "" },    /* 59 f1 */
	{ KB_FUNC,   "\033[OQ", "\033[b",  "\033[n",  "",        "" },    /* 60 f2 */
	{ KB_FUNC,   "\033[OR", "\033[c",  "\033[o",  "",        "" },    /* 61 f3 */
	{ KB_FUNC,   "\033[OS", "\033[d",  "\033[p",  "",        "" },    /* 62 f4 */
	{ KB_FUNC,   "\033[15~","\033[e",  "\033[q",  "",        "" },    /* 63 f5 */
	{ KB_FUNC,   "\033[17~","\033[f",  "\033[r",  "",        "" },    /* 64 f6 */
	{ KB_FUNC,   "\033[18~","\033[g",  "\033[s",  "",        "" },    /* 65 f7 */
	{ KB_FUNC,   "\033[19~","\033[h",  "\033[t",  "",        "" },    /* 66 f8 */
	{ KB_FUNC,   "\033[20~","\033[i",  "\033[u",  "",        "" },    /* 67 f9 */
	{ KB_FUNC,   "\033[21~","\033[j",  "\033[v",  "",        "" },    /* 68 f10 */
	{ KB_NUM,    "",        "",        "",        "",        "" },    /* 69 num lock */
	{ KB_SCROLL, "",        "",        "",        "",        "" },    /* 70 scroll lock */
	{ KB_KP,     "7",       "\033[H",  "7",       "",        "" },    /* 71 kp 7 */
	{ KB_KP,     "8",       "\033[A",  "8",       "",        "" },    /* 72 kp 8 */
	{ KB_KP,     "9",       "\033[5~", "9",       "",        "" },    /* 73 kp 9 */
	{ KB_KP,     "-",       "-",       "-",       "",        "" },    /* 74 kp - */
	{ KB_KP,     "4",       "\033[D",  "4",       "",        "" },    /* 75 kp 4 */
	{ KB_KP,     "5",       "\033[E",  "5",       "",        "" },    /* 76 kp 5 */
	{ KB_KP,     "6",       "\033[C",  "6",       "",        "" },    /* 77 kp 6 */
	{ KB_KP,     "+",       "+",       "+",       "",        "" },    /* 78 kp + */
	{ KB_KP,     "1",       "\033[F",  "1",       "",        "" },    /* 79 kp 1 */
	{ KB_KP,     "2",       "\033[B",  "2",       "",        "" },    /* 80 kp 2 */
	{ KB_KP,     "3",       "\033[6~", "3",       "",        "" },    /* 81 kp 3 */
	{ KB_KP,     "0",       "\033[2~", "0",       "",        "" },    /* 82 kp 0 */
	{ KB_KP,     ",",       "\033[3~", ",",       "",        "" },    /* 83 kp . */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 84 0 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 85 0 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 86 0 */
	{ KB_FUNC,   "\033[23~","\033[k",  "\033[w",  "",        "" },    /* 87 f11 */
	{ KB_FUNC,   "\033[24~","\033[l",  "\033[x",  "",        "" },    /* 88 f12 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 89 0 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 90 0 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 91 0 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 92 0 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 93 0 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 94 0 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 95 0 */
	{ KB_EXT,    "",        "",        "",        "",        "" },    /* 96 extended */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 97 0 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 98 0 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 99 0 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 100 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 101 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 102 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 103 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 104 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 105 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 106 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 107 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 108 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 109 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 110 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 111 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 112 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 113 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 114 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 115 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 116 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 117 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 118 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 119 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 120 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 121 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 122 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 123 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 124 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 125 */
	{ KB_NONE,   "",        "",        "",        "",        "" },    /* 126 */
	{ KB_NONE,   "",        "",        "",        "",        "" }     /* 127 */
};


/* KB_KP (keypad keys) modifiers map */
static const unsigned char kpmod[] = {
	0, /* 0 no modifiers */
	5, /* 1 ctl */
	2, /* 2 shift */
	6, /* 3 ctl + shift */
	3, /* 4 alt */
	7, /* 5 ctl + alt */
	4, /* 6 shift + alt */
	8, /* 7 ctl + shift + alt */
	0, /* 8 altgr */
	5, /* 9 ctrl + altgr */
	2, /* 10 shift + altgr */
	6, /* 11 ctl + shift + altgr */
	3, /* 12 alt + altgr */
	7, /* 13 ctl + alt + altgr */
	4, /* 14 shift + alt + altgr */
	8  /* 15 ctl + shift + alt + altgr */
};


/* Gets key code from keyboard */
static char *_ttypc_kbd_get(ttypc_t *ttypc)
{
	static const char erreboot[] = "Failed to reboot\n";
	static unsigned char ext = 0, lkey = 0, lkeyup = 0, lext = 0;
	unsigned char dt;
	char *s = NULL;

	dt = inb((void *)ttypc->kbd);

	/* Extended scan code */
	if (scodes[dt & 0x7f].type == KB_EXT) {
		ext = 1;
		return NULL;
	}

	/* Key is released */
	if (dt & 0x80) {
		dt &= 0x7f;

		switch (scodes[dt].type) {
		case KB_SCROLL:
			if (!ext)
				ttypc->shiftst &= ~KB_SCROLL;
			break;

		case KB_NUM:
			if (!ext)
				ttypc->shiftst &= ~KB_NUM;
			break;

		case KB_CAPS:
			if (!ext)
				ttypc->shiftst &= ~KB_CAPS;
			break;

		case KB_CTL:
			ttypc->shiftst &= ~KB_CTL;
			break;

		case KB_SHIFT:
			if (!ext)
				ttypc->shiftst &= ~KB_SHIFT;
			break;

		case KB_ALT:
			if (ext)
				ttypc->shiftst &= ~KB_ALTGR;
			else
				ttypc->shiftst &= ~KB_ALT;
			break;
		}

		/* Last key released */
		if ((dt == lkey) && (ext == lext)) {
			lkey = 0;
			lext = 0;
			lkeyup = 1;
		}
	}
	/* Key is pressed */
	else {
		switch (scodes[dt].type) {
		/* Lock keys - Scroll, Num, Caps */
		case KB_SCROLL:
			if (ttypc->shiftst & KB_SCROLL)
				break;
			ttypc->shiftst |= KB_SCROLL;
			ttypc->lockst ^= KB_SCROLL;
			_ttypc_kbd_updateled(ttypc);
			break;

		case KB_NUM:
			if (ttypc->shiftst & KB_NUM)
				break;
			ttypc->shiftst |= KB_NUM;
			ttypc->lockst ^= KB_NUM;
			_ttypc_kbd_updateled(ttypc);
			break;

		case KB_CAPS:
			if (ttypc->shiftst & KB_CAPS)
				break;
			ttypc->shiftst |= KB_CAPS;
			ttypc->lockst ^= KB_CAPS;
			_ttypc_kbd_updateled(ttypc);
			break;

		/* Shift keys - Ctl, Shift, Alt */
		case KB_CTL:
			ttypc->shiftst |= KB_CTL;
			break;

		case KB_SHIFT:
			ttypc->shiftst |= KB_SHIFT;
			break;

		case KB_ALT:
			if (ext)
				ttypc->shiftst |= KB_ALTGR;
			else
				ttypc->shiftst |= KB_ALT;
			break;

		/* Function keys */
		case KB_FUNC:
		/* Regular ASCII */
		case KB_ASCII:
			/* Keys with extended scan codes don't depend on any modifiers */
			if (ext) {
				/* Handles keypad '/' key */
				s = scodes[dt].unshift;
				break;
			}
			/* Control modifier */
			if (ttypc->shiftst & KB_CTL) {
				s = scodes[dt].ctl;
				break;
			}

			/* Right alt and right alt with shift modifiers */
			if (ttypc->shiftst & KB_ALTGR) {
				if (ttypc->shiftst & KB_SHIFT)
					s = scodes[dt].shift_altgr;
				else
					s = scodes[dt].altgr;
			}
			/* Shift modifier */
			else if (ttypc->shiftst & KB_SHIFT) {
				s = scodes[dt].shift;
			}
			/* No modifiers */
			else {
				s = scodes[dt].unshift;
			}

			/* Caps lock */
			if ((ttypc->lockst & KB_CAPS) && (*scodes[dt].unshift >= 'a') && (*scodes[dt].unshift <= 'z')) {
				if (s == scodes[dt].altgr)
					s = scodes[dt].shift_altgr;
				else if (s == scodes[dt].shift_altgr)
					s = scodes[dt].altgr;
				else if (s == scodes[dt].shift)
					s = scodes[dt].unshift;
				else if (s == scodes[dt].unshift)
					s = scodes[dt].shift;
			}
			break;

		/* Keys without meaning */
		case KB_NONE:
			break;

		/* Keypad */
		case KB_KP:
			/* Keys with extended scan codes don't depend on any modifiers */
			if (ext) {
				/* Handles DEL, HOME, END, PU, PD and arrow (non keypad) keys */
				s = scodes[dt].shift;
				break;
			}

			/* Shift modifier */
			if (ttypc->shiftst & KB_SHIFT)
				s = scodes[dt].shift;
			/* Control modifier */
			else if (ttypc->shiftst & KB_CTL)
				s = scodes[dt].ctl;
			/* No modifiers */
			else
				s = scodes[dt].unshift;

			/* Num lock */
			if (ttypc->lockst & KB_NUM) {
				if (s == scodes[dt].shift)
					s = scodes[dt].unshift;
				else if ((s == scodes[dt].ctl) || (s == scodes[dt].unshift))
					s = scodes[dt].shift;
			}
			break;
		}
	}

	/* AutoRepeat Mode */
	if (!ttypc->vt->arm && (dt == lkey) && (ext == lext))
		s = NULL;

	/* Reboot sequence - Ctl-Alt-Del */
	if (s && ext && !strcmp(s, "\033[3~") && (ttypc->shiftst == (KB_CTL | KB_ALT))) {
		if (reboot(PHOENIX_REBOOT_MAGIC) < 0)
			ttypc_vt_write(ttypc->vt, 0, erreboot, sizeof(erreboot));
		s = NULL;
	}

	if (lkeyup) {
		lkeyup = 0;
	}
	else if ((scodes[dt].type == KB_FUNC) || (scodes[dt].type == KB_ASCII) || (scodes[dt].type == KB_KP)) {
		lkey = dt;
		lext = ext;
	}
	ext = 0;

	return s;
}


/* Keyboard interrupt handler */
static int ttypc_kbd_interrupt(unsigned int n, void *arg)
{
	ttypc_t *ttypc = (ttypc_t *)arg;

	return ttypc->kcond;
}


static void ttypc_kbd_ctlthr(void *arg)
{
	ttypc_t *ttypc = (ttypc_t *)arg;
	ttypc_vt_t *cvt;
	char *s, k;
	char buff[10];
	unsigned char m;

	mutexLock(ttypc->klock);
	for (;;) {
		/* Wait for character codes to show up in keyboard output buffer */
		while (!(inb((void *)((uintptr_t)ttypc->kbd + 4)) & 0x01))
			condWait(ttypc->kcond, ttypc->klock, 0);

		mutexLock(ttypc->lock);
		mutexLock((cvt = ttypc->vt)->lock);

		if ((s = _ttypc_kbd_get(ttypc)) == NULL) {
			mutexUnlock(cvt->lock);
			mutexUnlock(ttypc->lock);
			continue;
		}

		/* Scroll up one line */
		if (!strcmp(s, "\033[A") && ((ttypc->lockst & KB_SCROLL) || (ttypc->shiftst == (KB_CTL | KB_SHIFT)))) {
			_ttypc_vga_scroll(cvt, 1);
		}
		/* Scroll down one line */
		else if (!strcmp(s, "\033[B") && ((ttypc->lockst & KB_SCROLL) || (ttypc->shiftst == (KB_CTL | KB_SHIFT)))) {
			_ttypc_vga_scroll(cvt, -1);
		}
		/* Scroll up one page */
		else if (!strcmp(s, "\033[5~") && ((ttypc->lockst & KB_SCROLL) || (ttypc->shiftst == KB_SHIFT))) {
			_ttypc_vga_scroll(cvt, cvt->rows);
		}
		/* Scroll down one page */
		else if (!strcmp(s, "\033[6~") && ((ttypc->lockst & KB_SCROLL) || (ttypc->shiftst == KB_SHIFT))) {
			_ttypc_vga_scroll(cvt, -cvt->rows);
		}
		/* Scroll to top */
		else if (!strcmp(s, "\033[H") && ((ttypc->lockst & KB_SCROLL) || (ttypc->shiftst == KB_SHIFT))) {
			_ttypc_vga_scroll(cvt, cvt->scrbsz - cvt->scrbpos);
		}
		/* Scroll to bottom */
		else if (!strcmp(s, "\033[F") && ((ttypc->lockst & KB_SCROLL) || (ttypc->shiftst == KB_SHIFT))) {
			_ttypc_vga_scroll(cvt, -cvt->scrbpos);
		}
		/* Switch between VTs (up to 12) */
		else if (!strncmp(s, "\033[", 2) && (s[2] >= 'm') && (s[2] <= 'x') && !s[3]) {
			if ((s[2] - 'm') < NVTS)
				_ttypc_vga_switch(ttypc->vts + (s[2] - 'm'));
		}
		/* Regular character processing */
		else {
			/* Process KP/FN keys */
			if (!strncmp(s, "\033[", 2)) {
				m = kpmod[ttypc->shiftst & 0x0f];

				if ((k = s[2])) {
					if (!s[3] && ((k >= 'A' && k <= 'F') || k == 'H')) {
						if (m) {
							snprintf(buff, sizeof(buff), "\033[1;%u%c", m, k);
							s = buff;
						}
						/* Cursor Key Mode */
						else if (cvt->ckm) {
							snprintf(buff, sizeof(buff), "\033O%c", k);
							s = buff;
						}
					}
					else if ((s[3] == '~') && !s[4] && (k >= '2' && k <= '6' && (k != '4'))) {
						if (m) {
							snprintf(buff, sizeof(buff), "\033[%c;%u~", k, m);
							s = buff;
						}
					}
				}
			}
			else if (!strcmp(s, "\r") || !strcmp(s, "\n")) {
				/* Line feed/New line Mode */
				if (cvt->lnm)
					s = "\r\n";
			}

			while (*s && !libtty_putchar(&cvt->tty, *s++, NULL));
		}

		mutexUnlock(cvt->lock);
		mutexUnlock(ttypc->lock);
	}
}


/* Waits for keyboard controller status bit with small timeout */
static int ttypc_kbd_waitstatus(ttypc_t *ttypc, unsigned char bit, unsigned char state)
{
	unsigned int i;

	for (i = 0; i < 0xffff; i++) {
		if (!(inb((void *)((uintptr_t)ttypc->kbd + 4)) & ((1 << bit) ^ (state << bit))))
			return EOK;
		usleep(10);
	}

	return -ETIMEDOUT;
}


/* Reads a byte from keyboard controller output buffer */
/*
 * FIXME: (unused) Function not to be removed, needs to be preserved
 * for future implementation of ps2-aux (mouse device) support.
 */
__attribute__((unused)) static int ttypc_kbd_read(ttypc_t *ttypc)
{
	int err;

	/* Wait for output buffer not to be empty */
	if ((err = ttypc_kbd_waitstatus(ttypc, 0, 1)) < 0)
		return err;

	return inb((void *)ttypc->kbd);
}


/* Writes a byte to keyboard controller input buffer */
static int ttypc_kbd_write(ttypc_t *ttypc, unsigned char byte)
{
	int err;

	/* Wait for input buffer to be empty */
	if ((err = ttypc_kbd_waitstatus(ttypc, 1, 0)) < 0)
		return err;

	outb((void *)ttypc->kbd, byte);

	return EOK;
}


/* May not work for PS/2 emulation through USB legacy support */
int _ttypc_kbd_updateled(ttypc_t *ttypc)
{
	do {
		/* Send update LEDs command */
		if (ttypc_kbd_write(ttypc, 0xed) < 0)
			break;

		/* Send LEDs state */
		if (ttypc_kbd_write(ttypc, (ttypc->lockst >> 4) & 0x07) < 0)
			break;

		return 1;
	} while (0);

	return 0;
}


void ttypc_kbd_destroy(ttypc_t *ttypc)
{
	resourceDestroy(ttypc->klock);
	resourceDestroy(ttypc->kcond);
	resourceDestroy(ttypc->kinth);
}


int ttypc_kbd_init(ttypc_t *ttypc)
{
	int i, err;

	/* PS/2 Keyboard base IO-port */
	ttypc->kbd = (void *)0x60;

	/* Flush output buffer (max 16 bytes) */
	for (i = 0; i < 16; i++) {
		if (!(inb((void *)((uintptr_t)ttypc->kbd + 4)) & 1))
			break;

		inb((void *)ttypc->kbd);
		usleep(10);
	}

	/* Configure typematic */
	do {
		/* Send set typematic rate/delay command */
		if (ttypc_kbd_write(ttypc, 0xf3) < 0)
			break;

		/* 250 ms / 30.0 reports/sec */
		if (ttypc_kbd_write(ttypc, 0) < 0)
			break;
	} while (0);

	if ((err = mutexCreate(&ttypc->klock)) < 0)
		return err;

	if ((err = condCreate(&ttypc->kcond)) < 0) {
		resourceDestroy(ttypc->klock);
		return err;
	}

	/* Attach interrupt */
	if ((err = interrupt((ttypc->kirq = 1), ttypc_kbd_interrupt, ttypc, ttypc->kcond, &ttypc->kinth)) < 0) {
		resourceDestroy(ttypc->klock);
		resourceDestroy(ttypc->kcond);
		return err;
	}

	/* Launch keyboard control thread */
	if ((err = beginthread(ttypc_kbd_ctlthr, 1, ttypc->kstack, sizeof(ttypc->kstack), ttypc)) < 0) {
		resourceDestroy(ttypc->klock);
		resourceDestroy(ttypc->kcond);
		resourceDestroy(ttypc->kinth);
		return err;
	}

	return EOK;
}
