/*
 * Phoenix-RTOS
 *
 * PS/2 101-key US keyboard and 3-button mouse (based on FreeBSD 4.4 pcvt)
 *
 * Copyright 2001, 2007-2008 Pawel Pisarczyk
 * Copyright 2012, 2017, 2019, 2020, 2024 Phoenix Systems
 * Author: Pawel Pisarczyk, Lukasz Kosinski, Adam Greloch
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <paths.h>
#include <poll.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

#include <sys/file.h>
#include <sys/interrupt.h>
#include <sys/io.h>
#include <sys/threads.h>
#include <sys/reboot.h>

#include <libklog.h>
#include <fcntl.h>
#include <posix/utils.h>

#include "ttypc_kbd.h"
#include "ttypc_vga.h"
#include "event_queue.h"


/* Keyboard key map entry */
typedef struct {
	unsigned int type;
	char *unshift;
	char *shift;
	char *ctl;
	char *altgr;
	char *shift_altgr;
} ttypc_kbd_keymap_t;


typedef struct {
	ttypc_t *ttypc;
	unsigned int kbd_port;
	handle_t kplock;
	handle_t kcond;

	unsigned int mouse_port;
} poolthr_args_t;

poolthr_args_t poolthr_args;


typedef struct {
	ttypc_t *ttypc;
	handle_t kcond;
} ctlthr_args_t;

ctlthr_args_t ctlthr_args;


/* U.S 101 keys keyboard map */
/* clang-format off */
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

/* clang-format on */

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

	/* Copy this event in raw form to queue */
	event_queue_put(ttypc->keq, dt, 1, 0);

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


/* Keyboard/mouse interrupt handler */
static int ttypc_kbd_interrupt(unsigned int n, void *arg)
{
	ttypc_t *ttypc = (ttypc_t *)arg;

	return ttypc->kmcond;
}


static unsigned char ttypc_kbd_read_ctrl_buf(ttypc_t *ttypc)
{
	return inb((void *)((uintptr_t)ttypc->kbd + 4));
}


/* Macros for distinguishing the origin of pending output */
#define KBD_OUTPUT_PENDING(status)   (((status) & (1 << 0)) && !((status) & (1 << 5)))
#define MOUSE_OUTPUT_PENDING(status) (((status) & (1 << 0)) && ((status) & (1 << 5)))


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


static void ttypc_kbd_handle_mouse_event(ttypc_t *ttypc)
{
	unsigned char b = inb((void *)ttypc->kbd);

	event_queue_put(ttypc->meq, b, 3, 0);
}


static void ttypc_kbd_ctlthr(void *arg)
{
	ctlthr_args_t *ctlthr_args = (ctlthr_args_t *)arg;
	ttypc_t *ttypc = ctlthr_args->ttypc;
	ttypc_vt_t *cvt;
	char *s, k;
	char buff[10];
	unsigned char m;
	unsigned char status;

	mutexLock(ttypc->klock);
	for (;;) {
		while (1) {
			status = ttypc_kbd_read_ctrl_buf(ttypc);
			if (KBD_OUTPUT_PENDING(status)) {
				break;
			}
			else if (MOUSE_OUTPUT_PENDING(status)) {
				ttypc_kbd_handle_mouse_event(ttypc);
			}
			else {
				condWait(ttypc->kmcond, ttypc->klock, 0);
			}
		}

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


/* Reads a byte from keyboard I/O buffer */
static int ttypc_kbd_read(ttypc_t *ttypc)
{
	int err;

	/* Wait for output buffer not to be empty */
	if ((err = ttypc_kbd_waitstatus(ttypc, 0, 1)) < 0)
		return err;

	return inb((void *)ttypc->kbd);
}


static int _ttypc_kbd_write_to_port(ttypc_t *ttypc, void *port, unsigned char byte)
{
	int err;

	/* Wait for input buffer to be empty */
	if ((err = ttypc_kbd_waitstatus(ttypc, 1, 0)) < 0)
		return err;

	outb(port, byte);

	return EOK;
}


/* Writes a byte to keyboard control buffer */
static int ttypc_kbd_write_ctrl(ttypc_t *ttypc, unsigned char byte)
{
	return _ttypc_kbd_write_to_port(ttypc, (void *)ttypc->kbd + 4, byte);
}


/* Writes a byte to keyboard I/O buffer */
static int ttypc_kbd_write(ttypc_t *ttypc, unsigned char byte)
{
	return _ttypc_kbd_write_to_port(ttypc, (void *)ttypc->kbd, byte);
}


static void ttypc_kbd_poolthr(void *arg)
{
	poolthr_args_t *poolthr_args = (poolthr_args_t *)arg;
	ttypc_t *ttypc = poolthr_args->ttypc;
	msg_rid_t rid;
	msg_t msg;

	mutexLock(poolthr_args->kplock);
	for (;;) {
		if (msgRecv(poolthr_args->kbd_port, &msg, &rid) < 0)
			continue;

		if (libklog_ctrlHandle(poolthr_args->kbd_port, &msg, rid) == 0) {
			/* msg has been handled by libklog */
			continue;
		}

		switch (msg.type) {
			case mtOpen:
				msg.o.err = EOK;
				break;

			case mtRead:
				msg.o.err = event_queue_get(ttypc->keq, msg.o.data, 1, 0);
				break;

			case mtClose:
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(poolthr_args->kbd_port, &msg, rid);
	}
}


static void ttypc_kbd_mouse_poolthr(void *arg)
{
	poolthr_args_t *poolthr_args = (poolthr_args_t *)arg;
	ttypc_t *ttypc = poolthr_args->ttypc;
	msg_rid_t rid;
	msg_t msg;

	for (;;) {
		if (msgRecv(poolthr_args->mouse_port, &msg, &rid) < 0)
			continue;

		if (libklog_ctrlHandle(poolthr_args->mouse_port, &msg, rid) == 0) {
			/* msg has been handled by libklog */
			continue;
		}

		switch (msg.type) {
			case mtOpen:
				msg.o.err = EOK;
				break;

			case mtRead:
				if (msg.o.size < 3) {
					/* Pretend there's nothing to read. We don't want someone to
			 read partial mouse packets - this would mess up byte ordering
			 and confuse recipients. */
					msg.o.err = 0;
					break;
				}
				msg.o.err = event_queue_get(ttypc->meq, msg.o.data, 3, msg.i.io.mode);
				break;

			case mtWrite:
				if (ttypc_kbd_write(ttypc, ((unsigned char *)msg.i.data)[0]) < 0) {
					msg.o.err = -EINVAL;
					break;
				}
				msg.o.err = 1;
				break;

			case mtGetAttr:
				if ((msg.i.attr.type != atPollStatus)) {
					msg.o.err = -EINVAL;
					break;
				}

				msg.o.attr.val = 0;
				if (event_queue_count(ttypc->meq) >= 3)
					msg.o.attr.val |= POLLIN;

				msg.o.err = EOK;
				break;

			case mtClose:
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(poolthr_args->mouse_port, &msg, rid);
	}
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


static int _ttypc_kbd_mouse_init(ttypc_t *ttypc)
{
	unsigned char status;

	do {
		/* Enable mouse */
		ttypc_kbd_write_ctrl(ttypc, 0xA8);
		if ((status = ttypc_kbd_read(ttypc)) != 0xFA) {
			fprintf(stderr, "pc-tty: mouse not found\n");
			break;
		}

		/* Get compaq status byte */
		ttypc_kbd_write_ctrl(ttypc, 0x20);

		status = ttypc_kbd_read(ttypc);
		status |= (1 << 1);  /* Enable IRQ12 interrupt */
		status &= ~(1 << 5); /* Enable mouse clock (unset bit 5) */

		/* Set compaq status byte */
		ttypc_kbd_write_ctrl(ttypc, 0x60);
		ttypc_kbd_write(ttypc, status);

		/* Set defaults */
		ttypc_kbd_write_ctrl(ttypc, 0xD4);
		ttypc_kbd_write(ttypc, 0xF6);
		if ((status = ttypc_kbd_read(ttypc)) != 0xFA) {
			fprintf(stderr, "pc-tty: mouse initialization failed - no ACK after setting defaults\n");
			break;
		}

		/* Send mouse cmd - enable packet streaming */
		ttypc_kbd_write_ctrl(ttypc, 0xD4);
		ttypc_kbd_write(ttypc, 0xF4);
		if ((status = ttypc_kbd_read(ttypc)) != 0xFA) {
			fprintf(stderr, "pc-tty: mouse initialization failed - no ACK after enable packet streaming cmd\n");
			break;
		}

		/* Is the mouse alive? */
		ttypc_kbd_write_ctrl(ttypc, 0xD4);
		ttypc_kbd_write(ttypc, 0xEB);
		if ((status = ttypc_kbd_read(ttypc)) != 0xFA)
			fprintf(stderr, "pc-tty: mouse died\n");
	} while (0);

	if (status != 0xFA) {
		/** If mouse initialization had failed at any point, disable mouse
		  so that it doesn't clutter the I/O port with events we won't handle */
		ttypc_kbd_write_ctrl(ttypc, 0xA7);
		if ((status = ttypc_kbd_read(ttypc)) != 0xFA) {
			fprintf(stderr, "pc-tty: could not disable mouse (?)\n");
		}

		return -1;
	}

	return EOK;
}


void ttypc_kbd_destroy(ttypc_t *ttypc)
{
	resourceDestroy(ttypc->klock);
	resourceDestroy(ttypc->kmcond);
	event_queue_destroy(ttypc->keq);
	event_queue_destroy(ttypc->meq);
	resourceDestroy(ttypc->kinth);
	resourceDestroy(ttypc->minth);
	resourceDestroy(poolthr_args.kplock);
	resourceDestroy(poolthr_args.kcond);
}


int ttypc_kbd_init(ttypc_t *ttypc)
{
	int i, err;
	oid_t oid;
	bool has_mouse = false;

	memset(&poolthr_args, 0, sizeof(poolthr_args_t));
	memset(&ctlthr_args, 0, sizeof(ctlthr_args_t));

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

	/* Wait for the filesystem */
	while (lookup("/", NULL, &oid) < 0)
		usleep(10000);

	/* Create virtual keyboard device */
	do {
		if ((err = portCreate(&poolthr_args.kbd_port)) < 0) {
			fprintf(stderr, "pc-tty: failed to create keyboard port\n");
			break;
		}
		oid.port = poolthr_args.kbd_port;
		oid.id = 0;
		if ((err = create_dev(&oid, _PATH_KBD)) < 0) {
			fprintf(stderr, "pc-tty: failed to register device %s\n", _PATH_KBD);
		}
	} while (0);

	/* Mouse initialization */
	if ((err = _ttypc_kbd_mouse_init(ttypc)) == EOK) {
		has_mouse = true;
		do {
			/* Create virtual mouse device */
			if ((err = portCreate(&poolthr_args.mouse_port)) < 0) {
				fprintf(stderr, "pc-tty: failed to create mouse port\n");
				break;
			}
			oid.port = poolthr_args.mouse_port;
			oid.id = 0;
			if ((err = create_dev(&oid, _PATH_MOUSE)) < 0) {
				fprintf(stderr, "pc-tty: failed to register device %s\n", _PATH_MOUSE);
			}
		} while (0);
	}

	do {
		if ((err = mutexCreate(&ttypc->klock)) < 0)
			break;

		if ((err = condCreate(&ttypc->kmcond)) < 0)
			break;

		if ((err = event_queue_init(&ttypc->keq, 32)) < 0)
			break;

		if ((err = event_queue_init(&ttypc->meq, 256)) < 0)
			break;

		/* Attach KIRQ1 (kbd event) interrupt handle */
		if ((err = interrupt((ttypc->kirq = 1), ttypc_kbd_interrupt, ttypc, ttypc->kmcond, &ttypc->kinth)) < 0)
			break;

		/* Attach KIRQ12 (mouse event) interrupt handle */
		if ((err = interrupt((ttypc->mirq = 12), ttypc_kbd_interrupt, ttypc, ttypc->kmcond, &ttypc->minth)) < 0)
			break;

		poolthr_args.ttypc = ttypc;
		ctlthr_args.ttypc = ttypc;
		ctlthr_args.kcond = poolthr_args.kcond;

		if ((err = mutexCreate(&poolthr_args.kplock)) < 0)
			break;

		if ((err = condCreate(&poolthr_args.kcond)) < 0)
			break;

		/* Launch keyboard/mouse control thread */
		if ((err = beginthread(ttypc_kbd_ctlthr, 1, ttypc->kstack, sizeof(ttypc->kstack), &ctlthr_args)) < 0)
			break;

		/* Launch keyboard pool thread */
		if ((err = beginthread(ttypc_kbd_poolthr, 1, ttypc->kpstack, sizeof(ttypc->kpstack), &poolthr_args)) < 0)
			break;

		if (has_mouse) {
			/* Launch mouse pool thread */
			if ((err = beginthread(ttypc_kbd_mouse_poolthr, 1, ttypc->mpstack, sizeof(ttypc->mpstack), &poolthr_args)) < 0)
				break;
		}
	} while (0);

	if (err < 0) {
		resourceDestroy(ttypc->klock);
		resourceDestroy(ttypc->kmcond);
		event_queue_destroy(ttypc->keq);
		event_queue_destroy(ttypc->meq);
		resourceDestroy(ttypc->kinth);
		resourceDestroy(ttypc->minth);
		resourceDestroy(poolthr_args.kplock);
		resourceDestroy(poolthr_args.kcond);
		return err;
	}

	return EOK;
}
