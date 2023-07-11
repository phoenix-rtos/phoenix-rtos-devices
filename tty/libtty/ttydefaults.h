/*-
 * Copyright (c) 1982, 1986, 1993
 *	The Regents of the University of California.  All rights reserved.
 * (c) UNIX System Laboratories, Inc.
 * All or some portions of this file are derived from material licensed
 * to the University of California by American Telephone and Telegraph
 * Co. or Unix System Laboratories, Inc. and are reproduced herein with
 * the permission of UNIX System Laboratories, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)ttydefaults.h	8.4 (Berkeley) 1/21/94
 */

/*
 * System wide defaults for terminal state.  Linux version.
 */
#ifndef _TTYDEFAULTS_H_
#define _TTYDEFAULTS_H_

/*
 * Defaults on "first" open.
 */
#define TTYDEF_IFLAG (BRKINT | ISTRIP | ICRNL | IMAXBEL)
#define TTYDEF_OFLAG (OPOST | ONLCR | CR0 | NL0 | BS0 | TAB3 | VT0 | FF0)
#define TTYDEF_LFLAG (ECHO | ICANON | ISIG | IEXTEN | ECHOE | ECHOK | ECHOCTL)
#define TTYDEF_CFLAG (CREAD | CS8 | CLOCAL)
#define TTYDEF_SPEED (B115200)

/*
 * Control Character Defaults
 */
#define CTRL(x) (x & 037)
#define CEOF    CTRL('d')
#ifdef _POSIX_VDISABLE
#define CEOL _POSIX_VDISABLE
#else
#define CEOL '\0' /* XXX avoid _POSIX_VDISABLE */
#endif
#define CERASE  0177
#define CERASE2 CTRL('H')
#define CINTR   CTRL('c')
#ifdef _POSIX_VDISABLE
#define CSTATUS _POSIX_VDISABLE
#else
#define CSTATUS '\0' /* XXX avoid _POSIX_VDISABLE */
#endif
#define CKILL    CTRL('u')
#define CMIN     1
#define CQUIT    034 /* FS, ^\ */
#define CSUSP    CTRL('z')
#define CTIME    0
#define CDSUSP   CTRL('y')
#define CSTART   CTRL('q')
#define CSTOP    CTRL('s')
#define CLNEXT   CTRL('v')
#define CDISCARD CTRL('o')
#define CWERASE  CTRL('w')
#define CREPRINT CTRL('r')
#define CEOT     CEOF
/* compat */
#define CBRK   CEOL
#define CRPRNT CREPRINT
#define CFLUSH CDISCARD

/* Characters that cannot be modified through c_cc. */
#define CTAB '\t'
#define CNL  '\n'
#define CCR  '\r'


/* PROTECTED INCLUSION ENDS HERE */
#endif /* !_TTYDEFAULTS_H_ */

/*
 * #define TTYDEFCHARS to include an array of default control characters.
 */
#ifdef TTYDEFCHARS
cc_t ttydefchars[NCCS] = {
	CINTR, CQUIT, CERASE, CKILL, CEOF, CTIME, CMIN,
	CSTART, CSTOP, CSUSP, CEOL, CREPRINT, CDISCARD,
	CWERASE, CLNEXT, CERASE2, CEOL
};
#undef TTYDEFCHARS
#endif
