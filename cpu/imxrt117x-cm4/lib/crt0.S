/*
 * Phoenix-RTOS
 *
 * i.MX RT117x Cortex-M4 core start code
 *
 * Copyright 2021 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#define __ASSEMBLY__
#include "interrupt.h"

.syntax unified
.cpu cortex-m4

.section .init, "x"
.org 0x100, 0xff

.globl _init_vectors
.type _init_vectors, %object
_init_vectors:
.word _end + 4096
.word _start

.word _exception    /* NMI */
.word _exception    /* HardFault */
.word _exception    /* MemMgtFault */
.word _exception    /* BusFault */
.word _exception    /* UsageFault */
.word 0
.word 0
.word 0
.word 0
.word _svc          /* SVC */
.word _exception    /* Debug */
.word 0
.word _pendsv       /* PendSV */
.word _systick      /* Systick */

.rept SIZE_INTERRUPTS
.word _interrupt
.endr
.size _init_vectors, .-_init_vectors

.thumb
.thumb_func


.macro saveCtx
	vstmdb sp!, {s16-s31}
	mrs r0, ipsr
	mrs r3, psp
	sub r1, sp, #48
	ldr r2, =0xe000ef38
	ldr r2, [r2]
	stmdb sp!, {r1-r11, lr}
.endm


.macro restoreCtx
	ldmia sp!, {r1-r11, lr}
	vldmia sp!, {s16-s31}

	ldr r1, =0xe000ef38
	str r2, [r1]

	msr psp, r3

	/* Check if we're returning to userspace */
	and r1, lr, #4
	ror r1, r1, #2
	orr r1, r1, #(1 << 2)
	msr control, r1
.endm


.globl _start
.type _start, %function
_start:
	cpsid if

	/* Enable FPU */
	ldr r0, =0xe000ed88
	ldr r1, [r0]
	orr r1, r1, #(0xf << 20)
	str r1, [r0]

	/* Enable FPU ctx */
	mrs r0, control
	orr r0, r0, #(1 << 2)
	msr control, r0

	/* Init vector table pointer */
	ldr r0, =0xe000ed08
	ldr r1, =_init_vectors
	str r1, [r0]
	ldr r0, [r1]
	bic r0, 7
	msr msp, r0

	ldr r0, =_startc
	bx r0
.size _start, .-_start
.ltorg



.globl _exception
.type _exception, %function
_exception:
	saveCtx

	mrs r0, ipsr

	blx _exceptionHandler

	restoreCtx

	dmb
	bx lr
.size _exception, .-_exception
.ltorg


.globl _systick
.type _systick, %function
_systick:
	saveCtx

	blx _systickHandler

	restoreCtx

	dmb
	bx lr
.size _systick, .-_pendsv
.ltorg


.globl _svc
.type _svc, %function
_svc:
	push {r12, lr} /* r12 just for stack aligment */
	blx _syscallHandler
	pop {r12, lr}
	bx lr
.size _svc, .-_svc
.ltorg


.globl _pendsv
.type _pendsv, %function
_pendsv:
	saveCtx

	blx _pendsvHandler

	restoreCtx

	dmb
	bx lr
.size _pendsv, .-_pendsv
.ltorg

.extern interrupt_dispatch

.globl _interrupt
.type _interrupt, %function
_interrupt:
	saveCtx

	mrs r0, ipsr

	blx interrupt_dispatch

	restoreCtx

	dmb
	bx lr
.size _interrupt, .-_interrupt
.ltorg
