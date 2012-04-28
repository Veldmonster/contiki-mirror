/*
 * Copyright (c) 2011, Universal Concepts CC, South Africa
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 * This port targets the Stellaris ARM, Cortex M3, LM3S9xxxx
 * which is based on siskin's, multiple-netif branch
 * for the AVR Zigbit and Microchips ENC28J60.
 * @(#)$
 */

/*
 *  Filename: mtarch.c
 *  Created on: 12 Feb 2012
 *  Author: Anton Veldhuizen
 */

#include "lm3s9b96inc.h"

#include <stdio.h>
#include "sys/mt.h"
#include "dev/rs232.h"

static volatile unsigned long *prevsp;
static volatile unsigned long *nextsp;
static struct mtarch_thread *running;

/*--------------------------------------------------------------------------*/
void IsrPendSV(void) __attribute__ (( naked ));
void IsrPendSV(void) {
	/* Save registers (that was not saved by hardware) on the current stack pointer (MSP)*/
	__asm__ volatile("push {r4-r11}");

	/* Context switch */
	nextsp = running->sp;
	/* Save current MSP as previous stack pointer */
	__asm__ volatile ("mrs %0, msp" : "=r" (prevsp) );

	running->sp = prevsp;
	/* Set MSP to the next stack pointer */
	__asm__ volatile ("msr msp, %0" : : "r" (nextsp) );

	/* Restore registers (that won't be done by hardware) on the current stack pointer (MSP) */
	__asm__ volatile("pop {r4-r11}");

	/* Return using MSP (indicated by last 4 bits of r14 or LR) */
	__asm__ volatile("orr r14, #0x9");
	__asm__ volatile("bx r14");
}
/*--------------------------------------------------------------------------*/
void mtarch_init(void) {
}
/*--------------------------------------------------------------------------*/
static void mtarch_wrapper(void) {
	/* Call thread function with argument */
	((void(*)(void *)) running->function)((void*) running->data);
}
/*--------------------------------------------------------------------------*/
void mtarch_start(struct mtarch_thread *t, void(*function)(void *), void *data) {
	int i;
	for (i = 0; i < MTARCH_STACKSIZE; ++i) {
		t->stack[i] = i;
	}

	t->sp = &t->stack[MTARCH_STACKSIZE - 1];

	*t->sp = 0x01000000; /* xPSR */
	t->sp--;
	*t->sp = (unsigned long) mtarch_wrapper; /* PC */
	t->sp--;
	*t->sp = 0; /* LR */
	t->sp -= 5; /* R12, R3, R2 and R1. */
	*t->sp = 0; /* R0 */
	t->sp -= 8; /* R11, R10, R9, R8, R7, R6, R5 and R4. */

	/* Store function and argument (used in mtarch_wrapper) */
	t->data = data;
	t->function = function;
}
/*--------------------------------------------------------------------------*/
void mtarch_exec(struct mtarch_thread *t) {
	running = t;

	// Trigger PendSV ISR
	*((unsigned long volatile *) NVIC_INT_CTRL) = NVIC_INT_CTRL_PEND_SV;
}
/*--------------------------------------------------------------------------*/
void mtarch_remove(void) {

}
/*--------------------------------------------------------------------------*/
void mtarch_yield(void) {
	// Trigger PendSV ISR
	*((unsigned long volatile *) NVIC_INT_CTRL) = NVIC_INT_CTRL_PEND_SV;
}
/*--------------------------------------------------------------------------*/
void mtarch_pstop(void) {

}
/*--------------------------------------------------------------------------*/
void mtarch_pstart(void) {

}
/*--------------------------------------------------------------------------*/
void mtarch_stop(struct mtarch_thread *thread) {

}
/*--------------------------------------------------------------------------*/
int mtarch_stack_usage(struct mt_thread *t) {
	unsigned long i;

	for (i = 0; i < MTARCH_STACKSIZE; ++i) {
		if (t->thread.stack[i] != i) {
			return MTARCH_STACKSIZE - i;
		}
	}

	return MTARCH_STACKSIZE;
}
/*--------------------------------------------------------------------------*/
