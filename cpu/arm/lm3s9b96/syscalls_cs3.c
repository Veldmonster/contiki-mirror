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
 *  Filename: syscalls_cs3.c
 *  Created on: 22 Mar 2012
 *  Author: Anton Veldhuizen
 */

#include "lm3s9b96inc.h"

#include <driverlib/uart.h>

#include <sys/stat.h>
#include <string.h>

#include "dev/rs232.h"

static uint8_t stdout_rs232_port = RS232_PORT_0;

int _close(int file) {
  return 0;
}

int _fstat(int file, struct stat *st) {
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file) {
  return 1;
}

int _lseek(int file, int ptr, int dir) {
  return 0;
}

int _open(const char *name, int flags, int mode) {
	if (strcmp("uart0", name) == 0) {
		stdout_rs232_port = RS232_PORT_0;
	}
	else if (strcmp("uart1", name) == 0) {
		stdout_rs232_port = RS232_PORT_1;
	}
	rs232_init(stdout_rs232_port, 115200,
						(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

  return 1;
}

int _read(int file, char *ptr, int len) {
	return len;
}

static char *heap_end = 0;
caddr_t _sbrk(int incr) {
  extern char __cs3_heap_start; /* Defined by the linker */
  extern char __cs3_heap_end; /* Defined by the linker */
  char *prev_heap_end;
  if (heap_end == 0) {
    heap_end = &__cs3_heap_start;
  }
  prev_heap_end = heap_end;
  if (heap_end + incr > &__cs3_heap_end) {
    /* Heap and stack collision */
    return (caddr_t)0;
  }
  heap_end += incr;
  return (caddr_t) prev_heap_end;
}

int _write(int file, char *ptr, int len) {
	int i;
	for (i = 0; i < len; i++) {
#if ADD_CARRAGE_RETURNS_TO_SERIAL_OUTPUT
		if(*ptr=='\n') {
			rs232_send(stdout_rs232_port, '\r');
		}
		if(*ptr=='\r') {
			ptr++;
		}
		else {
			rs232_send(stdout_rs232_port, *ptr++);
		}
#else
    rs232_send(stdout_rs232_port, *ptr++);
#endif
  }

  return len;
}

//int _write(int file, char *ptr, int len) {
//  int i;
//  for (i = 0; i < len; i++) {
//#if ADD_CARRAGE_RETURNS_TO_SERIAL_OUTPUT
//		if(*ptr++ == '\n') {
//			rs232_send(stdout_rs232_port, '\r');
//		}
//		if(*ptr++ != '\r') {
//			char c1 = *ptr;
//			unsigned char c2 = *ptr;
//			rs232_send(stdout_rs232_port, *ptr++);
//		}
//#else
//		rs232_send(stdout_rs232_port, *ptr++);
//#endif
//	}
//  return len;
//}
