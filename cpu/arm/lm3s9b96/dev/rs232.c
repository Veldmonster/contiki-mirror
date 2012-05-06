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
 *  Filename: rs232.c
 *  Created on: 06 Jan 2012
 *  Author: Anton Veldhuizen
 */

#include "lm3s9b96inc.h"

#include <stdarg.h>
#include <stdio.h>
#include <driverlib/interrupt.h>
#include <driverlib/uart.h>

#include "contiki-conf.h"
#include "contiki.h"

#include "dev/slip.h"
#include "dev/rs232.h"

typedef struct {
	int initialized;
	int (* input_handler)(unsigned char);
} rs232_t;

static rs232_t rs232_ports[2] = {
	{ // UART0
		0,
    NULL
  },
  { // UART1
		0,
    NULL
  }
};

static unsigned long uartBase[2] = { UART0_BASE, UART1_BASE };
/*---------------------------------------------------------------------------*/

void IsrUart0(void)
{
  unsigned char c;
  unsigned long ulStatus;

  ulStatus = MAP_UARTIntStatus(UART0_BASE, true);

  MAP_UARTIntClear(UART0_BASE, ulStatus);

  while(MAP_UARTCharsAvail(UART0_BASE))
  {
  	c = MAP_UARTCharGetNonBlocking(UART0_BASE);

    if(rs232_ports[RS232_PORT_0].input_handler != NULL) {
      rs232_ports[RS232_PORT_0].input_handler(c);
    }
  }
}

void IsrUart1(void)
{
  unsigned char c;
  unsigned long ulStatus;

  ulStatus = MAP_UARTIntStatus(UART1_BASE, true);

  MAP_UARTIntClear(UART1_BASE, ulStatus);

  while(MAP_UARTCharsAvail(UART1_BASE))
  {
  	c = MAP_UARTCharGetNonBlocking(UART1_BASE);

    if(rs232_ports[RS232_PORT_1].input_handler != NULL) {
      rs232_ports[RS232_PORT_1].input_handler(c);
    }
  }
}

/*---------------------------------------------------------------------------*/
void
rs232_init (uint8_t port, uint32_t bd, uint32_t ffmt)
{
	if (rs232_ports[port].initialized == 0) {
		rs232_ports[port].initialized = 1;

		if (ffmt == 0) {
			ffmt = UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE;
		}

		if (RS232_PORT_0 == port && MAP_SysCtlPeripheralPresent(SYSCTL_PERIPH_UART0))
		{
			MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
			MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
			MAP_UARTConfigSetExpClk(UART0_BASE, MAP_SysCtlClockGet(), bd, ffmt);
			MAP_IntEnable(INT_UART0);
			MAP_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
			MAP_UARTEnable(UART0_BASE);

			rs232_ports[port].input_handler = NULL;
		}
		else if(RS232_PORT_1 == port && MAP_SysCtlPeripheralPresent(SYSCTL_PERIPH_UART1))
		{
			MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
			MAP_GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1); // UART1
			MAP_UARTConfigSetExpClk(UART1_BASE, MAP_SysCtlClockGet(), bd, ffmt);
			MAP_IntEnable(INT_UART1);
			MAP_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
			MAP_UARTEnable(UART1_BASE);

			rs232_ports[port].input_handler = NULL;
		}
	}
}

/*---------------------------------------------------------------------------*/
void
rs232_print(uint8_t port, char *buf)
{
  while(*buf) {
#if ADD_CARRAGE_RETURNS_TO_SERIAL_OUTPUT
	if(*buf=='\n') {
		rs232_send(port, '\r');
	}
	if(*buf=='\r') {
		buf++;
	}
	else {
		rs232_send(port, *buf++);
	}
#else
    rs232_send(port, *buf++);
#endif
  }
}
/*---------------------------------------------------------------------------*/
void
rs232_printf(uint8_t port, const char *fmt, ...)
{
  va_list ap;
  static char buf[RS232_PRINTF_BUFFER_LENGTH];

  va_start (ap, fmt);
  vsnprintf (buf, RS232_PRINTF_BUFFER_LENGTH, fmt, ap);
  va_end(ap);

  rs232_print (port, buf);
}
/*---------------------------------------------------------------------------*/
void
rs232_send(uint8_t port, unsigned char c)
{
	/* This function is also invoked by syscalls */
	/* UARTCharPut will block until there is space on the FIFO */
	MAP_UARTCharPut(uartBase[port], c);
}
/*---------------------------------------------------------------------------*/
void
rs232_set_input(uint8_t port, int (*f)(unsigned char))
{
  rs232_ports[port].input_handler = f;
}
/*---------------------------------------------------------------------------*/
void
slip_arch_writeb(unsigned char c)
{
//  rs232_send(SLIP_PORT, c);
}

/*---------------------------------------------------------------------------*/
void rs232_redirect_stdout (uint8_t port) {
	/* freopen(...) will invoke _open(...) in syscalls */
	if (port == RS232_PORT_0) {
		freopen("uart0", "w", stdout);
	}
	else if (port == RS232_PORT_1) {
		freopen("uart1", "w", stdout);
	}
	setvbuf(stdout, (char *) NULL, _IONBF, 0);
}
