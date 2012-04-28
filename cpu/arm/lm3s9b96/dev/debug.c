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
 *  Filename: debug.c
 *  Created on: 06 Jan 2012
 *  Author: Anton Veldhuizen
 */

#include "lm3s9b96inc.h"

#include <driverlib/uart.h>
#include <utils/uartstdio.h>

#include "compiler.h"
#include "delay.h"

/*-----------------------------------------------------------------------------------*/
static void
delay(void)
{
  unsigned char i;
  for(i = 0; i < 1; ++i) {
    delay_1us(10000UL);
  }
}
/*-----------------------------------------------------------------------------------*/
static char buffer[40];
static prog_char hextab[] =
  {'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f'};
/*-----------------------------------------------------------------------------------*/
static void
print_buffer(unsigned char len)
{
  unsigned char i;

  for(i = 0; i < len; ++i) {
    delay();
    MAP_UARTCharPut(UART0_BASE, (unsigned char)buffer[i]));
  }
}
/*-----------------------------------------------------------------------------------*/
void
debug_print8(unsigned char v)
{
  /*  buffer[0] = v / 100 + '0';
  buffer[1] = (v / 10) % 10 + '0';
  buffer[2] = v % 10 + '0';
  buffer[3] = ' ';
  buffer[4] = PRG_RDB(hextab + (v >> 4));
  buffer[5] = PRG_RDB(hextab + (v & 0x0f));    
  buffer[6] = '\n';
  print_buffer(7);*/
}
/*-----------------------------------------------------------------------------------*/
void
debug_print16(unsigned short v)
{
  /*  buffer[0] = v / 10000 + '0';
  buffer[1] = (v / 1000) % 10 + '0';
  buffer[2] = (v / 100) % 10 + '0';
  buffer[3] = (v / 10) % 10 + '0';
  buffer[4] = v % 10 + '0';
  buffer[5] = ' ';
  buffer[6] = PRG_RDB(hextab + ((v & 0xf000) >> 12));
  buffer[7] = PRG_RDB(hextab + ((v & 0x0f00) >> 8));
  buffer[8] = PRG_RDB(hextab + ((v & 0xf0) >> 4));
  buffer[9] = PRG_RDB(hextab + (v & 0x0f));    
  buffer[10] = '\n';
  print_buffer(11);*/
}
/*-----------------------------------------------------------------------------------*/
void
debug_print(prog_char *str)
{
  /*  unsigned char i;

  for(i = 0; PRG_RDB(str + i) != 0; ++i) {
    buffer[i] = PRG_RDB(str + i);
  }
  print_buffer(i);*/
}
/*-----------------------------------------------------------------------------------*/
