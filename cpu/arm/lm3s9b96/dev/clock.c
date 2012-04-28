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
 *  Filename: clock.c
 *  Created on: 06 Jan 2012
 *  Author: Anton Veldhuizen
 */


#include "lm3s9b96inc.h"

#include "sys/clock.h"
#include "sys/etimer.h"

static volatile clock_time_t count;
static volatile uint8_t scount;
volatile unsigned long seconds;
long sleepseconds;

/*
  CLOCK_SECOND is the number of ticks per second.
  It is defined through CONF_CLOCK_SECOND in the contiki-conf.h for each platform.
  The usual AVR default is ~125 ticks per second, counting a prescaler the CPU clock
  using the 8 bit timer0.
  
  As clock_time_t is an unsigned 16 bit data type, intervals up to 524 seconds
  can be measured with 8 millisecond precision. 
  For longer intervals a 32 bit global is incremented every second. 
*/
/*---------------------------------------------------------------------------*/
/* This routine can be called to add seconds to the clock after a sleep
 * of an integral number of seconds.
 */
void clock_adjust_seconds(uint8_t howmany) {
   seconds += howmany;
   sleepseconds +=howmany;
}

/*---------------------------------------------------------------------------*/
void
IsrSysTick(void)
{
  count++;
  if(++scount == CLOCK_SECOND) {
    scount = 0;
    seconds++;
  }

  // gcc will save all registers on the stack if an external routine is called
  if(etimer_pending()) {
    etimer_request_poll();
  }
}

/*---------------------------------------------------------------------------*/
void
clock_init(void)
{
  //
  // Configure SysTick for a 125 ticks per second (8ms resolution) interrupt.
  //
  MAP_SysTickIntDisable();
  MAP_SysTickPeriodSet(MAP_SysCtlClockGet() / 125);
  MAP_SysTickEnable();
  MAP_SysTickIntEnable();
}

/*---------------------------------------------------------------------------*/
clock_time_t
clock_time(void)
{
  clock_time_t tmp;
  do {
    tmp = count;
  } while(tmp != count);
  return tmp;
}
/*---------------------------------------------------------------------------*/
/**
 * Delay the CPU for a multiple of TODO
 */
void
clock_delay(unsigned int i)
{
	//MAP_SysCtlDelay(i);
}

/*---------------------------------------------------------------------------*/
/**
 * Wait for a multiple of 1 / 125 sec = 0.008 ms.
 */
void
clock_wait(clock_time_t t)
{
  clock_time_t endticks = clock_time() + t;
  if (sizeof(clock_time_t) == 1) {
    while ((signed char )(clock_time() - endticks) < 0) {;}
  } else if (sizeof(clock_time_t) == 2) {
    while ((signed short)(clock_time() - endticks) < 0) {;}
  } else {
    while ((signed long )(clock_time() - endticks) < 0) {;}
  }
}
/*---------------------------------------------------------------------------*/
void
clock_set_seconds(unsigned long sec)
{
    // TODO
}

unsigned long
clock_seconds(void)
{
  unsigned long tmp;
  do {
    tmp = seconds;
  } while(tmp != seconds);
  return tmp;
}
