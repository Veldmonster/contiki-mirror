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
 *  Filename: flash.c
 *  Created on: 06 Jan 2012
 *  Author: Anton Veldhuizen
 */

#include "dev/flash.h"

//#include <avr/boot.h>
#include <inttypes.h>
//#include <avr/interrupt.h>
//#include <avr/pgmspace.h>

/*---------------------------------------------------------------------------*/
/*
 * The following code was taken from the avr-libc manual:
 */
void
flash_write_page(uint32_t page, uint8_t *buf)
{
//  uint16_t i;
//  uint8_t sreg;
//
//  /* Disable interrupts. */
//
//  sreg = SREG;
//  cli();
//
//  eeprom_busy_wait();
//
//  boot_page_erase(page);
//  boot_spm_busy_wait();      /* Wait until the memory is erased. */
//
//  for(i = 0; i < SPM_PAGESIZE; i += 2) {
//    /* Set up little-endian word. */
//
//    uint16_t w = *buf++;
//    w += (*buf++) << 8;
//
//    boot_page_fill(page + i, w);
//  }
//
//  boot_page_write(page);      /* Store buffer in flash page. */
//  boot_spm_busy_wait();       /* Wait until the memory is written. */
//
//  /* Reenable RWW-section again. We need this if we want to jump back
//   * to the application after bootloading. */
//
//  boot_rww_enable();
//
//  /* Re-enable interrupts (if they were ever enabled). */
//
//  SREG = sreg;
}
/*---------------------------------------------------------------------------*/
