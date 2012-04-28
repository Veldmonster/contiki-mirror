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
 *  Filename: memory-nv.c
 *  Created on: 09 Jan 2012
 *  Author: Anton Veldhuizen
 */

#include "lm3s9b96inc.h"

#include <driverlib/flash.h>

#include "lm3s9b96def.h"

void
loadUserRegs(uint8_t *mac_address)
{
	unsigned long userReg0 = 0xffffffff;
	unsigned long userReg1 = 0xffffffff;
	if (MAP_FlashUserGet(&userReg0, &userReg1) == 0)
	{
		mac_address[0] = (uint8_t)(userReg0);
		mac_address[1] = (uint8_t)(userReg0 >> 8);
		mac_address[2] = (uint8_t)(userReg0 >> 16);
		mac_address[3] = (uint8_t)(userReg0 >> 24);
		mac_address[4] = (uint8_t)(userReg1);
		mac_address[5] = (uint8_t)(userReg1 >> 8);
		mac_address[6] = (uint8_t)(userReg1 >> 16);
		mac_address[7] = (uint8_t)(userReg1 >> 24);
	}
}

void
loadRadioMACAddress(uint8_t *mac_address)
{
	// TODO: Anton - Convert ethernet address or read from flash?
	mac_address[0] = 0x99;
	mac_address[1] = 0xaa;
	mac_address[2] = 0x99;
	mac_address[3] = 0xaa;
	mac_address[4] = 0x99;
	mac_address[5] = 0xaa;
	mac_address[6] = 0x99;
	mac_address[7] = 0xaa;
}
