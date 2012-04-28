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
 *  Filename: contiki-arm-dk-lm3s9b96-main.c
 *  Created on: 06 Jan 2012
 *  Author: Anton Veldhuizen
 */
#include "lm3s9b96inc.h"

#include <stdio.h>
#include <string.h>
#include <driverlib/flash.h>
#include <driverlib/gpio.h>
#include <driverlib/interrupt.h>
#include <driverlib/sysctl.h>
#include <driverlib/systick.h>
#include <driverlib/udma.h>
#include <driverlib/uart.h>
#include <boards/dk-lm3s9b96/drivers/set_pinout.h>

#include "memory-nv.h"
#include "mtarch.h"
#include <dev/watchdog.h>
#include <dev/delay.h>

#include "lib/mmem.h"
#include "loader/symbols-def.h"
#include "loader/symtab.h"

#define ANNOUNCE_BOOT 1    //adds about 600 bytes to program size
#define DEBUG 1
#if DEBUG
#define PRINTF(FORMAT,args...) printf(FORMAT,##args)
#define PRINTSHORT(FORMAT,args...) printf(FORMAT,##args)
#else
#define PRINTF(...)
#define PRINTSHORT(...)
#endif

#if RF230BB           //radio driver using contiki core mac
#include "radio/rf230bb/rf230bb.h"
#include "net/mac/frame802154.h"
#include "net/mac/framer-802154.h"
#include "net/sicslowpan.h"
#else                 //radio driver using Atmel/Cisco 802.15.4'ish MAC
#include <stdbool.h>
#include "mac.h"
#include "sicslowmac.h"
#include "sicslowpan.h"
#include "ieee-15-4-manager.h"
#endif /*RF230BB*/

#include "contiki.h"
#include "contiki-net.h"
#include "contiki-lib.h"

#include "dev/rs232.h"
#include "dev/serial-line.h"
#include "ethernet/ethernet-drv.h"

#include "sicslowmac.h"

void uip_log(char *msg) { PRINTF("%s",msg); }
	
#if RF230BB
//PROCINIT(&etimer_process, &tcpip_process );
#else
PROCINIT(&etimer_process, &mac_process, &tcpip_process );
#endif

/* Put default MAC address in EEPROM */
// TODO: Anton - Reinstate
//uint8_t mac_address[8] EEMEM = {0x99, 0xaa, 0x99, 0xaa, 0x99, 0xaa, 0x99, 0xaa};

/*
 *If compiled with -D DEBUG and ASSERT fails then, this routine will be called.
 *The error routine that is called if the driver library encounters an error.
*/
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

void
init_lowlevel(void)
{
	/* Set the system clock to run at 50MHz from the PLL. */
	MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
					   SYSCTL_XTAL_16MHZ);

  MAP_IntPrioritySet(FAULT_SYSTICK, 0x80);
  MAP_IntPrioritySet(INT_ETH, 0xC0);
  MAP_IntPrioritySet(FAULT_PENDSV, 0xE0); /* Lowest priority */
  MAP_IntPrioritySet(FAULT_SVCALL, 0xE0);

	/* Set the device pinout appropriately for this board. */
	PinoutSet();

	delay_init();

  watchdog_init();

  /* Initialize and set stdout */
  rs232_redirect_stdout(RS232_PORT_0);

  /* Clock */
  clock_init();

  /* rtimers needed for radio cycling */
  rtimer_init();

  /* Initialize process subsystem */
  process_init();

  /* etimers must be started before ctimer_init */
  process_start(&etimer_process, NULL);

  /* RF230BB is passed in as a CFLAGS argument Makefile.radio */
#if RF230BB

  ctimer_init();
  /* Start radio and radio receive process */
  NETSTACK_RADIO.init();
  //ethernet_init();
  //process_start(&ethernet_process, NULL);

  /* Set addresses BEFORE starting tcpip process */


  rimeaddr_t addr;
  memset(&addr, 0, sizeof(rimeaddr_t));
	uint8_t mac_address[8];
  loadRadioMACAddress((uint8_t *)&addr.u8);

#if UIP_CONF_IPV6
  memcpy(&uip_lladdr[IF_RADIO].addr, &addr.u8, 8);
#endif
  rf230_set_pan_addr(IEEE802154_PANID, 0, (uint8_t *)&addr.u8);
#ifdef CHANNEL_802_15_4
  rf230_set_channel(CHANNEL_802_15_4);
#else
  rf230_set_channel(26);
#endif

  rimeaddr_set_node_addr(&addr);

  PRINTF("MAC address %x:%x:%x:%x:%x:%x:%x:%x\n",addr.u8[0],addr.u8[1],addr.u8[2],addr.u8[3],addr.u8[4],addr.u8[5],addr.u8[6],addr.u8[7]);

  /* Initialize stack protocols */
  queuebuf_init();
  NETSTACK_RDC.init();
  NETSTACK_MAC.init();
  NETSTACK_NETWORK.init();

#if ANNOUNCE_BOOT
  printf("%s %s, channel %u",NETSTACK_MAC.name, NETSTACK_RDC.name,rf230_get_channel());
  if (NETSTACK_RDC.channel_check_interval) {//function pointer is zero for sicslowmac
    unsigned short tmp;
    tmp=CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval == 0 ? 1:\
                                   NETSTACK_RDC.channel_check_interval());
    if (tmp<65535) printf(", check rate %u Hz",tmp);
  }
  printf("\n");
#endif

#if UIP_CONF_ROUTER
#if ANNOUNCE_BOOT
  printf("Routing Enabled\n");
#endif
  //rime_init(rime_udp_init(NULL));
  //uip_router_register(&rimeroute);
#endif

  process_start(&tcpip_process, NULL);

#else
/* mac process must be started before tcpip process! */
  process_start(&mac_process, NULL);
  process_start(&tcpip_process, NULL);
#endif /*RF230BB*/
}

int
main(void)
{
  /* Initialize hardware and associated processes*/
  init_lowlevel();

  /* Autostart processes */
  autostart_start(autostart_processes);

  printf("\n********BOOTING CONTIKI*********\n");

  printf("System online.\n");

  /* Main scheduler loop */
  while(1) {
    process_run();
  }

  return 0;
}
