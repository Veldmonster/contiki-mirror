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
 * for the AVR Zigbit and Microchips ETHERNET.
 * @(#)$
 */

/*
 *  Filename: ethernet-drv.c
 *  Created on: 06 Jan 2012
 *  Author: Anton Veldhuizen
 */

#include "lm3s9b96inc.h"

#include <string.h>
#include <driverlib/ethernet.h>

#include "memory-nv.h"
#include "ethernet/ethernet-drv.h"
#include "contiki-net.h"
#include "net/uip-neighbor.h"
#include "net/uip.h"
#include "net/uip-fw.h"
#include "net/uip_arp.h"

#define DEBUG 0
#include "net/uip-debug.h"

#define BUF ((struct uip_eth_hdr *)&uip_buf[0])

PROCESS(ethernet_process, "ETHERNET driver");

struct uip_eth_addr eth_mac_addr;
static void (* input_callback)(void) = NULL;

/*---------------------------------------------------------------------------*/
void
ethernet_set_input_callback(void (*c)(void))
{
  input_callback = c;
}

/*---------------------------------------------------------------------------*/
uint8_t
ethernet_active(void)
{
	return !MAP_EthernetSpaceAvail(ETH_BASE);
}

uint8_t
ethernet_output(void)
{
	PRINTF("ETHERNET send: %d bytes\n",uip_len);
  uip_arp_out();
	MAP_EthernetPacketPut(ETH_BASE, uip_buf, uip_len);

  return 0;
}

/*
 * Placeholder - switching off ethernet chip wasn't yet considered
 */
void ethernet_exit(void)
{}


/*
 * Wrapper for lowlevel ethernet init code
 * in current configuration it reads the Ethernet driver MAC address
 * from EEPROM memory
 */
void ethernet_init()
{
	uint8_t userReg[8];
	load_user_regs(userReg);
	eth_mac_addr.addr[0] = userReg[0];
	eth_mac_addr.addr[1] = userReg[1];
	eth_mac_addr.addr[2] = userReg[2];
	eth_mac_addr.addr[3] = userReg[4];
	eth_mac_addr.addr[4] = userReg[5];
	eth_mac_addr.addr[5] = userReg[6];

  uip_setethaddr(eth_mac_addr);

	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ETH);
	MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_ETH);

	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	MAP_GPIOPinConfigure(GPIO_PF2_LED1);
	MAP_GPIOPinConfigure(GPIO_PF3_LED0);
	MAP_GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);

  /* Initialize the Ethernet controller for operation */
	MAP_EthernetInitExpClk(ETH_BASE, MAP_SysCtlClockGet());

  /*
   * Configure the Ethernet controller for normal operation
   * Enable TX Duplex Mode
   * Enable TX Padding
  */
	MAP_EthernetConfigSet(ETH_BASE, (ETH_CFG_TX_DPLXEN | ETH_CFG_TX_PADEN | ETH_CFG_TX_CRCEN | ETH_CFG_RX_AMULEN | ETH_CFG_RX_BADCRCDIS));
	MAP_EthernetMACAddrSet(ETH_BASE, eth_mac_addr.addr);

	MAP_EthernetEnable(ETH_BASE);
}

/*---------------------------------------------------------------------------*/
static void
pollhandler(void)
{
  process_poll(&ethernet_process);

  uip_len = MAP_EthernetPacketGetNonBlocking(ETH_BASE, uip_buf, UIP_BUFSIZE);
  if (uip_len > 0) {
	  PRINTF("ETHERNET receive: %d bytes\n", uip_len);
#if UIP_CONF_IPV6
    if (BUF->type == uip_htons(UIP_ETHTYPE_IPV6)) {
      uip_neighbor_add(&IPBUF->srcipaddr, &BUF->src);
      tcpip_input();
    }
    else
#endif /* UIP_CONF_IPV6 */
    if (BUF->type == uip_htons(UIP_ETHTYPE_IP)) {
      tcpip_input();
    }
    else if (BUF->type == uip_htons(UIP_ETHTYPE_ARP)) {
      uip_arp_arpin();
      /* If the above function invocation resulted in data that
				 should be sent out on the network, the global variable
				 uip_len is set to a value > 0. */
      if (uip_len > 0) {
      	MAP_EthernetPacketPut(ETH_BASE, uip_buf, uip_len);
      }
    }
	}
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ethernet_process, ev, data)
{
  PROCESS_POLLHANDLER(pollhandler());

  PROCESS_BEGIN();

  uip_arp_init();

  tcpip_set_outputfunc(ethernet_output);

  process_poll(&ethernet_process);
  
  PROCESS_WAIT_UNTIL(ev == PROCESS_EVENT_EXIT);

  ethernet_exit();

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
