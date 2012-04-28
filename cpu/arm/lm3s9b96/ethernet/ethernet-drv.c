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

#define DEBUG 0
#include "net/uip-debug.h"

//#define ETHERNET_INTERFACE_ID IF_FALLBACK
#define ETHERNET_INTERFACE_ID 0

#if UIP_CONF_LLH_LEN == 0
#define ETHERNET_LLH_LEN 14
uint8_t ll_header[ETHERNET_LLH_LEN];
#define BUF ((struct uip_eth_hdr *)&ll_header[0])
#else
#define BUF ((struct uip_eth_hdr *)&uip_buf[0])
#endif

#define IPBUF ((struct uip_tcpip_hdr *)&uip_buf[UIP_LLH_LEN])

PROCESS(ethernet_process, "ETHERNET driver");

uint8_t eth_mac_addr[6];
static void (* input_callback)(void) = NULL;
/*---------------------------------------------------------------------------*/
void
ethernet_set_input_callback(void (*c)(void))
{
  input_callback = c;
}

/*---------------------------------------------------------------------------*/
uint8_t
ethernet_output(uip_lladdr_t *lladdr)
{
	PRINTF("ETHERNET send: %d bytes\n",uip_len);
	if (lladdr == NULL) {
		(&BUF->dest)->addr[0] = 0x33;
		(&BUF->dest)->addr[1] = 0x33;
		(&BUF->dest)->addr[2] = IPBUF->destipaddr.u8[12];
		(&BUF->dest)->addr[3] = IPBUF->destipaddr.u8[13];
		(&BUF->dest)->addr[4] = IPBUF->destipaddr.u8[14];
		(&BUF->dest)->addr[5] = IPBUF->destipaddr.u8[15];
	}
	else {
		memcpy(&BUF->dest, lladdr, uip_ds6_if.lladdr_len);
	}
	memcpy(&BUF->src, &uip_lladdr[ETHERNET_INTERFACE_ID].addr, uip_ds6_if.lladdr_len);
	BUF->type = UIP_HTONS(UIP_ETHTYPE_IPV6);
	uip_len += sizeof(struct uip_eth_hdr);
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
	loadUserRegs(userReg);
	eth_mac_addr[0] = userReg[0];
	eth_mac_addr[1] = userReg[1];
	eth_mac_addr[2] = userReg[2];
	eth_mac_addr[3] = userReg[4];
	eth_mac_addr[4] = userReg[5];
	eth_mac_addr[5] = userReg[6];

	//uip_ds6_if[ETHERNET_INTERFACE_ID].lladdr_len = 6;
	uip_lladdr.addr[0] = eth_mac_addr[0];
	uip_lladdr.addr[1] = eth_mac_addr[1];
	uip_lladdr.addr[2] = eth_mac_addr[2];
	uip_lladdr.addr[3] = eth_mac_addr[3];
	uip_lladdr.addr[4] = eth_mac_addr[4];
	uip_lladdr.addr[5] = eth_mac_addr[5];

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
	//MAP_EthernetConfigSet(ETH_BASE, (ETH_CFG_TX_DPLXEN | ETH_CFG_TX_PADEN | ETH_CFG_TX_CRCEN | ETH_CFG_RX_AMULEN | ETH_CFG_RX_BADCRCDIS));
	MAP_EthernetConfigSet(ETH_BASE, (ETH_CFG_TX_DPLXEN | ETH_CFG_TX_PADEN | ETH_CFG_TX_CRCEN | ETH_CFG_RX_AMULEN | ETH_CFG_RX_BADCRCDIS));
	MAP_EthernetMACAddrSet(ETH_BASE, eth_mac_addr);

	MAP_EthernetEnable(ETH_BASE);

	PRINTF("FALLBACK MAC %x:%x:%x:%x:%x:%x\n",uip_lladdr[ETHERNET_INTERFACE_ID].addr[0],uip_lladdr[ETHERNET_INTERFACE_ID].addr[1],uip_lladdr[ETHERNET_INTERFACE_ID].addr[2],uip_lladdr[ETHERNET_INTERFACE_ID].addr[3],uip_lladdr[ETHERNET_INTERFACE_ID].addr[4],uip_lladdr[ETHERNET_INTERFACE_ID].addr[5]);
}

/*---------------------------------------------------------------------------*/
static void
pollhandler(void)
{
  process_poll(&ethernet_process);

  uip_len = MAP_EthernetPacketGetNonBlocking(ETH_BASE, uip_buf, UIP_BUFSIZE);
  if (uip_len > 0) {

		#if UIP_CONF_LLH_LEN == 0
  		/* Strip header from link local packet and put in ll_header (alias for BUF) */
			int i;
			for (i = 0; i < uip_len; i++) {
				if (i < ETHERNET_LLH_LEN) {
					ll_header[i] = uip_buf[i];
				}
				else {
					uip_buf[i - ETHERNET_LLH_LEN] = uip_buf[i];
				}
			}
			uip_buf[uip_len - ETHERNET_LLH_LEN] = '\0';
		#endif

	  PRINTF("ETHERNET receive: %d bytes\n",uip_len);
		if (BUF->type == uip_htons(UIP_ETHTYPE_IPV6))
		{
			//uip_active_interface = ETHERNET_INTERFACE_ID;
			tcpip_input();
		}
		else
		{
			uip_len = 0;
		}
	}
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ethernet_process, ev, data)
{
  PROCESS_POLLHANDLER(pollhandler());

  PROCESS_BEGIN();

  tcpip_set_outputfunc(ETHERNET_INTERFACE_ID, ethernet_output);

  process_poll(&ethernet_process);
  
  PROCESS_WAIT_UNTIL(ev == PROCESS_EVENT_EXIT);

  ethernet_exit();

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
