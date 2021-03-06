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
 * which is based on the sky-ip example
 * @(#)$
 */

/*
 *  Filename: contiki-dk-lm3s9b96-main.c
 *  Created on: 30 April 2012
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

#include "contiki.h"
#include "mtarch.h"
#include "memory-nv.h"
#include "dev/watchdog.h"
#include "dev/delay.h"
#include "dev/leds.h"
#include "dev/rs232.h"
#include "ethernet/ethernet-drv.h"
#include "lib/random.h"
#include "radio/rf230bb/rf230bb.h"
#include "net/netstack.h"
#include "net/mac/frame802154.h"

#if WITH_UIP6
#include "net/uip-ds6.h"
#endif /* WITH_UIP6 */

#include "net/rime.h"

#include "node-id.h"
//#include "cfs-coffee-arch.h"
//#include "cfs/cfs-coffee.h"
#include "sys/autostart.h"
#include "sys/profile.h"

#if UIP_CONF_ROUTER

#ifndef UIP_ROUTER_MODULE
#ifdef UIP_CONF_ROUTER_MODULE
#define UIP_ROUTER_MODULE UIP_CONF_ROUTER_MODULE
#else /* UIP_CONF_ROUTER_MODULE */
#define UIP_ROUTER_MODULE rimeroute
#endif /* UIP_CONF_ROUTER_MODULE */
#endif /* UIP_ROUTER_MODULE */

extern const struct uip_router UIP_ROUTER_MODULE;
#endif /* UIP_CONF_ROUTER */

#if DCOSYNCH_CONF_ENABLED
static struct timer mgt_timer;
#endif
//extern int lm3s9b96_dco_required;

#ifndef WITH_UIP
#define WITH_UIP 0
#endif

#if WITH_UIP
#include "net/uip.h"
#include "net/uip-fw.h"
#include "net/uip-fw-drv.h"
#include "net/uip-over-mesh.h"
static struct uip_fw_netif ethif =
  {UIP_FW_NETIF(192,168,0,210, 255,255,255,255, ethernet_output)};
static struct uip_fw_netif meshif =
  {UIP_FW_NETIF(192,168,0,211, 255,255,0,0, uip_over_mesh_send)};

#endif /* WITH_UIP */

#define UIP_OVER_MESH_CHANNEL 8
#if WITH_UIP
static uint8_t is_gateway;
#endif /* WITH_UIP */

#ifdef EXPERIMENT_SETUP
#include "experiment-setup.h"
#endif

uint8_t mac_address[8];

void init_platform(void);

/*---------------------------------------------------------------------------*/
#if 0
int
force_float_inclusion()
{
  extern int __fixsfsi;
  extern int __floatsisf;
  extern int __mulsf3;
  extern int __subsf3;

  return __fixsfsi + __floatsisf + __mulsf3 + __subsf3;
}
#endif
/*---------------------------------------------------------------------------*/
void uip_log(char *msg) { puts(msg); }
/*---------------------------------------------------------------------------*/
#ifndef RF_CHANNEL
#define RF_CHANNEL              26
#endif
/*---------------------------------------------------------------------------*/
#if 0
void
force_inclusion(int d1, int d2)
{
  snprintf(NULL, 0, "%d", d1 % d2);
}
#endif
/*---------------------------------------------------------------------------*/
static void
set_rime_addr(void)
{
  rimeaddr_t addr;
  int i;

  memset(&addr, 0, sizeof(rimeaddr_t));
#if UIP_CONF_IPV6
  memcpy(addr.u8, mac_address, sizeof(addr.u8));
#else
  if(node_id == 0) {
    for(i = 0; i < sizeof(rimeaddr_t); ++i) {
      addr.u8[i] = mac_address[7 - i];
    }
  } else {
    addr.u8[0] = node_id & 0xff;
    addr.u8[1] = node_id >> 8;
  }
#endif
  rimeaddr_set_node_addr(&addr);
  printf("Rime started with address ");
  for(i = 0; i < sizeof(addr.u8) - 1; i++) {
    printf("%d.", addr.u8[i]);
  }
  printf("%d\n", addr.u8[i]);
}
/*---------------------------------------------------------------------------*/
static void
print_processes(struct process * const processes[])
{
  /*  const struct process * const * p = processes;*/
  printf("Starting");
  while(*processes != NULL) {
    printf(" '%s'", (*processes)->name);
    processes++;
  }
  putchar('\n');
}
/*--------------------------------------------------------------------------*/
#if WITH_UIP
static void
set_gateway(void)
{
  if(!is_gateway) {
    leds_on(LEDS_RED);
    printf("%d.%d: making myself the IP network gateway.\n\n",
	   rimeaddr_node_addr.u8[0], rimeaddr_node_addr.u8[1]);
    printf("IPv4 address of the gateway: %d.%d.%d.%d\n\n",
	   uip_ipaddr_to_quad(&uip_hostaddr));
    uip_over_mesh_set_gateway(&rimeaddr_node_addr);
    uip_over_mesh_make_announced_gateway();
    is_gateway = 1;
  }
}
#endif /* WITH_UIP */
/*---------------------------------------------------------------------------*/
#if WITH_TINYOS_AUTO_IDS
uint16_t TOS_NODE_ID = 0x1234; /* non-zero */
uint16_t TOS_LOCAL_ADDRESS = 0x1234; /* non-zero */
#endif /* WITH_TINYOS_AUTO_IDS */

//int
//main(int argc, char **argv)
int
main(void)
{
  /* Initialize hardware and associated processes*/
	/* Set the system clock to run at 50MHz from the PLL. */
	MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
					   SYSCTL_XTAL_16MHZ);

  MAP_IntPrioritySet(FAULT_SYSTICK, 0x80);
  MAP_IntPrioritySet(INT_ETH, 0xC0);
  MAP_IntPrioritySet(FAULT_PENDSV, 0xE0); /* Lowest priority */
  MAP_IntPrioritySet(FAULT_SVCALL, 0xE0);

	/* Set the device pinout appropriately for this board. */
	PinoutSet();

  /* Clock */
  clock_init();


  load_radio_mac_address(mac_address);

  /* Non-interrupt based (cpu busy) delay */
	delay_init();

	/* Prepare watchdog */
  watchdog_init();

  /* Initialize and set stdout */
  rs232_redirect_stdout(RS232_PORT_0);

  /* Initialize ethernet (replaces slip) */
  ethernet_init();

  /* rtimers needed for radio cycling */
  rtimer_init();
  
  /* Hardware initialization done! */
#if WITH_TINYOS_AUTO_IDS
  node_id = TOS_NODE_ID;
#else /* WITH_TINYOS_AUTO_IDS */
  /* Restore node id if such has been stored in external mem */
  node_id_restore();
#endif /* WITH_TINYOS_AUTO_IDS */

  /* for setting "hardcoded" IEEE 802.15.4 MAC addresses */
#ifdef IEEE_802154_MAC_ADDRESS
  {
    uint8_t ieee[] = IEEE_802154_MAC_ADDRESS;
    memcpy(mac_address, ieee, sizeof(uip_lladdr.addr));
    mac_address[7] = node_id & 0xff;
  }
#endif

  random_init(mac_address[0] + node_id);
  
  leds_off(LEDS_BLUE);
  /*
   * Initialize Contiki and our processes.
   */
  process_init();
  process_start(&etimer_process, NULL);

  ctimer_init();

  init_platform();

  set_rime_addr();
  
  rf230_init();
  {
    uint8_t longaddr[8];
    uint16_t shortaddr;
    
    shortaddr = (rimeaddr_node_addr.u8[0] << 8) +
      rimeaddr_node_addr.u8[1];
    memset(longaddr, 0, sizeof(longaddr));
    rimeaddr_copy((rimeaddr_t *)&longaddr, &rimeaddr_node_addr);
    printf("MAC %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x ",
           longaddr[0], longaddr[1], longaddr[2], longaddr[3],
           longaddr[4], longaddr[5], longaddr[6], longaddr[7]);
    
    rf230_set_pan_addr(IEEE802154_PANID, shortaddr, longaddr);
  }
  rf230_set_channel(RF_CHANNEL);

  printf(CONTIKI_VERSION_STRING " started. ");
  if(node_id > 0) {
    printf("Node id is set to %u.\n", node_id);
  } else {
    printf("Node id is not set.\n");
  }

  /*  printf("MAC %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
	 mac_address[0], mac_address[1], mac_address[2], mac_address[3],
	 mac_address[4], mac_address[5], mac_address[6], mac_address[7]);*/

#if WITH_UIP6
  memcpy(&uip_lladdr.addr, mac_address, sizeof(uip_lladdr.addr));
  /* Setup nullmac-like MAC for 802.15.4 */
/*   sicslowpan_init(sicslowmac_init(&cc2420_driver)); */
/*   printf(" %s channel %u\n", sicslowmac_driver.name, RF_CHANNEL); */

  /* Setup X-MAC for 802.15.4 */
  queuebuf_init();
  NETSTACK_RDC.init();
  NETSTACK_MAC.init();
  NETSTACK_NETWORK.init();

  printf("%s %s, channel check rate %lu Hz, radio channel %u\n",
         NETSTACK_MAC.name, NETSTACK_RDC.name,
         CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval() == 0 ? 1:
                         NETSTACK_RDC.channel_check_interval()),
         RF_CHANNEL);

  process_start(&tcpip_process, NULL);

  printf("Tentative link-local IPv6 address ");
  {
    uip_ds6_addr_t *lladdr;
    int i;
    lladdr = uip_ds6_get_link_local(-1);
    for(i = 0; i < 7; ++i) {
      printf("%02x%02x:", lladdr->ipaddr.u8[i * 2],
             lladdr->ipaddr.u8[i * 2 + 1]);
    }
    printf("%02x%02x\n", lladdr->ipaddr.u8[14], lladdr->ipaddr.u8[15]);
  }

  if(!UIP_CONF_IPV6_RPL) {
    uip_ipaddr_t ipaddr;
    int i;
    uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
    uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
    uip_ds6_addr_add(&ipaddr, 0, ADDR_TENTATIVE);
    printf("Tentative global IPv6 address ");
    for(i = 0; i < 7; ++i) {
      printf("%02x%02x:",
             ipaddr.u8[i * 2], ipaddr.u8[i * 2 + 1]);
    }
    printf("%02x%02x\n",
           ipaddr.u8[7 * 2], ipaddr.u8[7 * 2 + 1]);
  }

#else /* WITH_UIP6 */

  NETSTACK_RDC.init();
  NETSTACK_MAC.init();
  NETSTACK_NETWORK.init();

  printf("%s %s, channel check rate %lu Hz, radio channel %u\n",
         NETSTACK_MAC.name, NETSTACK_RDC.name,
         CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval() == 0? 1:
                         NETSTACK_RDC.channel_check_interval()),
         RF_CHANNEL);
#endif /* WITH_UIP6 */

//#if !WITH_UIP && !WITH_UIP6
//  uart1_set_input(serial_line_input_byte);
//  serial_line_init();
//#endif

#if PROFILE_CONF_ON
  profile_init();
#endif /* PROFILE_CONF_ON */

  leds_off(LEDS_GREEN);

#if TIMESYNCH_CONF_ENABLED
  timesynch_init();
  timesynch_set_authority_level((rimeaddr_node_addr.u8[0] << 4) + 16);
#endif /* TIMESYNCH_CONF_ENABLED */

#if WITH_UIP
  process_start(&tcpip_process, NULL);
  process_start(&uip_fw_process, NULL);	/* Start IP output */
  process_start(&ethernet_process, NULL);

  ethernet_set_input_callback(set_gateway);

  {
    uip_ipaddr_t hostaddr, netmask;

    uip_init();

    // TODO: Anton - code seems wrong
    //uip_ipaddr(&hostaddr, 172,16,
	  //     rimeaddr_node_addr.u8[0],rimeaddr_node_addr.u8[1]);
    uip_ipaddr(&hostaddr, 192,168,
	       rimeaddr_node_addr.u8[1],rimeaddr_node_addr.u8[0]);
    uip_ipaddr(&netmask, 255,255,0,0);
    uip_ipaddr_copy(&meshif.ipaddr, &hostaddr);

    uip_sethostaddr(&hostaddr);
    uip_setnetmask(&netmask);
    uip_over_mesh_set_net(&hostaddr, &netmask);
    /*    uip_fw_register(&ethif);*/
    uip_over_mesh_set_gateway_netif(&ethif);
    uip_fw_default(&meshif);
    uip_over_mesh_init(UIP_OVER_MESH_CHANNEL);
    printf("uIP started with IP address %d.%d.%d.%d\n",
	   uip_ipaddr_to_quad(&hostaddr));
  }
#endif /* WITH_UIP */

  energest_init();
  ENERGEST_ON(ENERGEST_TYPE_CPU);

  watchdog_start();

  print_processes(autostart_processes);
  autostart_start(autostart_processes);

  /*
   * This is the scheduler loop.
   */
#if DCOSYNCH_CONF_ENABLED
  timer_set(&mgt_timer, DCOSYNCH_PERIOD * CLOCK_SECOND);
#endif

  /*  watchdog_stop();*/
  while(1) {
    int r;
#if PROFILE_CONF_ON
    profile_episode_start();
#endif /* PROFILE_CONF_ON */
    do {
      /* Reset watchdog. */
      watchdog_periodic();
      r = process_run();
    } while(r > 0);
#if PROFILE_CONF_ON
    profile_episode_end();
#endif /* PROFILE_CONF_ON */

    /*
     * Idle processing.
     */
// TODO: Anton
//    int s = splhigh();		/* Disable interrupts. */
    /* uart1_active is for avoiding LPM3 when still sending or receiving */
//    if(process_nevents() != 0 || uart1_active()) {
		if(process_nevents() != 0 || ethernet_active()) {
// TODO: Anton
//      splx(s);			/* Re-enable interrupts. */
    }
		else {
      static unsigned long irq_energest = 0;

#if DCOSYNCH_CONF_ENABLED
      /* before going down to sleep possibly do some management */
      if(timer_expired(&mgt_timer)) {
        watchdog_periodic();
				timer_reset(&mgt_timer);
				msp430_sync_dco();
#if CC2420_CONF_SFD_TIMESTAMPS
        cc2420_arch_sfd_init();
#endif /* CC2420_CONF_SFD_TIMESTAMPS */
      }
#endif
      
      /* Re-enable interrupts and go to sleep atomically. */
      ENERGEST_OFF(ENERGEST_TYPE_CPU);
      ENERGEST_ON(ENERGEST_TYPE_LPM);
      /* We only want to measure the processing done in IRQs when we
	 are asleep, so we discard the processing time done when we
	 were awake. */
      energest_type_set(ENERGEST_TYPE_IRQ, irq_energest);
      watchdog_stop();
      /* check if the DCO needs to be on - if so - only LPM 1 */
// TODO: Anton
//      if (lm3s9b96_dco_required) {
//	_BIS_SR(GIE | CPUOFF); /* LPM1 sleep for DMA to work!. */
//      } else {
//	_BIS_SR(GIE | SCG0 | SCG1 | CPUOFF); /* LPM3 sleep. This
//						statement will block
//						until the CPU is
//						woken up by an
//						interrupt that sets
//						the wake up flag. */
//      }
      /* We get the current processing time for interrupts that was
	 done during the LPM and store it for next time around.  */
// TODO: Anton
//      dint();
      irq_energest = energest_type_time(ENERGEST_TYPE_IRQ);
// TODO: Anton
//      eint();
      watchdog_start();
      ENERGEST_OFF(ENERGEST_TYPE_LPM);
      ENERGEST_ON(ENERGEST_TYPE_CPU);
    }
  }

  return 0;
}
/*---------------------------------------------------------------------------*/
#if LOG_CONF_ENABLED
void
log_message(char *m1, char *m2)
{
  printf("%s%s\n", m1, m2);
}
#endif /* LOG_CONF_ENABLED */
