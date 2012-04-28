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
 *  Filename: halbb.c
 *  Created on: 06 Jan 2012
 *  Author: Anton Veldhuizen
 */

#include "lm3s9b96inc.h"

#include <stdlib.h>
#include <driverlib/interrupt.h>
#include <driverlib/ssi.h>

#include "hal.h"
#include "at86rf230_registermap.h"

/*============================ VARIABLES =====================================*/
//
// Define base address for SSI module to use
//
#define SSI1 SSI1_BASE

volatile extern signed char rf230_last_rssi;
static uint8_t spiRx;

/*============================ IMPLEMENTATION ================================*/
void spi_enable() {
	MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_5, 0);
}

void spi_disable() {
	MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_5, GPIO_PIN_5);
}

void spi_write(uint8_t txData) {
	unsigned long rxData;

	MAP_SSIDataPut(SSI1, txData);
	MAP_SSIDataGet(SSI1, &rxData);

	spiRx = (uint8_t)(rxData & 0xFF);
}

uint8_t spi_read() {
	return spiRx;
}

uint8_t spi_transfer(uint8_t txData) {
	unsigned long rxData;

	MAP_SSIDataPut(SSI1, txData);
	MAP_SSIDataGet(SSI1, &rxData);

	return (uint8_t)(rxData & 0xFF);
}

void hal_init(void) {
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
	delay_us(5);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
	delay_us(5);
	MAP_GPIOPinTypeSSI(GPIO_PORTH_BASE,
			GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7);

	MAP_GPIOPinConfigure(GPIO_PH4_SSI1CLK);
	MAP_GPIOPinConfigure(GPIO_PH6_SSI1RX);
	MAP_GPIOPinConfigure(GPIO_PH7_SSI1TX);

	/* Manaully control /SEL line, else SSI deselects between each packet*/
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, GPIO_PIN_5);
	MAP_GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_5, GPIO_PIN_5);

	MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_SSI1);
	delay_us(5);
	MAP_SSIDisable(SSI1);
	MAP_SSIConfigSetExpClk(SSI1, MAP_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
			SSI_MODE_MASTER, 7500000, 8);
	delay_us(5);
	MAP_SSIEnable(SSI1);

	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	delay_us(5);
	MAP_GPIOPinTypeGPIOOutput(PORT_RST, PIN_RST);
	MAP_GPIOPinWrite(PORT_RST, PIN_RST, PIN_RST); /* Active Low */
	MAP_GPIOPinTypeGPIOInput(PORT_DIG2, PIN_DIG2);

	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	delay_us(5);
	MAP_GPIOPinTypeGPIOOutput(PORT_SLP_TR, PIN_SLP_TR);
	MAP_GPIOPinWrite(PORT_SLP_TR, PIN_SLP_TR, 0);
	MAP_GPIOPinTypeGPIOInput(PORT_CLKM, PIN_CLKM);
	MAP_GPIOPinTypeGPIOInput(PORT_IRQ, PIN_IRQ);
	MAP_GPIOIntTypeSet(PORT_IRQ, PIN_IRQ, GPIO_HIGH_LEVEL);
	MAP_GPIOPinIntEnable(PORT_IRQ, PIN_IRQ);
	MAP_GPIOPinIntClear(PORT_IRQ, PIN_IRQ);
	MAP_IntEnable(INT_GPIOG);

	// TODO: Delete
//	for (;;) {
//		long c;
//
//		hal_set_slptr_low();
//		c = hal_get_slptr();
//		hal_set_slptr_high();
//		c = hal_get_slptr();
//
//		hal_set_rst_low();
//		c = hal_get_rst();
//		hal_set_rst_high();
//		c = hal_get_rst();
//
//		hal_register_write(0xAA, 0xAA);
//	}
}
/*----------------------------------------------------------------------------*/
/** \brief  This function reads data from one of the radio transceiver's registers.
 *
 *  \param  address Register address to read from. See datasheet for register
 *                  map.
 *
 *  \see Look at the at86rf230_registermap.h file for register address definitions.
 *
 *  \returns The actual value of the read register.
 */
uint8_t hal_register_read(uint8_t address) {
	uint8_t register_value;
	/* Add the register read command to the register address. */
	/* Address should be < 0x2f so no need to mask */
	address |= 0x80;

	spi_enable();

	/*Send Register address and read register content.*/
	spi_transfer(address);
	register_value = spi_transfer(0);

	spi_disable();

	return register_value;
}

/*----------------------------------------------------------------------------*/
/** \brief  This function writes a new value to one of the radio transceiver's
 *          registers.
 *
 *  \see Look at the at86rf230_registermap.h file for register address definitions.
 *
 *  \param  address Address of register to write.
 *  \param  value   Value to write.
 */
void hal_register_write(uint8_t address, uint8_t value) {
	/* Add the Register Write (short mode) command to the address. */
	address |= 0xc0;

	spi_enable();

	/*Send Register address and write register content.*/
	spi_transfer(address);
	spi_transfer(value);

	spi_disable();
}
/*----------------------------------------------------------------------------*/
/** \brief  This function reads the value of a specific subregister.
 *
 *  \see Look at the at86rf230_registermap.h file for register and subregister
 *       definitions.
 *
 *  \param  address  Main register's address.
 *  \param  mask  Bit mask of the subregister.
 *  \param  position   Bit position of the subregister
 *  \retval Value of the read subregister.
 */
uint8_t hal_subregister_read(uint8_t address, uint8_t mask, uint8_t position) {
	/* Read current register value and mask out subregister. */
	uint8_t register_value = hal_register_read(address);
	register_value &= mask;
	register_value >>= position; /* Align subregister value. */

	return register_value;
}
/*----------------------------------------------------------------------------*/
/** \brief  This function writes a new value to one of the radio transceiver's
 *          subregisters.
 *
 *  \see Look at the at86rf230_registermap.h file for register and subregister
 *       definitions.
 *
 *  \param  address  Main register's address.
 *  \param  mask  Bit mask of the subregister.
 *  \param  position  Bit position of the subregister
 *  \param  value  Value to write into the subregister.
 */
void hal_subregister_write(uint8_t address, uint8_t mask, uint8_t position,
		uint8_t value) {
	/* Read current register value and mask area outside the subregister. */
	volatile uint8_t register_value = hal_register_read(address);
	register_value &= ~mask;

	/* Start preparing the new subregister value. shift in place and mask. */
	value <<= position;
	value &= mask;

	value |= register_value; /* Set the new subregister value. */

	/* Write the modified register value. */
	hal_register_write(address, value);
}
/*----------------------------------------------------------------------------*/
/** \brief  This function will upload a frame from the radio transceiver's frame
 *          buffer.
 *
 *          If the frame currently available in the radio transceiver's frame buffer
 *          is out of the defined bounds. Then the frame length, lqi value and crc
 *          be set to zero. This is done to indicate an error.
 *          This version is optimized for use with contiki RF230BB driver.
 *          The callback routine and CRC are left out for speed in reading the rx buffer.
 *          Any delays here can lead to overwrites by the next packet!
 *
 *  \param  rx_frame    Pointer to the data structure where the frame is stored.
 *  \param  rx_callback Pointer to callback function for receiving one byte at a time.
 */
void
//hal_frame_read(hal_rx_frame_t *rx_frame, rx_callback_t rx_callback)
hal_frame_read(hal_rx_frame_t *rx_frame) {
	uint8_t *rx_data;

	spi_enable();

	/*Send frame read (long mode) command.*/
	spi_transfer(0x20);
	/*Read frame length. This includes the checksum. */
	uint8_t frame_length = spi_transfer(0);

	rx_data = (rx_frame->data);
	rx_frame->length = frame_length;

	spi_write(0);
	do {
		*rx_data++ = spi_read();
		spi_write(0);
	} while (--frame_length > 0);

	/*Read LQI value for this frame.*/
	rx_frame->lqi = spi_read();

	spi_disable();
}

/*----------------------------------------------------------------------------*/
/** \brief  This function will download a frame to the radio transceiver's frame
 *          buffer.
 *
 *  \param  write_buffer    Pointer to data that is to be written to frame buffer.
 *  \param  length          Length of data. The maximum length is 127 bytes.
 */
void hal_frame_write(uint8_t *write_buffer, uint8_t length) {
	spi_enable();

	/* Send Frame Transmit (long mode) command and frame length */
	spi_transfer(0x60);
	spi_transfer(length);

	/* Download to the Frame Buffer.
	 * When the FCS is autogenerated there is no need to transfer the last two bytes
	 * since they will be overwritten.
	 */
#if !RF230_CONF_CHECKSUM
	length -= 2;
#endif
	do {
		spi_transfer(*write_buffer++);
	} while (--length);

	spi_disable();
}

/*----------------------------------------------------------------------------*/
/* This #if compile switch is used to provide a "standard" function body for the */
/* doxygen documentation. */
#if defined(DOXYGEN)
/** \brief ISR for the radio IRQ line, triggered by the input capture.
 *  This is the interrupt service routine for timer1.ICIE1 input capture.
 *  It is triggered of a rising edge on the radio transceivers IRQ line.
 */
void RADIO_VECT(void);
#else  /* !DOXYGEN */
/* These link to the RF230BB driver in rf230.c */
void rf230_interrupt(void);

extern hal_rx_frame_t rxframe[RF230_CONF_RX_BUFFERS];
extern uint8_t rxframe_head, rxframe_tail;

/* rf230interruptflag can be printed in the main idle loop for debugging */
#define DEBUG 0
#if DEBUG
volatile char rf230interruptflag;
#define INTERRUPTDEBUG(arg) rf230interruptflag=arg
#else
#define INTERRUPTDEBUG(arg)
#endif

/* Separate RF230 has a single radio interrupt and the source must be read from the IRQ_STATUS register */
void IsrRadioIRQ(void) {
	volatile uint8_t state;
	uint8_t interrupt_source; /* used after HAL_SPI_TRANSFER_OPEN/CLOSE block */

	long temp = MAP_GPIOPinIntStatus(PORT_IRQ, true);
	MAP_GPIOPinIntClear(PORT_IRQ, temp);
	if (temp & PIN_IRQ)
	{
		INTERRUPTDEBUG(1);

		/* Using SPI bus from ISR is generally a bad idea... */
		/* Note: all IRQ are not always automatically disabled when running in ISR */
		spi_enable();

		/*Send Register address and read register content.*/
		spi_transfer(0x80 | RG_IRQ_STATUS);
		/*Read Interrupt source.*/
		interrupt_source = spi_transfer(0);

		spi_disable();
	}

	/*Handle the incomming interrupt. Prioritized.*/
	if ((interrupt_source & HAL_RX_START_MASK)) {
		INTERRUPTDEBUG(10);
		/* Save RSSI for this packet if not in extended mode, scaling to 1dB resolution */
#if !RF230_CONF_AUTOACK
		rf230_last_rssi = 3 * hal_subregister_read(SR_RSSI);
#endif
	} else if (interrupt_source & HAL_TRX_END_MASK)
	{
		INTERRUPTDEBUG(11);

		state = hal_subregister_read(SR_TRX_STATUS);
		if ((state == BUSY_RX_AACK) || (state == RX_ON) || (state == BUSY_RX)
				|| (state == RX_AACK_ON)) {
			/* Received packet interrupt */
			/* Buffer the frame and call rf230_interrupt to schedule poll for rf230 receive process */
			if (rxframe[rxframe_tail].length) {
				INTERRUPTDEBUG(42);
			} else {
				INTERRUPTDEBUG(12);
			}

#ifdef RF230_MIN_RX_POWER		 
			/* Discard packets weaker than the minimum if defined. This is for testing miniature meshes.*/
			/* Save the rssi for printing in the main loop */
#if RF230_CONF_AUTOACK
			//       rf230_last_rssi = hal_subregister_read(SR_ED_LEVEL);
			rf230_last_rssi = hal_register_read(RG_PHY_ED_LEVEL);
#endif
			if (rf230_last_rssi >= RF230_MIN_RX_POWER) {
#endif
			hal_frame_read(&rxframe[rxframe_tail]);
			rxframe_tail++;
			if (rxframe_tail >= RF230_CONF_RX_BUFFERS)
			{
				rxframe_tail = 0;
			}
			rf230_interrupt();
#ifdef RF230_MIN_RX_POWER
		}
#endif

		}
	} else if (interrupt_source & HAL_TRX_UR_MASK)
	{
		INTERRUPTDEBUG(13);;
	} else if (interrupt_source & HAL_PLL_UNLOCK_MASK)
	{
		INTERRUPTDEBUG(14);;
	} else if (interrupt_source & HAL_PLL_LOCK_MASK)
	{
		INTERRUPTDEBUG(15);;
	} else if (interrupt_source & HAL_BAT_LOW_MASK)
	{
		/*  Disable BAT_LOW interrupt to prevent endless interrupts. The interrupt */
		/*  will continously be asserted while the supply voltage is less than the */
		/*  user-defined voltage threshold. */
		uint8_t trx_isr_mask = hal_register_read(RG_IRQ_MASK);
		trx_isr_mask &= ~HAL_BAT_LOW_MASK;
		hal_register_write(RG_IRQ_MASK, trx_isr_mask);
		INTERRUPTDEBUG(16);;
	} else {
		INTERRUPTDEBUG(99);;
	}
}
#   endif /* defined(DOXYGEN) */

/*----------------------------------------------------------------------------*/
/** @} */
/** @} */

/*EOF*/
