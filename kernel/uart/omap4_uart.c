/*
    FreeRTOS V7.0.1 - Copyright (C) 2011 Real Time Engineers Ltd.

	Modified by Dag Ågren, Åbo Akademi University - Dept. of Information Technologies:
	implemented support for UARTs on the OMAP4.

	FreeRTOS supports many tools and architectures. V7.0.0 is sponsored by:
	Atollic AB - Atollic provides professional embedded systems development
	tools for C/C++ development, code analysis and test automation.
	See http://www.atollic.com


    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

/* UART Driver for the OMAP4. */

#include "FreeRTOS.h"
#include "serial.h"
#include "spinlock.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#define REG8(address) (*(volatile unsigned char *)(address))
#define REG32(address) (*(volatile unsigned int *)(address))

#define OMAP44XX_L4_PER 0x48000000
#define UART1_BASE (OMAP44XX_L4_PER+0x6a000) // OMAP4460, UART1 in data sheet
#define UART2_BASE (OMAP44XX_L4_PER+0x6c000) // OMAP4460, UART2 in data sheet
#define UART3_BASE (OMAP44XX_L4_PER+0x20000) // OMAP4460, UART3 in data sheet
#define UART4_BASE (OMAP44XX_L4_PER+0x6e000) // OMAP4460, UART4 in data sheet

#define UART_RBR(base) REG8((base)+0x00)
#define UART_IER(base) REG8((base)+0x04)
#define UART_FCR(base) REG8((base)+0x08)
#define UART_IIR(base) REG8((base)+0x08)
#define UART_LCR(base) REG8((base)+0x0c)
#define UART_MCR(base) REG8((base)+0x10)
#define UART_LSR(base) REG8((base)+0x14)
#define UART_MSR(base) REG8((base)+0x18)
#define UART_SCR(base) REG8((base)+0x1c)
#define UART_MDR1(base) REG8((base)+0x20)

//#define UART_CLK 3686400

//static unsigned long ulUARTSemaphore = 0;
PRIVILEGED_DATA static volatile spinlock_t lock = {0,};
//PRIVILEGED_DATA volatile spinlock_t lock = {0,};
//PRIVILEGED_DATA static volatile int xCheckFlag = 0;
//PRIVILEGED_DATA static volatile int xCheckFlag = 0;
//int xCheckFlag = 0;

void vUARTInitialise(unsigned long ulUARTPeripheral, unsigned long ulBaud, unsigned long ulQueueSize )
{
	extern void prvSetupUARTInterrupt( void );
	/* UART3 Interrupt Enabled */
	//UART_IIR(UART3_BASE) |= 
	prvSetupUARTInterrupt();

	int tmp;
	int efr;

#if 0
	// 1. Disable UART to access the UARTi.UART_DLL and UARTi.UART_DLH registers:
	tmp = read((int *)UART_MDR1);
	write(tmp | 0x7, (int *)UART_MDR1);
	
	// 2. Switch to register configuration mode B to access the UARTi.UART_EFR register:
	write(0x00BF, (int *)UART_LCR);
	
	// 3. Enable access to the UARTi.UART_IER[7:4] bit field:
	efr = read((int *)UART_EFR);
	write(efr | ENHANCED_EN, (int *)UART_EFR);
	
	// 4. Switch to register operational mode to access the UARTi.UART_IER register:
	write(0x00, (int *)UART_LCR);
	
	// 5. Clear the UARTi.UART_IER register (set the UARTi.UART_IER[4] SLEEP_MODE bit to 0 to change
	// the UARTi.UART_DLL and UARTi.UART_DLH registers). Set the UARTi.UART_IER register value to
	// 0x0000.
	write(0, (int *)UART_IER);
	
	// 6. Switch to register configuration mode B to access the UARTi.UART_DLL and UARTi.UART_DLH
	// registers:
	write(0x00BF, (int *)UART_LCR);
	
	// 7. Load the new divisor value:
	write(0x1a, (int *)UART_DLL);
	write(0x00, (int *)UART_DLH);
	
	// 8. Switch to register operational mode to access the UARTi.UART_IER register:
	write(0, (int *)UART_LCR);
	
	// 9. Load the new interrupt configuration (0: Disable the interrupt; 1: Enable the interrupt):
	write(0x1, (int *)UART_IER);
	
	// 10. Switch to register configuration mode B to access the UARTi.UART_EFR register:
	write(0x00BF, (int *)UART_LCR);
	
	// 11. Restore the UARTi.UART_EFR[4] ENHANCED_EN value saved in Step 3a.
	tmp = read((int *)UART_EFR);
	tmp &= ~ENHANCED_EN;
	write(tmp | (efr & ENHANCED_EN), (int *)UART_EFR);
	
	// 12. Load the new protocol formatting (parity, stop-bit, character length) and switch to register operational
	// mode:
	// Set the UARTi.UART_LCR[7] DIV_EN bit to 0.
	// Set the UARTi.UART_LCR[6] BREAK_EN bit to 0.
	// 8 bits, No parity, 1 stop bit
	write(0x3, (int *)UART_LCR);
	
	// 13. Load the new UART mode:
	write(0x0, (int *)UART_MDR1);

	UART_IER(UART3_BASE) |= 0x03;
#endif

	/* First Disable the Peripheral. */
//	*UARTCR(ulBase) = 0UL;

	/* Configure the Peripheral. */
//	*UARTIBRD(ulBase) = UART_CLK / ( 16 * ulBaud );
//	*UARTFBRD(ulBase) = UART_CLK % ( 16 * ulBaud );
//	*UARTLCR_H(ulBase) = ( 3 << 5 ) | ( 1 << 4 );

	/* Configure the Interrupts. */
//	*UARTIFLS(ulBase) = ( 0 << 3 ) | ( 0 << 0 );
//	*UARTICR(ulBase) = 0xFFFF;	/* Clear all Iterrupts. */

	/* Finally enable the peripheral. */
//	*UARTCR(ulBase) = ( 1 << 9 ) | ( 1 << 8 ) | /* Loopback Enable ( 1 << 7 ) | */ ( 1 << 0 );
}
/*----------------------------------------------------------------------------*/

portBASE_TYPE xUARTSendCharacter( unsigned long ulUARTPeripheral, signed char cChar, portTickType xDelay )
{
	unsigned long base;
	switch(ulUARTPeripheral)
	{
		case 0: base=UART1_BASE; break;
		case 1: base=UART2_BASE; break;
		case 2: base=UART3_BASE; break;
		case 3: base=UART4_BASE; break;
		default: return pdFALSE;
	}

	while((UART_LSR(base)&0x20)==0);
	UART_RBR(base)=cChar;

	return pdTRUE;
}
/*----------------------------------------------------------------------------*/

portBASE_TYPE xUARTReceiveCharacter( unsigned long ulUARTPeripheral, signed char *pcChar, portTickType xDelay )
{
	unsigned long base;
	switch(ulUARTPeripheral)
	{
		case 0: base=UART1_BASE; break;
		case 1: base=UART2_BASE; break;
		case 2: base=UART3_BASE; break;
		case 3: base=UART4_BASE; break;
		default: return pdFALSE;
	}

	if((UART_LSR(base)&0x01)==0) return pdFALSE;

	*pcChar=UART_RBR(base);
	return pdTRUE;
}

void vSerialPutString( xComPortHandle pxPort, const signed char * const pcString, unsigned short usStringLength )
{
	unsigned short usIndex = 0; 
	//while(lock.flag!=0);
	//lock.flag++;
	portDISABLE_INTERRUPTS();
	__spin_lock(&lock);
	//xUARTSendCharacter( (unsigned long)pxPort, __spin_lock(&lock)+33, portMAX_DELAY );
	//xUARTSendCharacter( (unsigned long)pxPort, lock.flag+40, portMAX_DELAY );
	//if(portCORE_ID()==0)

	//	xCheckFlag++;
	//xUARTSendCharacter( (unsigned long)pxPort, xCheckFlag+33, portMAX_DELAY );
	for ( usIndex = 0; usIndex < usStringLength; usIndex++ )
	{
		if ( pdTRUE != xUARTSendCharacter( (unsigned long)pxPort, pcString[usIndex], portMAX_DELAY ) )
		{    
			/* Something has gone wrong with the queue. */
			break;
		}    
	}
	//if(portCORE_ID())
	//	xUARTSendCharacter( (unsigned long)pxPort, xCheckFlag+33, portMAX_DELAY );
	__spin_unlock(&lock);
	portENABLE_INTERRUPTS();
	//lock.flag--;
	//xUARTSendCharacter( (unsigned long)pxPort, lock.flag+40, portMAX_DELAY );
}

signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed char *pcRxedChar, portTickType xBlockTime )
{
	  return xUARTReceiveCharacter( (unsigned long)pxPort, pcRxedChar, xBlockTime );
}

signed portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, signed char cOutChar, portTickType xBlockTime )
{
	  return xUARTSendCharacter( (unsigned long)pxPort, cOutChar, xBlockTime );
}
