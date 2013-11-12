#ifndef FROMA_HEADER_H
#define FROMA_HEADER_H

/* Standard includes. */
#include <stdio.h>
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Demo Includes. */

/* Uart Includes. */
#include "serial.h"

/* Application Includes. */
#include "apps.h"
#include "shell.h"

/* Sync Includes. */
#include "spinlock.h"

/* Define Reference Includes. */
#include "keyboard.h"			// Keyboard Map

/*----------------------------------------------------------------------------*/

/* Delay between cycles of the 'check' task. */
#define mainCHECK_DELAY						( ( portTickType ) 5000 / portTICK_RATE_MS )

/* Demo task priorities. */
#define mainQUEUE_POLL_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )
#define mainSEM_TEST_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainGEN_Q_PRIORITY			( tskIDLE_PRIORITY )

/* Misc. */
#define mainQUEUE_SIZE					( 3 )
#define mainNO_DELAY						( ( portTickType ) 0 )

#define mainPRINT_PORT					( configUART_PORT )
#define mainPRINT_BAUDRATE			( 115200 )

/*----------------------------------------------------------------------------*/

/*
 * Configure the clocks, GPIO and other peripherals as required by the demo.
 */

/*----------------------------------------------------------------------------*/

#define SYS_CFGDATA (*(volatile uint32_t *)0x100000a0)
#define SYS_CFGCTRL (*(volatile uint32_t *)0x100000a4)
#define SYS_CFGSTAT (*(volatile uint32_t *)0x100000a8)

#define mPutCh(ch)		(xSerialGetChar((xComPortHandle)mainPRINT_PORT, &ch, portMAX_DELAY))
#define mGetCh(ch)		(xSerialPutChar((xComPortHandle)mainPRINT_PORT, ch, portMAX_DELAY))

#define PRIMARY_CPU_ID		0
#define SECONDARY_CPU_ID	1

#endif
