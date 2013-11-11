#include "FROMA_HEADER.h"

#define tskIDLE_STACK_SIZE	configMINIMAL_STACK_SIZE

static void prvSetupHardware();
PRIVILEGED_DATA static volatile int sharedValue = 0;
extern xTaskHandle xShellTaskHandle;

int secondary_main( void )
{
	xTaskHandle xPrimeTaskHandle;
	//char cAddress[20];
	//sharedValue++;
	prvSetupHardware();
	//sprintf( cAddress, "Core: %ld\r\n", 1 );
	//vSerialPutString((xComPortHandle)configUART_PORT,(const signed char * const)cAddress, strlen(cAddress) );
	//(void)xSerialPutChar( (xComPortHandle)mainPRINT_PORT, 0x61, 0UL );
	xTaskCreate( vPrimeTask, (const signed char *)"Prime", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, xPrimeTaskHandle);
	vTaskSuspend(xPrimeTaskHandle);
	while(1);
}

int main( void )
{
	//int num = 1000;
	sharedValue++;
	/* Initialise the Hardware. */
	prvSetupHardware();

	/* Start the tasks defined within the file. */
	//xTaskCreate( vCheckTask, (const signed char *)"Check", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL );
	//xTaskCreate( vPrimeTask, (const signed char *)"Prime", configMINIMAL_STACK_SIZE, &num, mainCHECK_TASK_PRIORITY, NULL );
	//xTaskCreate( vUARTEchoTask, (const signed char *)"EchoTask", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL );
	xTaskCreate( vShellTask, (const signed char *)"Shell", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, xShellTaskHandle);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Should never reach here. */
	vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)"Should never reach here!\r\n", 26 );

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;
}
/*----------------------------------------------------------------------------*/

void vApplicationTickHook( void )
{
	vSerialPutString( (xComPortHandle)mainPRINT_PORT, (const signed char * const)"Tick\r\n", 6 );
}
/*----------------------------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	signed char cChar;
	if ( pdTRUE == xSerialGetChar( (xComPortHandle)mainPRINT_PORT, &cChar, 0UL ) )
	{
		(void)xSerialPutChar( (xComPortHandle)mainPRINT_PORT, cChar, 0UL );
	}
}
/*----------------------------------------------------------------------------*/

static unsigned long prvReadP15_C1_C0_1( void )
{
	volatile unsigned long ulReturn = 0UL;
	/* Read Configuration Register C15, c0, 0. */
	__asm volatile(
			" mrc p15, 0, %[retval], c1, c0, 1 			\n"
			: [retval] "=r" (ulReturn) : :
			);
	return ulReturn;
}
/*----------------------------------------------------------------------------*/

static unsigned long prvReadP15_C1_C0_2( void )
{
	volatile unsigned long ulReturn = 0UL;
	/* Read Configuration Register C15, c0, 0. */
	__asm volatile(
			" mrc p15, 0, %[retval], c1, c0, 2 			\n"
			: [retval] "=r" (ulReturn) : :
			);
	return ulReturn;
}
/*----------------------------------------------------------------------------*/

static unsigned long prvReadP15_C1_C1_0( void )
{
	volatile unsigned long ulReturn = 0UL;
	/* Read Configuration Register C15, c0, 0. */
	__asm volatile(
			" mrc p15, 0, %[retval], c1, c1, 0 			\n"
			: [retval] "=r" (ulReturn) : :
			);
	return ulReturn;
}
/*----------------------------------------------------------------------------*/

static unsigned long prvReadP15_C1_C1_1( void )
{
	volatile unsigned long ulReturn = 0UL;
	/* Read Configuration Register C15, c0, 0. */
	__asm volatile(
			" mrc p15, 0, %[retval], c1, c1, 1 			\n"
			: [retval] "=r" (ulReturn) : :
			);
	return ulReturn;
}
/*----------------------------------------------------------------------------*/

static unsigned long prvReadP15_C1_C1_2( void )
{
	volatile unsigned long ulReturn = 0UL;
	/* Read Configuration Register C15, c0, 0. */
	__asm volatile(
			" mrc p15, 0, %[retval], c1, c1, 2 			\n"
			: [retval] "=r" (ulReturn) : :
			);
	return ulReturn;
}

/*----------------------------------------------------------------------------*/

static unsigned long prvReadP15_C12_C0_1( void )
{
	volatile unsigned long ulReturn = 0UL;
	/* Read Configuration Register C15, c0, 0. */
	__asm volatile(
			" mrc p15, 0, %[retval], c12, c0, 1 		\n"
			: [retval] "=r" (ulReturn) : :
			);
	return ulReturn;
}
/*----------------------------------------------------------------------------*/

static void prvSetupHardware( void )
{
	unsigned long ulVector = 0UL;
	unsigned long ulValue = 0UL;
	unsigned long looped = 99999999UL;
	char cAddress[32];

	ulValue = portCORE_ID();

	portDISABLE_INTERRUPTS();

	/* Install the Spurious Interrupt Handler to help catch interrupts. */
	extern void vPortUnknownInterruptHandler( void *pvParameter );
	extern void vPortInstallInterruptHandler( void (*vHandler)(void *), void *pvParameter, unsigned long ulVector, 
			unsigned char ucEdgeTriggered, unsigned char ucPriority, unsigned char ucProcessorTargets );
	for ( ulVector = 0; ulVector < portMAX_VECTORS; ulVector++ )
		vPortInstallInterruptHandler( vPortUnknownInterruptHandler, (void *)ulVector, ulVector, pdTRUE, configMAX_SYSCALL_INTERRUPT_PRIORITY, 1 );

	extern void vUARTInitialise(unsigned long ulUARTPeripheral, unsigned long ulBaud, unsigned long ulQueueSize );
	if(ulValue==PRIMARY_CPU_ID)
		vUARTInitialise( mainPRINT_PORT, mainPRINT_BAUDRATE, 64 );

	//sprintf( cAddress, "Core: %ld\r\n", ulValue );
	//vSerialPutString((xComPortHandle)configUART_PORT,(const signed char * const)cAddress, strlen(cAddress) );

	//sprintf( cAddress, "SVA: 0x%X", &sharedValue);
	//sprintf( cAddress, "%d", sharedValue);
	//vSerialPutString((xComPortHandle)configUART_PORT,(const signed char * const)cAddress, strlen(cAddress) );
	/* Perform any other peripheral configuration. */
}
/*----------------------------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	__asm volatile (" smc #0 ");
}
/*----------------------------------------------------------------------------*/

extern void vAssertCalled( char *file, int line )
{
	printf("Assertion failed at %s, line %d\n\r",file,line);
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

