#include "FROMA_HEADER.h"

#define COMMAND_BUFFER_SIZE		100

xTaskHandle xShellTaskHandle;

struct shellCommandEntry gsCommandTable[] = {
	{"help", "View available commands and their description.", vHelp},
	{"version", "Show the version information of the OS.", vVersion},
	{"clear", "Clear Screen.", vClear},
	{"state", "View Tasks State.", vTaskState},
	{"prime", "Search Prime Number.", vPrime},
};

void vShellTask( void *pvParameters ){
	portCHAR vcCommandBuffer[COMMAND_BUFFER_SIZE];
	portBASE_TYPE xCommandBufferIndex = 0;
	signed portCHAR cInputKey;

	vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)"FROMA$ ", 7);
	while(1){
		if(pdTRUE == xSerialGetChar((xComPortHandle)mainPRINT_PORT, &cInputKey, portMAX_DELAY)){

			switch(cInputKey){
				case KEY_BACKSPACE:
					if(xCommandBufferIndex > 0){
						xSerialPutChar((xComPortHandle)mainPRINT_PORT, cInputKey, portMAX_DELAY);
						xSerialPutChar((xComPortHandle)mainPRINT_PORT, ' ', portMAX_DELAY);
						xSerialPutChar((xComPortHandle)mainPRINT_PORT, cInputKey, portMAX_DELAY);
						xCommandBufferIndex--;
					}
					break;
				case KEY_ENTER:
					if(xCommandBufferIndex > 0){
						vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)"\r\n", strlen("\r\n"));
						vcCommandBuffer[xCommandBufferIndex] = '\0';
						vExecuteCommand(vcCommandBuffer);
					}
					memset(vcCommandBuffer, 0, COMMAND_BUFFER_SIZE);
					xCommandBufferIndex = 0;
					break;

				default:
					if(xCommandBufferIndex < COMMAND_BUFFER_SIZE - 1){			// -1 meaning: SECURE ENTER BUFFER SPACE 
						vcCommandBuffer[xCommandBufferIndex++] = cInputKey;
						xSerialPutChar((xComPortHandle)mainPRINT_PORT, cInputKey, portMAX_DELAY);
					}
					else
						vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)"Command Buffer is Full\r\n", strlen("Command Buffer is Full\r\n"));
					break;
			}
		}
	}
}

void vExecuteCommand(const char *pcCommandBuffer){
	int i, iSpaceIndex, iCommandTypeNumber;
	char *pcCommand;
	char cTempBuffer[COMMAND_BUFFER_SIZE+20];

	iCommandTypeNumber = sizeof(gsCommandTable)/sizeof(struct shellCommandEntry);

	pcCommand = strtok(pcCommandBuffer, " ");
	for(i=0; i<iCommandTypeNumber; i++){
		if(!strcmp(pcCommand, gsCommandTable[i].pcCommand)){
			gsCommandTable[i].pvFunction(strtok(NULL, ""));
			break;
		}
	}
	if(i>=iCommandTypeNumber){
		sprintf(cTempBuffer, "%s: command not found\r\n", pcCommandBuffer);
		vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)cTempBuffer, strlen(cTempBuffer));
	}
	vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)"FROMA$ ", 7);
}

void vHelp(const char *pcParameterBuffer){
	int i, iCommandTypeNumber;

	iCommandTypeNumber = sizeof(gsCommandTable)/sizeof(struct shellCommandEntry);
	
	vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)"=== Shell of FROMA Help ===\r\n", strlen("=== Shell of FROMA Help ===\r\n"));
	for(i=0; i<iCommandTypeNumber; i++){
		vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)gsCommandTable[i].pcCommand, strlen(gsCommandTable[i].pcCommand));
		vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)"\t- ", 3);
		vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)gsCommandTable[i].pcHelp, strlen(gsCommandTable[i].pcHelp));
		vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)"\r\n", strlen("\r\n"));
	}
}

void vClear(const char *pcParameterBuffer){
	portBASE_TYPE xTerminalHeight;

	vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)"\033[2J", strlen("\033[2J"));
#if 0	
	xTerminalHeight = 50;
	while(xTerminalHeight--)
		vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)"\r\n", 2);
	xTerminalHeight = 25;
	while(xTerminalHeight--){
		xSerialPutChar((xComPortHandle)mainPRINT_PORT, 224, portMAX_DELAY);
		xSerialPutChar((xComPortHandle)mainPRINT_PORT, 72, portMAX_DELAY);
	}
#endif
}

void vVersion(const char *pcParameterBuffer){
	portCHAR cTempBuffer[50];
	portCHAR *pcVersion = "1.04";

	vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)"Distributor ID:\tFROMA\r\n", strlen("Distributor ID:\tFROMA\r\n"));
	sprintf(cTempBuffer, "Description:\tFROMA %s SMP Supported\r\n", pcVersion);
	vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)cTempBuffer, strlen(cTempBuffer));
	sprintf(cTempBuffer, "Release:\t%s\r\n", pcVersion);
	vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)cTempBuffer, strlen(cTempBuffer));
	vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)"Codename:\trudder\r\n", strlen("Codename:\trudder\r\n"));
}

void vPrime(const char *pcParameterBuffer){
	extern int xPrimeTaskStart;
	int num = 99999999;
	//xPrimeTaskStart = 1;
	//vTaskSuspend(xShellTaskHandle);
	vTaskDelay(num);
	taskYIELD();
}

void vTaskState(const char *pcParameterBuffer){
	portBASE_TYPE xNumberOfTasks;
	portCHAR cTempBuffer[300];

	xNumberOfTasks = uxTaskGetNumberOfTasks();

	sprintf(cTempBuffer, "Number Of Tasks: %u\r\n", xNumberOfTasks);
	vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)cTempBuffer, strlen(cTempBuffer));

	sprintf(cTempBuffer, "Task Name\tStatus\tPrority\tStack\tNumber", xNumberOfTasks);
	vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)cTempBuffer, strlen(cTempBuffer));

	vTaskList(cTempBuffer);
	vSerialPutString((xComPortHandle)mainPRINT_PORT, (const signed char * const)cTempBuffer, strlen(cTempBuffer));
}
