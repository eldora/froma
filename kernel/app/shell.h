#ifndef SHELL_H
#define SHELL_H

void vShellTask(void *pvParameters);
void vHelp(const char *pcParameterBuffer);
void vClear(const char *pcParameterBuffer);
void vVersion(const char *pcParameterBuffer);
void vPrime(const char *pcParameterBuffer);
void vTaskState(const char *pcParameterBuffer);
void vExecuteCommand(const char *pcCommandBuffer);

/** struct **/
struct shellCommandEntry {
	char *pcCommand;
	char *pcHelp;

	void (*pvFunction)(const char *pcParameter);
};

#endif
