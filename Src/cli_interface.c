/*
 * cli_interface.c
 *
 *  Created on: Jun 28, 2019
 *      Author: Andy.Kobyljanec
 */

#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "task.h"
#include "semphr.h"

#include "serial.h"

#define PRIORITY_CLI        5

#define INPUT_MAX_SIZE      200

#define CMD_QUEUE_LENGTH    200

// Longest wait for the serial port to become free
#define MAX_MUTEX_WAIT      pdMS_TO_TICKS(300)

static SemaphoreHandle_t xTxMutex;

static void command_console_task(void *pvParameters);

static const char * const pcWelcomeMessage =
    "FreeRTOS command server.\r\nType Help to view a list of registered commands.\r\n\r\n>";

void cli_interface_init(void)
{
    xTxMutex = xSemaphoreCreateMutex();
    vQueueAddToRegistry(xTxMutex, "CLI tx mutex");

    xTaskCreate(command_console_task, "CLI", 200,
                NULL,
                PRIORITY_CLI,
                NULL);
}

static void command_console_task(void *pvParameters)
{
    xComPortHandle xPort;
    char cRxChar;
    uint8_t ucInputIndex = 0;
    char *pcOutputString;
    static char cInputString[INPUT_MAX_SIZE];
    static char cLastInputString[INPUT_MAX_SIZE];
    BaseType_t xReturned;
    memset(cLastInputString, 0x00, INPUT_MAX_SIZE);

    xPort = xSerialPortInitMinimal(115200, CMD_QUEUE_LENGTH);

    pcOutputString = FreeRTOS_CLIGetOutputBuffer();

    vSerialPutString(xPort, (signed char *) pcWelcomeMessage,
                     strlen(pcWelcomeMessage));

    for(;;)
    {
        while(xSerialGetChar(xPort, &cRxChar, portMAX_DELAY) != pdPASS)
            ;

        if(xSemaphoreTake(xTxMutex, MAX_MUTEX_WAIT) == pdPASS)
        {
            // Echo character back
            xSerialPutChar(xPort, cRxChar, portMAX_DELAY);

            // Test for end of line
            if((cRxChar == '\n') || (cRxChar == '\r'))
            {
                vSerialPutString(xPort, (signed char *) "\r\n", 2);

                if(ucInputIndex == 0)
                {
                    strcpy(cInputString, cLastInputString);
                }

                // Pass command into command interpreter
                do
                {
                    xReturned = FreeRTOS_CLIProcessCommand(cInputString,
                                                           pcOutputString,
                                                           configCOMMAND_INT_MAX_OUTPUT_SIZE);

                    vSerialPutString(xPort, (signed char *) pcOutputString,
                                     strlen(pcOutputString));

                }
                while(xReturned != pdFALSE);

                // Clear the input ready string and remember last command in case it is resent
                strcpy(cLastInputString, cInputString);
                ucInputIndex = 0;
                memset(cInputString, 0x00, INPUT_MAX_SIZE);
            }
            else
            {
                if(cRxChar == '\r')
                {
                    // ignore char
                }
                // Check for backspace
                else if((cRxChar == '\b') || (cRxChar == 0x7F))
                {
                    if(ucInputIndex > 0)
                    {
                        ucInputIndex--;
                        cInputString[ucInputIndex] = '\0';
                    }
                }
                else
                {
                    // A character was entered
                    if((cRxChar >= ' ') && (cRxChar <= '~'))
                    {
                        if(ucInputIndex < INPUT_MAX_SIZE)
                        {
                            cInputString[ucInputIndex] = cRxChar;
                            ucInputIndex++;
                        }
                    }
                }
            }
            xSemaphoreGive(xTxMutex);
        }
    }
}
