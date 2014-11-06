#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"
#include "semphr.h"

/* include files. */
#include "vtUtilities.h"
#include "i2cInfrared.h"
#include "I2CTaskMsgTypes.h"

#include "lpc17xx_gpio.h"
#define DEBUG 1

/* *********************************************** */
// definitions and data structures that are private to this file
// Length of the queue to this task
#define vtSensorQLen 10 
// actual data structure that is sent in a message
typedef struct __vtInfraredMsg {
	uint8_t msgType;
	uint8_t	length;	 // Length of the message to be printed
	uint8_t buf[vtInfraredMaxLen+1]; // On the way in, message to be sent, on the way out, message received (if any)
} vtInfraredMsg;

// I have set this to a large stack size because of (a) using printf() and (b) the depth of function calls
//   for some of the i2c operations	-- almost certainly too large, see LCDTask.c for details on how to check the size
#define baseStack 3
#if PRINTF_VERSION == 1
#define i2cSTACK_SIZE		((baseStack+5)*configMINIMAL_STACK_SIZE)
#else
#define i2cSTACK_SIZE		(baseStack*configMINIMAL_STACK_SIZE)
#endif

// end of defs
/* *********************************************** */

/* The i2cSensor task. */
static portTASK_FUNCTION_PROTO( vi2cSensorUpdateTask, pvParameters );

/*-----------------------------------------------------------*/
// Public API
void vStarti2cInfraredTask(vtInfraredStruct *params,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c,vtLCDStruct *lcd, struct vtNavStruct *nav)
{
	// Create the queue that will be used to talk to this task
	if ((params->inQ = xQueueCreate(vtSensorQLen,sizeof(vtInfraredMsg))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	/* Start the task */
	portBASE_TYPE retval;
	params->dev = i2c;
	params->lcdData = lcd;
	params->navData = nav;
	if ((retval = xTaskCreate( vi2cSensorUpdateTask, ( signed char * ) "i2cInfrared", i2cSTACK_SIZE, (void *) params, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}

portBASE_TYPE SendInfraredTimerMsg(vtInfraredStruct *sensorData,portTickType ticksElapsed,portTickType ticksToBlock)
{
	if (sensorData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtInfraredMsg sensorBuffer;
	sensorBuffer.length = sizeof(ticksElapsed);
	if (sensorBuffer.length > vtInfraredMaxLen) {
		// no room for this message
		VT_HANDLE_FATAL_ERROR(sensorBuffer.length);
	}
	memcpy(sensorBuffer.buf,(char *)&ticksElapsed,sizeof(ticksElapsed));
	sensorBuffer.msgType = SensorMsgTypeTimer;
	return(xQueueSend(sensorData->inQ,(void *) (&sensorBuffer),ticksToBlock));
}

portBASE_TYPE SendInfraredValueMsg(vtInfraredStruct *sensorData,uint8_t msgType,uint8_t* values,portTickType ticksToBlock)
{
	vtInfraredMsg sensorBuffer;

	if (sensorData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	sensorBuffer.length = vtInfraredMaxLen;
	if (sensorBuffer.length > vtInfraredMaxLen) {
		// no room for this message
		VT_HANDLE_FATAL_ERROR(sensorBuffer.length);
	}
	memcpy(sensorBuffer.buf,values,vtInfraredMaxLen);
	sensorBuffer.msgType = msgType;
	return(xQueueSend(sensorData->inQ,(void *) (&sensorBuffer),ticksToBlock));
}

// End of Public API
/*-----------------------------------------------------------*/
int getMsgType(vtInfraredMsg *Buffer)
{
	return(Buffer->msgType);
}

// I2C commands for the infrared sensor
const uint8_t i2cCmdReadVals[]= {0xF0, 0xBB};

// end of I2C command definitions

// This is the actual task that is run
static portTASK_FUNCTION( vi2cSensorUpdateTask, pvParameters )
{
	// Get the parameters
	vtInfraredStruct *param = (vtInfraredStruct *) pvParameters;
	// Get the I2C device pointer
	vtI2CStruct *devPtr = param->dev;
	// Get the LCD information pointer
	vtLCDStruct *lcdData = param->lcdData;
	// Get the Navigation task pointer
	vtNavStruct *navData = param->navData;
	// String buffer for printing
	char lcdBuffer[vtLCDMaxLen+1];
	// Buffer for receiving messages
	vtInfraredMsg msgBuffer;

	// Assumes that the I2C device (and thread) have already been initialized

	// Like all good tasks, this should never exit
	for(;;)
	{
		// Wait for a message from either a timer or from an I2C operation
		if (xQueueReceive(param->inQ,(void *) &msgBuffer,portMAX_DELAY) != pdTRUE) {
			printf("error getting sensor msg\n");
			//VT_HANDLE_FATAL_ERROR(0);
			continue;
		}

		// Now, based on the type of the message, we decide on the action to take
		switch(getMsgType(&msgBuffer)) {
			case SensorMsgTypeTimer: {
				#if DEBUG == 1
				GPIO_SetValue(0,0x20000);
				#endif 
				// Send query to the sensors
				if (vtI2CEnQ(devPtr,vtI2CMsgTypeSensorRead,0x4F,sizeof(i2cCmdReadVals),i2cCmdReadVals,vtInfraredMaxLen) != pdTRUE) {
					// If we can't get a complete message from the rover in time, give up and try again
					printf("couldn't get sensor data back in time\n");
					break;
				}
				printf("sent sensor query\n");
				#if DEBUG == 1
				GPIO_ClearValue(0,0x20000);
				#endif
				break;
			}
			case vtI2CMsgTypeSensorRead: {
				// Ensure msg was intended for this task. If not, break out and wait for next sensor msg
				if (msgBuffer.buf[0] != 0xF0) {
					break;
				}
				// Check msg integrity. 
				uint8_t i;
				uint8_t badMsg = 0;
				for (i = 1; i < vtInfraredMaxLen; i++) {
					if (msgBuffer.buf[i] == 0xF0 || msgBuffer.buf[i] == 0xF1)  badMsg = 1;
				}
				// If we've gotten a bad msg, break and wait for next sensor msg
				if (badMsg) {
					break;
				}

				#if PRINTF_VERSION == 1
				printf("Distance1 %d inches\n",distance1);
				sprintf(lcdBuffer,"d= %f inches",distance1);
				#else
				// we do not have full printf (so no %f) and therefore need to print out integers
				printf("Distance1 %d inches\n",msgBuffer.buf[1]);
				printf("Distance2 %d inches\n",msgBuffer.buf[2]);
				printf("Distance3 %d inches\n",msgBuffer.buf[3]);
				printf("Distance4 %d inches\n",msgBuffer.buf[4]);
				printf("Distance5 %d inches\n",msgBuffer.buf[5]);
				printf("Distance6 %d inches\n",msgBuffer.buf[6]);
				#endif
//				if (lcdData != NULL) {
//					for (i = 0; i < vtInfraredMaxLen; i++) {
//						sprintf(lcdBuffer,"d%d=%d inches",i,msgBuffer.buf[i]);
//						if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,portMAX_DELAY) != pdTRUE) {
//							VT_HANDLE_FATAL_ERROR(0);
//						}
//						if (distance1 != 0) {
//							if (SendLCDGraphMsg(lcdData,distance1,portMAX_DELAY) != pdTRUE) {
//								VT_HANDLE_FATAL_ERROR(0);
//							}
//						}
//					}
//				}

				// Send data to navigation task
				if (navData != NULL) {
					if (SendNavValueMsg(navData,vtNavMsgSensorData,&msgBuffer.buf[1],portMAX_DELAY) != pdTRUE) {
						printf("error sending sensor data to nav\n");
						VT_HANDLE_FATAL_ERROR(0);
					}
				}
				
				break;
			}
			default: {
				printf("bad sensor msg\n");
				VT_HANDLE_FATAL_ERROR(getMsgType(&msgBuffer));
				break;
			}
		}
	}
}