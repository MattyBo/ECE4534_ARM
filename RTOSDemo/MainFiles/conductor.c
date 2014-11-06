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
#include "vtI2C.h"
#include "i2cInfrared.h"
#include "motor.h"
#include "I2CTaskMsgTypes.h"
#include "conductor.h"

/* *********************************************** */
// definitions and data structures that are private to this file

// I have set this to a large stack size because of (a) using printf() and (b) the depth of function calls
//   for some of the i2c operations	-- almost certainly too large, see LCDTask.c for details on how to check the size
#define INSPECT_STACK 1
#define baseStack 2
#if PRINTF_VERSION == 1
#define conSTACK_SIZE		((baseStack+5)*configMINIMAL_STACK_SIZE)
#else
#define conSTACK_SIZE		(baseStack*configMINIMAL_STACK_SIZE)
#endif
// end of defs
/* *********************************************** */

/* The i2cTemp task. */
static portTASK_FUNCTION_PROTO( vConductorUpdateTask, pvParameters );

/*-----------------------------------------------------------*/
// Public API
void vStartConductorTask(vtConductorStruct *params,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c,vtInfraredStruct *sensorData, vtMotorStruct *motorData)
{
	/* Start the task */
	portBASE_TYPE retval;
	params->dev = i2c;
	params->sensorData = sensorData;
	params->motorData = motorData;
	if ((retval = xTaskCreate( vConductorUpdateTask, ( signed char * ) "Conductor", conSTACK_SIZE, (void *) params, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}

// End of Public API
/*-----------------------------------------------------------*/

// This is the actual task that is run
static portTASK_FUNCTION( vConductorUpdateTask, pvParameters )
{
	uint8_t rxLen, status;
	uint8_t Buffer[vtI2CMLen];
	uint8_t *valPtr = &(Buffer[0]);
	// Get the parameters
	vtConductorStruct *param = (vtConductorStruct *) pvParameters;
	// Get the I2C device pointer
	vtI2CStruct *devPtr = param->dev;
	// Get the LCD information pointer
	vtInfraredStruct *infraredData = param->sensorData;
	vtMotorStruct *motorData = param->motorData;
	uint8_t recvMsgType;

	// Like all good tasks, this should never exit
	for(;;)
	{
		// Wait for a message from an I2C operation
		if (vtI2CDeQ(devPtr,vtI2CMLen,Buffer,&rxLen,&recvMsgType,&status) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}

		// Decide where to send the message 
		//   This just shows going to one task/queue, but you could easily send to
		//   other Q/tasks for other message types
		// This isn't a state machine, it is just acting as a router for messages
		switch(recvMsgType) {
			case vtI2CMsgTypeSensorRead: {
				printf("sending msg from conductor to sensor task\n");
				SendInfraredValueMsg(infraredData,recvMsgType,Buffer,portMAX_DELAY);
				printf("sent msg from conductor to sensor task\n");
				break;
			}
			case vtI2CMsgTypeMotorCommand: {
				printf("sending msg from conductor to motor task\n");
			 	SendMotorValueMsg(motorData,vtI2CMsgTypeMotorStatus,Buffer,portMAX_DELAY);
				printf("sending msg from conductor to motor task\n");
				break;
			}
			default: {
				printf("bad conductor msg\n");
				VT_HANDLE_FATAL_ERROR(recvMsgType);
				break;
			}
		}
	}
}