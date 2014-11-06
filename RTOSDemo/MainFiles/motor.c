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
#include "LCDtask.h"
#include "motor.h"
#include "I2CTaskMsgTypes.h"

#include "lpc17xx_gpio.h"
#define DEBUG 0

/* *********************************************** */
// definitions and data structures that are private to this file
// Length of the queue to this task
#define vtMotorQLen 10 
// actual data structure that is sent in a message
typedef struct __vtMotorMsg {
	uint8_t msgType;
	uint8_t	length;	 // Length of the message to be printed
	uint8_t buf[vtMotorMaxLen+1]; // On the way in, message to be sent, on the way out, message received (if any)
} vtMotorMsg;

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

/* The Motor task. */
static portTASK_FUNCTION_PROTO( motorUpdateTask, pvParameters );

/*-----------------------------------------------------------*/
// Public API
void vStartMotorTask(vtMotorStruct *params,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c,vtLCDStruct *lcd, struct vtNavStruct *nav)
{
	// Create the queue that will be used to talk to this task
	if ((params->inQ = xQueueCreate(vtMotorQLen,sizeof(vtMotorMsg))) == NULL) {
		printf("error creating motor task\n");
		VT_HANDLE_FATAL_ERROR(0);
	}
	/* Start the task */
	portBASE_TYPE retval;
	params->dev = i2c;
	params->lcdData = lcd;
	params->navData = nav;
	if ((retval = xTaskCreate( motorUpdateTask, ( signed char * ) "Motor", i2cSTACK_SIZE, (void *) params, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}

portBASE_TYPE SendMotorTimerMsg(vtMotorStruct *motorData,portTickType ticksElapsed,portTickType ticksToBlock)
{
	if (motorData == NULL) {
		printf("error sending motor timer message\n");
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtMotorMsg motorBuffer;
	motorBuffer.length = sizeof(ticksElapsed);
	if (motorBuffer.length > vtMotorMaxLen) {
		// no room for this message
		printf("error no room for motor timer message\n");
		VT_HANDLE_FATAL_ERROR(motorBuffer.length);
	}
	memcpy(motorBuffer.buf,(char *)&ticksElapsed,sizeof(ticksElapsed));
	motorBuffer.msgType = MotorMsgTypeTimer;
	return(xQueueSend(motorData->inQ,(void *) (&motorBuffer),ticksToBlock));
}

portBASE_TYPE SendMotorValueMsg(vtMotorStruct *motorData,uint8_t msgType,uint8_t* values,portTickType ticksToBlock)
{
	vtMotorMsg motorBuffer;

	if (motorData == NULL) {
		printf("error sending motor message\n");
		VT_HANDLE_FATAL_ERROR(0);
	}
	motorBuffer.length = vtMotorMaxLen;
	if (motorBuffer.length > vtMotorMaxLen) {
		// no room for this message
		printf("error no room for motor message\n");
		VT_HANDLE_FATAL_ERROR(motorBuffer.length);
	}
	memcpy(motorBuffer.buf,values,vtMotorExpectedLen);
	motorBuffer.msgType = msgType;
	return(xQueueSend(motorData->inQ,(void *) (&motorBuffer),ticksToBlock));
}

// End of Public API
/*-----------------------------------------------------------*/
int getMsgType(vtMotorMsg *Buffer)
{
	return(Buffer->msgType);
}

// I2C commands for the infrared sensor
const uint8_t i2cCmdMotorMove[]= {0xF1, 0x01};
const uint8_t i2cCmdMotorError[]= {0xF1, 0xF0};

void sendErrorMsg(vtI2CStruct *devPtr) {
	if (vtI2CEnQ(devPtr,vtI2CMsgTypeMotorCommand,0x4F,sizeof(i2cCmdMotorError),i2cCmdMotorError,vtMotorMaxLen) != pdTRUE) {
		printf("error sending motor error message\n");
		VT_HANDLE_FATAL_ERROR(0);
	}
//	vtI2CEnQ(devPtr,vtI2CMsgTypeMotorCommand,0x4F,sizeof(i2cCmdMotorError),i2cCmdMotorError,vtMotorMaxLen);
}
// end of I2C command definitions

// Definitions of the states for the FSM below
const uint8_t fsmStateWaitForNav = 0;
const uint8_t fsmStateMotorStatus = 1;

// This is the actual task that is run
static portTASK_FUNCTION( motorUpdateTask, pvParameters )
{
	uint8_t distance = 0;
	uint8_t direction = FORWARD;
	// Get the parameters
	vtMotorStruct *param = (vtMotorStruct *) pvParameters;
	// Get the I2C device pointer
	vtI2CStruct *devPtr = param->dev;
	// Get the LCD information pointer
	vtLCDStruct *lcdData = param->lcdData;
	// Get the Navigation task pointer
	vtNavStruct *navData = param->navData;
	// String buffer for printing
	char lcdBuffer[vtLCDMaxLen+1];
	// Buffer for receiving messages
	vtMotorMsg msgBuffer;
	uint8_t currentState;

	// Assumes that the I2C device (and thread) have already been initialized

	// This task is implemented as a Finite State Machine.  The incoming messages are examined to see
	//   whether or not the state should change.

	currentState = fsmStateWaitForNav;
	// Like all good tasks, this should never exit
	for(;;)
	{
		// Wait for a message from either a timer or from an I2C operation
		if (xQueueReceive(param->inQ,(void *) &msgBuffer,portMAX_DELAY) != pdTRUE) {
			printf("error getting a motor message\n");
			//VT_HANDLE_FATAL_ERROR(0);
			continue;
		}

		// Now, based on the type of the message and the state, we decide on the new state and action to take
		switch(getMsgType(&msgBuffer)) {
			case MotorMsgTypeTimer: {
				if (currentState == fsmStateMotorStatus) {
					#if DEBUG == 1
					GPIO_SetValue(0,0x20000);
					#endif
					// We should have gotten a status message by now. Send an error message 
					sendErrorMsg(devPtr);
					#if DEBUG == 1
					GPIO_ClearValue(0,0x20000);
					#endif
				} else {
					// do nothing for now
				}
				break;
			}
			case vtI2CMsgTypeMotorCommand: {
				if (currentState == fsmStateWaitForNav) {
					// Send a motor command
					uint8_t motorCommand[] = {0xF1, msgBuffer.buf[0]};
					if (vtI2CEnQ(devPtr,vtI2CMsgTypeMotorCommand,0x4F,sizeof(motorCommand),motorCommand,vtMotorMaxLen) != pdTRUE) {
						printf("problem sending motor command\n");
						VT_HANDLE_FATAL_ERROR(0);
					}		
					currentState = fsmStateMotorStatus;
				}
				break;
			}
			case vtI2CMsgTypeMotorStatus: {
				if (currentState == fsmStateMotorStatus) {
					// Ensure msg was intended for this task. 
					// If not, break out and send an error msg. We should be getting a motor status
					if (msgBuffer.buf[0] != 0xF1) {
						sendErrorMsg(devPtr);
					}
					// Check msg integrity. 
					uint8_t i;
					uint8_t badMsg = 0;
					for (i = 1; i < vtMotorExpectedLen; i++) {
						if (msgBuffer.buf[i] == 0xF1 || msgBuffer.buf[i] == 0xF0)  badMsg = 1;
					}
					// If we've gotten a bad msg, send an error msg so we get one.
					if (badMsg) {
						sendErrorMsg(devPtr);
					}
	
					direction = msgBuffer.buf[1];
					distance = msgBuffer.buf[2];
					// Send data to navigation task
					if (navData != NULL) {
						if (SendNavValueMsg(navData,vtNavMsgMotorData,&msgBuffer.buf[1],portMAX_DELAY) != pdTRUE) {
							printf("error sending motor data to nav\n");
							VT_HANDLE_FATAL_ERROR(0);
						}
					}
					
//					#if PRINTF_VERSION == 1
//					printf("Dis %d cm, Dir %d\n",distance,direction);
//					sprintf(lcdBuffer,"Dis %d cm, Dir %d\n",distance,direction);
//					#else
//					// we do not have full printf (so no %f) and therefore need to print out integers
//					printf("Dis %d cm, Dir %d\n",distance,direction);
//					sprintf(lcdBuffer,"Dis %d cm, Dir %d\n",distance,direction);
//					#endif
//					if (lcdData != NULL) {
//						if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,portMAX_DELAY) != pdTRUE) {
//							VT_HANDLE_FATAL_ERROR(0);
//						}
//					}
				} else {
					// do nothing for now
				}
				currentState = fsmStateWaitForNav;
				break;
			}
			default: {
				printf("bad motor msg\n");
				VT_HANDLE_FATAL_ERROR(getMsgType(&msgBuffer));
				break;
			}
		}
	}
}