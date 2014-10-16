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
#include "navigation.h"
#include "i2ctaskmsgtypes.h"

#include "lpc17xx_gpio.h"
#define DEBUG 0

/* *********************************************** */
// definitions and data structures that are private to this file
// Length of the queue to this task
#define vtNavQLen 10 
// actual data structure that is sent in a message
typedef struct __vtMotorMsg {
	uint8_t msgType;
	uint8_t	length;	 // Length of the message to be printed
	uint8_t buf[vtNavMaxLen+1]; // On the way in, message to be sent, on the way out, message received (if any)
} vtNavMsg;

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
static portTASK_FUNCTION_PROTO( navUpdateTask, pvParameters );

/*-----------------------------------------------------------*/
// Public API
void vStartNavTask(vtNavStruct *params, unsigned portBASE_TYPE uxPriority, vtLCDStruct *lcd, struct vtInfraredStruct *sensor, vtMotorStruct *motor)
{
	// Create the queue that will be used to talk to this task
	if ((params->inQ = xQueueCreate(vtNavQLen,sizeof(vtNavMsg))) == NULL) {
		printf("error creating nav task\n");
		VT_HANDLE_FATAL_ERROR(0);
	}
	/* Start the task */
	portBASE_TYPE retval;
	params->lcdData = lcd;
	params->sensorData = sensor;
	params->motorData = motor;
	if ((retval = xTaskCreate( navUpdateTask, ( signed char * ) "Navigation", i2cSTACK_SIZE, (void *) params, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}

portBASE_TYPE SendNavValueMsg(vtNavStruct *navData,uint8_t msgType,uint8_t* values,portTickType ticksToBlock)
{
	vtNavMsg navBuffer;

	if (navData == NULL) 
	{
		printf("error sending motor message\n");
		VT_HANDLE_FATAL_ERROR(0);
	}
	navBuffer.length = vtNavMaxLen;
	if (navBuffer.length > vtNavMaxLen) 
	{
		// no room for this message
		printf("error no room for motor message\n");
		VT_HANDLE_FATAL_ERROR(navBuffer.length);
	}
	memcpy(navBuffer.buf,values,vtNavMaxLen);
	navBuffer.msgType = msgType;
	return(xQueueSend(navData->inQ,(void *) (&navBuffer),ticksToBlock));
}

// End of Public API
/*-----------------------------------------------------------*/
int getMsgType(vtNavMsg *Buffer)
{
	return(Buffer->msgType);
}

// This method figures out where the rover needs to go next. Right now, it simply makes decisions based on the
// wall around it with no regard to the map.
uint8_t determineMove(uint8_t front, uint8_t fRight, uint8_t bRight, uint8_t back, uint8_t bLeft, uint8_t fLeft) {
	uint8_t move = NOMOVE;
	// There is a wall in front of me
	if (front <= MIN_DISTANCE) 
	{
		// We can turn right
		if (fRight > MIN_DISTANCE && bRight > MIN_DISTANCE) 
		{
			printf("Turning right\n");
			move = RIGHT;
		}
		// We can turn left
		else if (fLeft >  MIN_DISTANCE && bLeft > MIN_DISTANCE) 
		{
			printf("Turning left\n");
			move = LEFT;
		}
		// We need to turn around
		else if (back > MIN_DISTANCE) 
		{
			printf("Turning around\n");
			move = BACK;
		}
		// We're in a box and can't do anything...
		else 
		{
			printf("I'M IN A BOX\n");
			move = NOMOVE;
		}
	}
	// There is no wall in front of me. Move forward, parallel to the closest wall.
	else {
		// The right wall is closer so go parallel to that
		if (fRight < fLeft)
		{
			// Make sure we aren't too close to the wall
			// ADD THIS LOGIC


			// The rover is not parallel to the wall, fix it
			if (abs(fRight - bRight) >= 1) 
			{
				// The front is farther away than the back
				if (fRight > bRight) 
				{
					move = RIGHT;
					printf("I am going forward but fixing parallel right by turning right\n");
				}
				// The back is farther away than the right
				else 
				{
					move = LEFT;
					printf("I am going forward but fixing parallel right by turning left\n");
				}
			}
			// We are parallel (or basically parallel) so go forward
			else
			{
				move = FORWARD;
				printf("I am moving forward, parallel to the right wall\n");
			}			
		}
		// The left wall is closer so go parallel to that
		else 
		{
			// The rover is not parallel to the wall, fix it
			if (abs(fLeft - bLeft) >= 1) 
			{
				// The front is farther away than the back
				if (fLeft > bLeft) 
				{
					move = LEFT;
					printf("I am going forward but fixing parallel left by turning left\n");
				}
				// The back is farther away than the right
				else 
				{
					move = RIGHT;
					printf("I am going forward but fixing parallel left by turning right\n");
				}
			}
			// We are parallel (or close enough) so go forward
			else
			{
				move = FORWARD;
				printf("I am moving forward, parallel to the left wall\n");
			}
		}
	}

	return move;
}

// end of I2C command definitions

// Definitions of the states for the FSM below
const uint8_t fsmStateWaitForSensors = 0;
const uint8_t fsmStateDetermineMove = 1;
const uint8_t fsmStateParallel = 2;
const uint8_t fsmStateTooClose = 3;
const uint8_t fsmStateTooFar = 4;
const uint8_t fsmStateWallInFront = 5;

// This is the actual task that is run
static portTASK_FUNCTION( navUpdateTask, pvParameters )
{
	uint8_t distanceFront = 0;
	uint8_t distanceFRight= 0;
	uint8_t distanceBRight = 0;
	uint8_t distanceBack = 0;
	uint8_t distanceBLeft = 0;
	uint8_t distanceFLeft = 0;
	uint8_t currentMove = NOMOVE;
	uint8_t lastMove = NOMOVE;
	uint8_t travelDistance = 0;

	// Get the parameters
	vtNavStruct *param = (vtNavStruct *) pvParameters;
	// Get the Infrared information pointer
	vtInfraredStruct *sensors = param->sensorData;
	// Get the Motor information pointer
	vtMotorStruct *motors = param->motorData;
	// Get the LCD information pointer
	vtLCDStruct *lcdData = param->lcdData;
	// String buffer for printing
	char lcdBuffer[vtLCDMaxLen+1];
	// Buffer for receiving messages
	vtNavMsg msgBuffer;
	uint8_t currentState;

	// Assumes that the I2C device (and thread) have already been initialized

	// This task is implemented as a Finite State Machine.  The incoming messages are examined to see
	//   whether or not the state should change.

	currentState = fsmStateWaitForSensors;
	// Like all good tasks, this should never exit
	for(;;)
	{
		// Wait for a message from either a timer or from an I2C operation
		if (xQueueReceive(param->inQ,(void *) &msgBuffer,portMAX_DELAY) != pdTRUE) {
			printf("error getting a nav message\n");
			VT_HANDLE_FATAL_ERROR(0);
			//continue;
		}

		// Now, based on the type of the message and the state, we decide on the new state and action to take
		switch(getMsgType(&msgBuffer)) {
			case vtNavMsgSensorData: {
				distanceFront = msgBuffer.buf[0];
				distanceFRight = msgBuffer.buf[1];
				distanceBRight = msgBuffer.buf[2];
				distanceBack = msgBuffer.buf[3];
				distanceBLeft = msgBuffer.buf[4];
				distanceFLeft = msgBuffer.buf[5];
				currentMove = determineMove(distanceFront,distanceFRight,distanceBRight,distanceBack,distanceBLeft,distanceFLeft);
				if (lcdData != NULL) {
					sprintf(lcdBuffer,"%d,%d,%d,%d,%d,%d",distanceFront,distanceFRight,distanceBRight,distanceBack,distanceBLeft,distanceFLeft);
					if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,portMAX_DELAY) != pdTRUE) {
						VT_HANDLE_FATAL_ERROR(0);
					}
					switch (currentMove)
					{
					    case NOMOVE:
							sprintf(lcdBuffer, "I can't move");
							break;
						case FORWARD:
							sprintf(lcdBuffer, "Move forward");
							break;
						case LEFT:
							sprintf(lcdBuffer, "Turn left");
							break;
						case RIGHT:
							sprintf(lcdBuffer, "Turn right");
							break;
						case BACK:
							sprintf(lcdBuffer, "Turn around");
							break;
						default:
							sprintf(lcdBuffer, "CONFUSION :(");
							break;
					}
					if (SendLCDPrintMsg(lcdData,strnlen(lcdBuffer,vtLCDMaxLen),lcdBuffer,portMAX_DELAY) != pdTRUE) {
						VT_HANDLE_FATAL_ERROR(0);
					}
				}
				break;
			}
			case vtNavMsgMotorCmd: {
				// I should never receive this type of message
				printf("Nav task received a motor cmd\n");
				VT_HANDLE_FATAL_ERROR(0);
				break;
			}
			default: {
				VT_HANDLE_FATAL_ERROR(getMsgType(&msgBuffer));
				break;
			}
		}
	}
}