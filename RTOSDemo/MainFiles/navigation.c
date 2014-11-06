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
#include "map.h"

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
void vStartNavTask(vtNavStruct *params, unsigned portBASE_TYPE uxPriority, vtLCDStruct *lcd, struct vtInfraredStruct *sensor, struct vtMotorStruct *motor)
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
		printf("error starting nav task\n");
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
		printf("Move forward\n");
		move = FORWARD;
	}

	return move;
}

/*
 * Generates a list representing all the walls in the map.
 * Returns a pointer to the first wall in the list.
*/
WallNode* generateWalls()
{
	WallNode *head;
	WallNode *curr;
	
	head = (struct Wall*)malloc(sizeof(struct Wall));
	curr = head;

	int i;
	for (i = 0; i < (numVerticies * 2); i = i + 2)
	{
		curr->startVertex[0] = verticies[i];
		curr->startVertex[1] = verticies[i+1];
		if (i + 2 < (numVerticies * 2))
		{
			curr->endVertex[0] = verticies[i+2];
			curr->endVertex[1] = verticies[i+3];
			curr->nextWall = (struct Wall*)malloc(sizeof(struct Wall));
		}
		else // The next wall is the first wall
		{
			curr->endVertex[0] = head->startVertex[0];
			curr->endVertex[1] = head->startVertex[1];
			curr->nextWall = head;
		}
		if (curr->startVertex[0] == curr->endVertex[0])
		{
			curr->length = abs(curr->startVertex[1] - curr->endVertex[1]);
		}
		else if (curr->startVertex[1] == curr->endVertex[1])
		{
			curr->length = abs(curr->startVertex[0] - curr->endVertex[0]);
		}
		else
		{
			printf("startx %d endx %d\n",curr->startVertex[0], curr->endVertex[0]);
			printf("starty %d endy %d\n",curr->startVertex[1], curr->endVertex[1]);
			printf("Error in map data!\n");
			VT_HANDLE_FATAL_ERROR(0);
		}
		curr->nextWall->prevWall = curr;
		curr = curr->nextWall;
	}

	return head;
}

// end of local functions

// Definitions of the states for the FSM below
const uint8_t fsmStateWaitForSensors = 0;
const uint8_t fsmStateMoving = 1;

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
	WallNode *currWall = NULL;

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

	// Before we can start the task, we need to interpret our map data
	// This variable will hold the first wall and will not change.
	const WallNode *wallList = generateWalls();

	#if DEBUG == 1
	// Check wall list
	WallNode *temp = wallList;
	printf("Start Vertex: %d,%d End Vertex: %d,%d Length: %d\n", temp->startVertex[0],temp->startVertex[1],temp->endVertex[0],temp->endVertex[1],temp->length);
	temp = temp->nextWall;
	while (temp != wallList)
	{
		printf("Start Vertex: %d,%d End Vertex: %d,%d Length: %d\n", temp->startVertex[0],temp->startVertex[1],temp->endVertex[0],temp->endVertex[1],temp->length);
		temp = temp->nextWall;
	}
	#endif

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
				if (currentState == fsmStateWaitForSensors)
				{
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
							printf("error with nav printing lcd msg?\n");
							VT_HANDLE_FATAL_ERROR(0);
						}
						uint8_t values[] = {currentMove, travelDistance};
						if (motors != NULL) {
							if (SendMotorValueMsg(motors,vtI2CMsgTypeMotorCommand,values,portMAX_DELAY) != pdTRUE) {
								printf("error sending msg from nav to motor task\n");
								VT_HANDLE_FATAL_ERROR(0);
							}
							printf("sent a msg from nav to motor task\n");
						}
						currentState = fsmStateMoving;
					}
				}
				break;
			}
			case vtNavMsgMotorData: {
				if (currentState == fsmStateMoving)
				{
					lastMove = msgBuffer.buf[0];
					travelDistance = msgBuffer.buf[1];
					printf("recieved distance report from mototrs\n");
					currentState = fsmStateWaitForSensors;
				}
				break;
			}
			default: {
				printf("bad nav msg\n");
				VT_HANDLE_FATAL_ERROR(getMsgType(&msgBuffer));
				break;
			}
		}
	}
}