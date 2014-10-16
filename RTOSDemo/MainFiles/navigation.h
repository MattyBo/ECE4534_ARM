#ifndef NAV_TASK_H
#define NAV_TASK_H
#include "lcdTask.h"
#include "motor.h"
#include "i2cInfrared.h"
// Structure used to pass parameters to the task
// Do not touch...
typedef struct vtNavStruct {
	vtLCDStruct *lcdData;
	struct vtInfraredStruct *sensorData;
	vtMotorStruct *motorData;
	xQueueHandle inQ;
} vtNavStruct;
// Maximum length of a message that can be received by this task
#define vtNavMaxLen   7

// Possible travel directions
#define FORWARD 0
#define RIGHT 1
#define LEFT 2
#define BACK 3
#define NOMOVE 10

// Min distance between the rover and the closest wall required to move forward/turn (in inches)
#define MIN_DISTANCE 12

// Public API
//
// Start the task
// Args:
//   navData: Data structure used by the task
//   uxPriority -- the priority you want this task to be run at
//   i2c: pointer to the data structure for an i2c task
//   lcd: pointer to the data structure for an LCD task (may be NULL)
//   sensor: pointer to the data structure for a Sensor task (may be NULL)
//   motor: pointer to the data structure for a Motor task (may be NULL)
void vStartNavTask(vtNavStruct *navData, unsigned portBASE_TYPE uxPriority, vtLCDStruct *lcd, struct vtInfraredStruct *sensor, vtMotorStruct *motor);

//
// Send a value message to the Nav task
// Args:
//   navData -- a pointer to a variable of type vtNavStruct
//   msgType -- the type of the message to send
//   values -- The values to send
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE SendNavValueMsg(vtNavStruct *navData,uint8_t msgType,uint8_t* values,portTickType ticksToBlock);
#endif