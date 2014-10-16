#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H
#include "vtI2C.h"
#include "lcdTask.h"
// Structure used to pass parameters to the task
// Do not touch...
typedef struct __MotorStruct {
	vtI2CStruct *dev;
	vtLCDStruct *lcdData;
	xQueueHandle inQ;
} vtMotorStruct;
// Maximum length of a message that can be received by this task
#define vtMotorMaxLen   7
#define vtMotorExpectedLen 3

// Possible travel directions
#define FORWARD 0
#define RIGHT 1
#define LEFT 2
#define BACK 3

// Public API
//
// Start the task
// Args:
//   motorData: Data structure used by the task
//   uxPriority -- the priority you want this task to be run at
//   i2c: pointer to the data structure for an i2c task
//   lcd: pointer to the data structure for an LCD task (may be NULL)
void vStartMotorTask(vtMotorStruct *motorData,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c,vtLCDStruct *lcd);
//
// Send a timer message to the Motor task
// Args:
//   motorData -- a pointer to a variable of type vtMotorStruct
//   ticksElapsed -- number of ticks since the last message (this will be sent in the message)
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE SendMotorTimerMsg(vtMotorStruct *motorData,portTickType ticksElapsed,portTickType ticksToBlock);
//
// Send a value message to the Motor task
// Args:
//   motorData -- a pointer to a variable of type vtMotorStruct
//   msgType -- the type of the message to send
//   values -- The values to send
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE SendMotorValueMsg(vtMotorStruct *motorData,uint8_t msgType,uint8_t* values,portTickType ticksToBlock);
#endif