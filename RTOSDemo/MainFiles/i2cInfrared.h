#ifndef I2CINFRARED_TASK_H
#define I2CINFRARED_TASK_H
#include "vtI2C.h"
#include "lcdTask.h"
#include "navigation.h"
// Structure used to pass parameters to the task
// Do not touch...	   
typedef struct vtInfraredStruct {
	vtI2CStruct *dev;
	vtLCDStruct *lcdData;
	struct vtNavStruct *navData;
	xQueueHandle inQ;
} vtInfraredStruct;
// Maximum length of a message that can be received by this task
//#define vtInfraredMaxLen   (sizeof(portTickType))
#define vtInfraredMaxLen   7 // This is how long the message back from the rover is


// Public API
//
// Start the task
// Args:
//   sensorData: Data structure used by the task
//   uxPriority -- the priority you want this task to be run at
//   i2c: pointer to the data structure for an i2c task
//   lcd: pointer to the data structure for an LCD task (may be NULL)
//   nav: pointer to the data structure for a Navigation task (may be NULL)
void vStarti2cInfraredTask(vtInfraredStruct *sensorData,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c,vtLCDStruct *lcd, struct vtNavStruct *nav);
//
// Send a timer message to the Sensor task
// Args:
//   sensorData -- a pointer to a variable of type vtInfraredStruct
//   ticksElapsed -- number of ticks since the last message (this will be sent in the message)
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE SendInfraredTimerMsg(vtInfraredStruct *sensorData,portTickType ticksElapsed,portTickType ticksToBlock);
//
// Send a value message to the Sensor task
// Args:
//   sensorData -- a pointer to a variable of type vtInfraredStruct
//   msgType -- the type of the message to send
//   value -- The value to send
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE SendInfraredValueMsg(vtInfraredStruct *sensorData,uint8_t msgType,uint8_t* values,portTickType ticksToBlock);
#endif