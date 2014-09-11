#ifndef _MY_TIMERS_H
#define _MY_TIMERS_H
#include "lcdTask.h"
#include "i2cSensor.h"
void startTimerForLCD(vtLCDStruct *vtLCDdata);
void startTimerForSensor(vtTempStruct *vtTempdata);
#endif