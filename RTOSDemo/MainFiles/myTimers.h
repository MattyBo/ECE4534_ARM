#ifndef _MY_TIMERS_H
#define _MY_TIMERS_H
#include "lcdTask.h"
#include "i2cTemp.h"
#include "i2cInfrared.h"
#include "motor.h"
void startTimerForLCD(vtLCDStruct *vtLCDdata);
void startTimerForTemperature(vtTempStruct *vtTempdata);
void startTimerForInfrared(vtInfraredStruct *vtInfraredData);
void startTimerForMotor(vtMotorStruct *vtMotorData);
#endif