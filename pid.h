#pragma once


#include "main.h"
#include "stm32f4xx_hal.h"
#include "basicFunction.h"

typedef struct pidDataStruct
{
	float maxIntegral;
	float Kp, Kd, Ki;
	float err, 
		errPrev, integral;
	float pid;
	uint16_t dt;
} PIDStruct;
	
extern PIDStruct _pid[8];


void REGULATOR_Init(PIDStruct *p, float Kp, float Kd, float Ki, float max_integral);
float REGULATOR_Count(PIDStruct *p, float err);