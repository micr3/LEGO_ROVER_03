#pragma once

#include "main.h"
#include "stm32f4xx_hal.h"
#include "pid.h"


#define M1	motor[0]
#define M2	motor[1]
#define M3	motor[2]
#define M4	motor[3]
#define M5	motor[4]
#define M6	motor[5]
#define M7	motor[6]
#define M8	motor[7]
#define M(i) motorWsk[(i)-1]

enum motorState {
	motorNoInit       = 0,
	motorInit         = 1,
	motorDisable      = 2,
	motorEnable       = 3,
	motorPWMcontrol   = 4,
	motorSpeedControl = 5,
	motorTimeControl  = 6,
	motorRotControl   = 7
};

enum motorDriver
{
	motorSoftStop        = 0b00,
	motorClockWise       = 0b01, 
	motorContraClockWise = 0b10, 
	motorHardStop        = 0b11
};


typedef struct motorDataStruct {
	uint8_t nMotor;
	
	uint32_t actEnc, actRot;    	//imp, rot
	float actSpeed;    		//rot per min
	uint32_t oldTime;
	
	float setSpeed, setPWM, setTime;
	
	uint8_t state, enable;
	PIDStruct *pid;
} motorStruct;

extern motorStruct motor[];
extern motorStruct *motorWsk[];

void MOTOR_Init(motorStruct *m, uint8_t number);
void MOTOR_Enable(motorStruct *m);
void MOTOR_Disable(motorStruct *m);

void MOTOR_SetPWM(motorStruct *m, int16_t _pwm);
void MOTOR_SetSpeed(motorStruct *m, float _speed);
void MOTOR_SetTime(motorStruct *m, int16_t _pwm, float _time);
void MOTOR_HardStop(motorStruct *m);
void MOTOR_SoftStop(motorStruct *m);

void MOTOR_RawSter(motorStruct *m);

void MOTOR_CyclicTask();