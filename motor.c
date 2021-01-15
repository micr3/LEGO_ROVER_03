
#include "motor.h"

motorStruct motor[8];
motorStruct *motorWsk[8];

void MOTOR_Init(motorStruct *m, uint8_t number)
{
	m->actEnc = 0;
	m->actRot = 0;
	m->actSpeed = 0;
	m->nMotor = number;
	m->pid = &_pid[number - 1];
	m->state = motorInit;
	m->enable = 0;
	motorWsk[number - 1] = m;
}

void MOTOR_Enable(motorStruct *m)
{
	m->state = motorEnable;
}
void MOTOR_Disable(motorStruct *m)
{
	m->state = motorDisable;
}

void MOTOR_SetPWM(motorStruct *m, int16_t _pwm)
{
	if (m->enable)
	{
		m->state = motorPWMcontrol;
		m->setPWM = (float)_pwm;
	}
}
void MOTOR_SetSpeed(motorStruct *m, float _speed)
{
	if (m->enable)
	{
		m->state = motorTimeControl;
		m->setSpeed = _speed;
	}
}
void MOTOR_SetTime(motorStruct *m, int16_t _pwm, float _time)
{
	if (m->enable)
	{
		m->state = motorTimeControl;
		m->setPWM = (float)_pwm;
		m->setTime = _time;
		m->oldTime = millis();
	}
}


void MOTOR_HardStop(motorStruct *m)
{
	SHIFT_Set(m->nMotor, motorHardStop);
	
}
void MOTOR_SoftStop(motorStruct *m)
{
	SHIFT_Set(m->nMotor, motorSoftStop);
}


void MOTOR_RawSter(motorStruct *m)
{
	uint8_t dir = 0;
	if (m->setPWM == 0)
		dir = motorSoftStop;
	else if (m->setPWM > 0 && m->setPWM <= 1000)
		dir = motorClockWise;
	else if (m->setPWM >= -1000 && m->setPWM < 0)
		dir = motorContraClockWise;
	int16_t pwm = 0;
	if (m->setPWM < 0.0)
		pwm = -((int)m->setPWM);
	else 
		pwm = ((int)m->setPWM);
	SHIFT_Set(m->nMotor, dir);	
	PWM_Set(m->nMotor, pwm);
}

void MOTOR_CyclicTask()
{
	static uint32_t oldTime = 0;
	if (timeIsUp(oldTime, MOTOR_REFRESH))
		for (uint8_t i = 0; i < 8; ++i)
			switch (motor[i].state)
			{
			case motorNoInit: 
				MOTOR_SoftStop(&motor[i]); 
				break;
			case motorInit: 
				motor[i].enable = 0; 
				break;
			case motorDisable: 
				MOTOR_SoftStop(&motor[i]); motor[i].enable = 0; 
				break;
			case motorEnable: 
				MOTOR_SoftStop(&motor[i]); motor[i].enable = 1; 
				break;
			case motorPWMcontrol: 
				MOTOR_RawSter(&motor[i]); 
			case motorSpeedControl: /////////PID based on speed
				break;
			case motorTimeControl:
				if (!timeIsUp(motor[i].oldTime, motor[i].setTime))
					MOTOR_RawSter(&motor[i]);
				else 
					MOTOR_SoftStop(&motor[i]);
				break;
			case motorRotControl: ///////// PID based on rotation
				break;
			}
}