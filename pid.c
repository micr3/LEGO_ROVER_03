#include "pid.h"

PIDStruct _pid[8];

void REGULATOR_Init(PIDStruct *p, float Kp, float Kd, float Ki, float max_integral)
{
	//przypisanie do struct wartosci z aplikacji
	p->Kp=Kp;
	p->Ki=Ki;
	p->Kd=Kd;
	p->maxIntegral=max_integral;
}

float REGULATOR_Count(PIDStruct *p, float err)
{
	p->err=err;

	//Wielkosc P
	float Pout= p->Kp *err;

	//Wielkosc I

	float _integral= p->integral+ err * p->dt;
			if(_integral >= p->maxIntegral)
				_integral=maxIntegral;
	p->integral=_integral;
	float Iout= p->Ki *_integral;

	// Wielkosc D
	float derivative= (err- p->errPrev)/p->dt;
	float Dout= p->Kd *derivative;

	p->pid= Pout + Iout + Dout; //wzor na PID
	p->errPrev=err;
	return p->pid;
}
