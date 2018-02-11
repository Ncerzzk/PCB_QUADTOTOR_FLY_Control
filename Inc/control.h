#ifndef __CONTROL_H
#define __CONTROL_H
#include "base.h"


//#define BRUSHLESS

#define I_TIME 0.002f
#define DUTY_MAX 100

#define Limit(value,max)     if(value>max)value=max;else if(value<-max)value=-max

typedef struct{
	float KP;
	float KD;
	float KI;
	float i;
	float last_err;
	float i_max;
	float last_d;
}PID_S;


#define clear_i(PID) PID.i=0//;PID.i_max=DUTY_MAX/10/(abs(PID.KI)==0.0?0.001:abs(PID.KI))

typedef enum{
	FLY_WAIT=0,
	FLY=1,
	LAND=2,
	STOP=3,
}Fly_State;

extern Fly_State  State;
void Fly_Control();

#endif
