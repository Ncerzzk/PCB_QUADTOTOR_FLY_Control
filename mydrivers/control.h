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

extern PID_S ROLL_PID;    //{0.1,0,0,0,0,2}
extern PID_S PITCH_PID; //{0.1,0,0,0,0,2}
extern PID_S YAW_PID;
	
extern PID_S ANGLE_SPEED_Y_PID;  //{-30,-100,-1,0,0,20   旧飞机
extern PID_S ANGLE_SPEED_X_PID;
extern PID_S ANGLE_SPEED_Z_PID; //{-20,-20,0,0,0,5};

extern PID_S Height_PID;  //0.3
extern PID_S ACCEL_SPEED_Z_PID;
extern PID_S Velocity_Z_PID;

extern PID_S X_PID;
extern PID_S Y_PID;

extern Fly_State  State;

extern char Angle_Speed_Z_Flag;
extern char Roll_Pitch_Flag;
extern char Motor_Open_Flag;
extern char Height_Open_Flag;
extern char NRF_Flag;
extern float base_duty;
extern float fly_duty;

extern float balance_roll;   //平衡位置的角度
extern float balance_pitch;  //平衡位置的角度

extern float pitch_target;     //3.25
extern float roll_target;  //48.5
extern float yaw_target;
extern float height_target;
extern float height_offset;
void Fly_Control();

void Fly_Init();
void Fly_Stop();
uint8_t Is_Flying();
#endif
