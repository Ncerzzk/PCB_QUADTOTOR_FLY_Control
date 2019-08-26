#ifndef __ANGLE_H
#define __ANGLE_H

#include "math.h"
#include "mpu9250.h"
#include "USART.h"
#include "base.h"


#define BLE_CONTROL	1

typedef struct{
  float Angle[3];
  float Angle_Speed[3];
  float Accel[3];
  float Accel_E[3];
  float Mag[3];
  float Velocity[3];
  float Height;
}Attitude;

typedef struct{
  GPRMC gps;
  float height;
  float x;
  float y;
}Position;

extern Attitude Flight_Attitude;
extern Position Flight_Position;

extern int16_t Gyro_Offset[3];


extern float gravity_X,gravity_Y,gravity_Z; //重力加速度分量
extern uint8_t ACE_Offset_Flag;



typedef enum{
	left,
	right,
	ahead,
	back
}DIRECTION;

float KalMan(Kal_Struct *kal,float x);
void Get_Accel_Angle(void);
void Get_Angle_Speed(void);

void Get_Angle(void);
void Adjust_Acc(void);
void Get_Accel_Angle_Fast(void);
void Get_Angle_Speed_Fast(void);
extern float a_z_offset,a_x_offset,a_y_offset;
extern float w_x_offset,w_y_offset,w_z_offset;
extern float ACE_Z_Offset;
extern float Relative_Height;

float Get_Attitude_Data(int16_t raw_data,float range,int16_t offset);
void IMU_Update(float * ac,float * gy,float *attitude,float *ace);
float Get_Scale_Data(float raw_data,float offset,float scale);
void Scale_Mag(float *mag);
void AHR_Update(float * ac,float * gy,float * mag,float * attitude,float *ace,uint8_t use_mag);
void Free_Falling_Detect(float * accel,int call_time);

void Get_Velocity(float * ace,float *v,int call_time,float ace_sub);
void Get_Height(float *vz,float refer_height,int call_time,float *height);
void Get_Position(Attitude * attitude,int call_time,float refer_height,GPRMC *refer_gps,Position *position);
void Get_Ace_Offset();
void Wait_Height_Init();

void Get_Attitude(Attitude * a);

typedef struct{
  int16_t Gyro_Offset[3];
  float Acc_Offset[3];
  float Acc_Scale[3];
  float Mag_Offset[3];
  float Mag_Scale[3];
}__Calibration;

extern __Calibration Calibration;
#endif

