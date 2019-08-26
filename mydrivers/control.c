#include "control.h"
#include "angle.h"
#include "tim.h"
#include "command.h"

#define HEIGHT_T  120
#define UP_DUTY			60

enum attitude{
  PITCH=0x00,
  ROLL=0x01,
  YAW=0x02,
  X=0x00,
  Y=0x01,
  Z=0x02
};


//期望的俯仰、横滚、偏航角
float pitch_target=0;     //3.25
float roll_target=0;  //48.5
float yaw_target=0;
float balance_roll=2.5;   //平衡位置的角度
float balance_pitch=2;  //平衡位置的角度    该位置的角度是由于加速度计的零偏误差造成的。
float height_target=200;   //单位cm
float height_offset=0;  
float Angle_Speed_X_Out=0;
float Angle_Speed_Y_Out=0;
float Angle_Speed_Z_Out=0;
float fly_duty=40;

float Velocity_Z_Out,Velocity_Z_Out_Old;
//float Height_Out,Height_Out_Old;
float Correct;

float Accel_Speed_Z_Out=0; //加速度环 高度环的内环


extern float MS5611_Height;

float base_duty=0; //50
float CHn_Out[4];

//PID 结构体参数
//带Single是单级PID

PID_S ROLL_PID={0.25,0.3,0,0,0,2};    //{0.1,0,0,0,0,2}
PID_S PITCH_PID={0.25,0.3,0,0,0,2}; //{0.1,0,0,0,0,2}
PID_S YAW_PID={-0.1,0,0,0,0,2};
	
PID_S ANGLE_SPEED_Y_PID={-30,-100,-1,0,0,20};  //{-30,-100,-1,0,0,20   旧飞机
PID_S ANGLE_SPEED_X_PID={-30,-100,-1,0,0,20};
PID_S ANGLE_SPEED_Z_PID={-10,-20,0,0,0,20}; //{-20,-20,0,0,0,5};

PID_S Height_PID={0,0,0,0,0,9000};  //0.3
PID_S ACCEL_SPEED_Z_PID={7,0,0,0,0,2};
PID_S Velocity_Z_PID={0.03,1,0.005,0,0,2000};

PID_S X_PID={0.1,0,0,0,0,9000};
PID_S Y_PID={0.1,0,0,0,0,9000};


Fly_State  State=STOP;

int position_x,position_y;

void set_flag_command(char * flag,int arg_num,float * args);
void Fly_Stop(void);

float * get_pid_ptr(char **s);
float Get_Correct(float now,float last,int correct_time,int step_time); 



void Fly_Init(void){      //解锁飞行，初始化高度、yaw
	State=FLY_WAIT;
	base_duty=fly_duty;           //FLY_WAIT状态，占空比可以手动更改
	ROLL_PID.i=0;
	PITCH_PID.i=0;
	ANGLE_SPEED_Y_PID.i=0;
	ANGLE_SPEED_X_PID.i=0;
	ANGLE_SPEED_Z_PID.i=0;
    
    Height_PID.i=0;
    Velocity_Z_PID.i=0;
    yaw_target=Flight_Attitude.Angle[YAW];
    
    
    //height_offset=Height;         //电机旋转后的气压计会突变，不可在此时初始化高度
    Correct=0;
    Velocity_Z_Out=0;
    Velocity_Z_Out_Old=0;
}


//set_pid wx p 100


void Fly_Stop(void){
	State=STOP;
}


float PID_Control(PID_S *PID,float target,float now){
	float err;
	float err_dt;
	float result;

	err=target-now;
	
	err_dt=err-PID->last_err;
	
	
	err_dt*=0.384f;
	err_dt+=PID->last_d*0.615f;   //低通滤波
	
	
	PID->last_err=err;
	
	PID->i+=err*I_TIME;
	
	Limit(PID->i,PID->i_max);
	PID->last_d=err_dt;
	
	result = err * PID->KP  +   err_dt * PID->KD   +   PID->i * PID->KI;
	return result;
}


//set_target roll 10

inline void Motor_Stop(){
  for(int i=1;i<=4;++i){
	Set_Speed(i,0);
  }
}

char Angle_Speed_Z_Flag=1;
char Roll_Pitch_Flag=1;
char Motor_Open_Flag=1;
char Height_Open_Flag=1;
char NRF_Flag=0;

float get_yaw_err(float now,float target){
  float sub,a_sub,result;
  //正方向：逆时针
  sub=target-now;
  a_sub=fabs(sub);
  if(a_sub>=180){
    a_sub=360-a_sub;
    result=-a_sub;
  }else{
    result=sub;
  }
  
  return result;
  
}

/*
thrust 油门
direction 方向舵
ele 升降舵
aile 副翼
*/
void RC_Set_Target(float thrust,float direction,float ele,float aile){
  if(thrust<1 && direction<1 && ele<1 && aile >95){
    Fly_Init();
    uprintf("set_fly!\r\n");
  }else if(thrust <1 && ele <1 && direction>47 &&direction<51){
    Fly_Stop();
    uprintf("set_stop!\r\n");
  }
  
  /*
  定高测试语句
*/
  uprintf("%f %f %f %f\r\n",thrust,direction,ele,aile);
  //HAL_Delay(10);
  if(direction>97){
    Fly_Stop();
    uprintf("set_stop direction!\r\n");
  }
  
  base_duty=thrust-50;       //定高测试，注释掉这句
  
  if(State!=STOP){
    if(base_duty<20){
      base_duty=20;
    } 
  }
  roll_target=(10*ele/100-5);
  pitch_target=-(10*aile/100-5);
}


void Fly_Control(){		
  float Pitch_Out=0;
  float Roll_Out=0;
  float Yaw_Out=0;
  
  
  if(State==STOP){
    Motor_Stop();
    return ;
  }
  
  if(Is_Flying()){
    Yaw_Out=PID_Control(&YAW_PID,0,get_yaw_err(Flight_Attitude.Angle[Z],yaw_target));
    Limit(Yaw_Out,10);
    if(Angle_Speed_Z_Flag)
      Angle_Speed_Z_Out=PID_Control(&ANGLE_SPEED_Z_PID,Yaw_Out,Flight_Attitude.Angle_Speed[Z]);
    else
      Angle_Speed_Z_Out=0;
  }
  
  //Limit(Angle_Speed_Z_Out,15);
  
  if(Roll_Pitch_Flag){ //外环开启
    Roll_Out=PID_Control(&ROLL_PID,roll_target+balance_roll,Flight_Attitude.Angle[ROLL]);     //如果是调角速度内环，注释掉这句
    Limit(Roll_Out,10);
    Pitch_Out=PID_Control(&PITCH_PID,pitch_target+balance_pitch,Flight_Attitude.Angle[PITCH]);  //如果是调角速度内环，注释掉这句
    Limit(Pitch_Out,10);
  }else{//外环关闭
    Roll_Out=0;
    Pitch_Out=0;
  }
  
  /*
  if(Height_Open_Flag){
    Velocity_Out=PID_Control(&Height_PID,height_target+height_offset,Height)+FLY_DUTY;
    //Velocity_Out=PID_Control(&Velocity_PID,0,Velocity[Z])+20;
  }else{
    Velocity_Out=0;
  }*/
  //Correct+=Get_Correct(Velocity_Z_Out,Velocity_Z_Out_Old,20,2);
  Correct=Velocity_Z_Out;
  Limit(Correct,50); 
  
  Angle_Speed_X_Out=PID_Control(&ANGLE_SPEED_X_PID,Roll_Out,Flight_Attitude.Angle_Speed[X]);
  Angle_Speed_Y_Out=PID_Control(&ANGLE_SPEED_Y_PID,Pitch_Out,Flight_Attitude.Angle_Speed[Y]);
  
  //串级PID
  CHn_Out[0]=Angle_Speed_X_Out-Angle_Speed_Y_Out+Angle_Speed_Z_Out+base_duty+Correct;  //以角度向前倾，向左倾为标准
  CHn_Out[1]=-Angle_Speed_X_Out-Angle_Speed_Y_Out-Angle_Speed_Z_Out+base_duty+Correct;
  CHn_Out[2]=-Angle_Speed_X_Out+Angle_Speed_Y_Out+Angle_Speed_Z_Out+base_duty+Correct;
  CHn_Out[3]=Angle_Speed_X_Out+Angle_Speed_Y_Out-Angle_Speed_Z_Out+base_duty+Correct;
  
  
  if(Motor_Open_Flag){
    for(int i=1;i<5;++i){ 
      Set_Speed(i,CHn_Out[i-1]);
    }
  }
}


void Height_Control(){
  static float height_out=0;
  static int height_cnt=0;
  
  
  if(Height_Open_Flag){
    height_cnt++;
    if(height_cnt==5){
      height_out=PID_Control(&Height_PID,height_target+height_offset,Flight_Position.height);
      height_cnt=0;
    }
    
    Velocity_Z_Out_Old=Velocity_Z_Out;
    
    //Height_Out=PID_Control(&Height_PID,height_target+height_offset,Height);
    Velocity_Z_Out=PID_Control(&Velocity_Z_PID,height_out,Flight_Attitude.Velocity[2]);
  }else{
    Velocity_Z_Out=0;
    Velocity_Z_Out_Old=0;
  }
}

/*
  correct_time:控制时间（长） 单位:ms
  step_time:步进时间（短）    单位:ms
*/
float Get_Correct(float now,float last,int correct_time,int step_time){
  float sub=now-last;
  float correct;
  int times;
  
  times=correct_time/step_time;
  correct=sub/times;
  return correct;
}
/*
  判断是否在飞行
*/
uint8_t Is_Flying(){
  if(State!=STOP){
    return 1;
  }else{
    return 0;
  }
}