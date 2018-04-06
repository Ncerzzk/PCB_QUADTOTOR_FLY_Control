
#include "cmd_fun.h"


inline void set_flag_command(char * flag,int arg_num,float * args){
	if(arg_num>1){
		uprintf("error arg num!\r\n");
		return ;
	}
	if(arg_num){
		*flag=(int)args[0];
	}else{
		*flag=!(*flag);
	}
	uprintf("ok,set flag=%d\r\n",*flag);
}

void add_duty(int arg_num,char **s ,float *args){
  if(arg_num!=0x0001){
    uprintf("error arg_num!\r\n");
    return ;
  }
  base_duty+=args[0];
  Limit(base_duty,100);
  uprintf("OK,base_duty=%f\r\n",base_duty);
}
void set_flag(int arg_num,char **s,float * args){
  if(arg_num!=0x0101&&arg_num!=0x0100){
    uprintf("error arg_num!\r\n");
    return ;
  }
  if(compare_string("az",s[0])){
    set_flag_command(&Angle_Speed_Z_Flag,arg_num&0x0f,args);
  }else if(compare_string("roll_pitch",s[0])){
    set_flag_command(&Roll_Pitch_Flag,arg_num&0x0f,args);
  }else if(compare_string("motor",s[0])){
    set_flag_command(&Motor_Open_Flag,arg_num&0x0f,args);
  }else if(compare_string("height",s[0])){
    if(Height-MS5611_Height<50&&Height-MS5611_Height>-50)
      set_flag_command(&Height_Open_Flag,arg_num&0x0f,args);
    else{
      uprintf("wait for the height init!\r\n");
    }
  }else if(compare_string("nrf",s[0])){
    set_flag_command(&NRF_Flag,arg_num&0x0f,args);
  }else if(compare_string("aceo",s[0])){
    set_flag_command(&ACE_Offset_Flag,arg_num&0x0f,args);
  }
}

void set_target(int arg_num,char **s,float * args){
  float * ptr=0;
  if(arg_num!=0x0101&&arg_num!=0x0100){
    uprintf("error arg_num!\r\n");
    return ;
  }
  if(compare_string("roll",s[0])){
    ptr=&balance_roll;
  }else if(compare_string("pitch",s[0])){
    ptr=&balance_pitch;
  }else if(compare_string("yaw",s[0])){
    ptr=&yaw_target;
  }else if(compare_string("height",s[0])){
    ptr=&height_target;
  }else{
    uprintf("error target!\r\n");
    return ;
  }
  
  if(arg_num==0x0101){
    *ptr=args[0];
  }
  uprintf("OK,%s_target = %f\r\n",s[0],*ptr); 
}

float * get_pid_ptr(char **s);
void set_pid(int arg_num,char **s,float * args){
  float *pid_ptr;
  if(arg_num!=0x0201){
    uprintf("error arg_num!\r\n");
    return ;
  }
  pid_ptr=get_pid_ptr(s);
  
  if(pid_ptr==0){
    uprintf("error control type or error pid type!\r\n");
    return ;
  }
  *pid_ptr=args[0];
  
  uprintf("OK ,set %s %s = %f\r\n",s[0],s[1],*pid_ptr);
}


void add_pid(int arg_num,char **s,float *args){
 float *pid_ptr;
  if(arg_num!=0x0201){
    uprintf("error arg_num!\r\n");
    return ;
  }
  pid_ptr=get_pid_ptr(s);
  
  if(pid_ptr==0){
    uprintf("error control type or error pid type!\r\n");
    return ;
  }
  *pid_ptr=*pid_ptr+args[0];
  
  uprintf("OK ,set %s %s = %f\r\n",s[0],s[1],*pid_ptr);  
}

static float * get_pid_ptr(char **s){
  char * control_type=s[0];
  char * pid_type=s[1];
  PID_S *ptr=0;
  float *float_ptr=0;
  if(compare_string("wx",control_type)){
    ptr=&ANGLE_SPEED_X_PID;
  }else if(compare_string("wy",control_type)){
    ptr=&ANGLE_SPEED_Y_PID;
  }else if(compare_string("wz",control_type)){
    ptr=&ANGLE_SPEED_Z_PID;
  }else if(compare_string("roll",control_type)){
    ptr=&ROLL_PID;
  }else if(compare_string("pitch",control_type)){
    ptr=&PITCH_PID;
  }else if(compare_string("yaw",control_type)){
    ptr=&YAW_PID;
  }else if(compare_string("az",control_type)){
    ptr=&ACCEL_SPEED_Z_PID;
  }else if(compare_string("vz",control_type)){
    ptr=&Velocity_Z_PID;
  }else if(compare_string("h",control_type)){
    ptr=&Height_PID;
  }else if(compare_string("x",control_type)){
    ptr=&X_PID;
  }else if(compare_string("y",control_type)){
    ptr=&Y_PID;
  }else{
    return 0;
  }
  
  if(compare_string("p",pid_type)){
    float_ptr=&(ptr->KP);
  }else if(compare_string("i",pid_type)){
    float_ptr=&(ptr->KI);
  }else if(compare_string("d",pid_type)){
    float_ptr=&(ptr->KD);
  }else if(compare_string("i_max",pid_type)){
    float_ptr=&(ptr->i_max);
  }else{
    return 0;
  }
  return float_ptr;
  
}

void set_fly(int arg_num,char **s,float * args){
	if(arg_num>0){
		uprintf("error arg_num!\r\n");
	}
	if(State==STOP)
		Fly_Init();
	else
		Fly_Stop();
	uprintf("OK,change Fly!\r\n");
}

void set_stop(int arg_num,char **s,float * args){
	if(arg_num>0){
		uprintf("error arg_num!\r\n");
        return ;
	}
	Fly_Stop();
	uprintf("OK,Stop!\r\n");
}

