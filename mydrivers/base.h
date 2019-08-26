#ifndef __BASE_H
#define __BASE_H

#include "outputdata.h"
#include "stm32f4xx_hal.h"
float avarge(int *data,int count);

typedef struct{
  char * wave_string;
  float * wave_ptr;
}wave_node;
extern wave_node Wave_Array[];

typedef enum{
  GPS_NO,
  GPS_OK
}GPS_State;
typedef struct{
    float UTC_TIME;
    GPS_State State;
    double Lat;
    double Long;
    float v;
}GPRMC;

typedef struct _kal_struct{
	float A;   //一般为1
	float B;   //一般为0
	float Q;//系统过程噪声的协方差
	float R;//测量噪声的协方差
	
	float kal_out; //上一次卡尔曼的输出
	
	float cov; //上一次卡尔曼的输出的协方差
	
}Kal_Struct;

typedef struct{
  float *buffer;
  int i;
  int max;
  uint8_t is_full;
}History_Buffer;


typedef struct{
  float * Window_Buffer;
  int max;  //数组长度
  int i;
}Window_Filter_Struct;

float Window_Filter(Window_Filter_Struct * wfs,float data);

float KalMan(Kal_Struct *kal,float x);

extern char Send_Wave_Flag;
void set_debug_wave(int arg_num,char ** string_prams,float * float_prams);
void debug_send_wave();
void load_prams(int arg_num,char ** s,float * args);
unsigned char Analize_GPS(char * raw_data,GPRMC * r);
void HB_Push(History_Buffer * hb,float data);
void HB_Clear(History_Buffer * hb);
int HB_Now(History_Buffer *hb);
float HB_Get(History_Buffer * hb,int index);

typedef struct
{
 //volatile 
   float Input_Butter[3];
 //volatile 
   float Output_Butter[3];
}Butter_BufferData;


typedef struct
{
  float a[3];
  float b[3];
}Butter_Parameter;

float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter);
void get_info(int arg_num,char **s,float *args);
float Limit_Dealt_Filter(float now,Window_Filter_Struct * wfs,float max_dealt);
#endif
