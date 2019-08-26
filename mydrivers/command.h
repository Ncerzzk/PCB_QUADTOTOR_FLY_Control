#ifndef COMMAND_H
#define COMMAND_H
#include "stm32f4xx_hal.h"

#define CMD_MAX_NUM 50
static int get_float(char *s,int float_num,float *floats);
typedef void (*cmdFunc)(int arg_num,char ** string_arg,float * float_arg);


void add_cmd(char * s,cmdFunc f);
void analize(char * s);
void command_init(void);

uint8_t check_prams(int16_t arg_num,int string_prams,int float_prams);
char compare_cmd(const char * cmd,char * s);
uint8_t compare_string(const char *s1,char * s2);
#endif
