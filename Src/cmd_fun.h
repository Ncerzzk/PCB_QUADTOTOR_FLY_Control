#ifndef __CMD_FUN_H
#define __CMD_FUN_H

#include "control.h"
#include "command.h"
#include "base.h"
#include "angle.h"
#include "ms5611.h"


void add_duty(int arg_num,char **s ,float *args);
void set_flag(int arg_num,char **s,float * args);
void set_target(int arg_num,char **s,float * args);
void set_pid(int arg_num,char **s,float * args);
void add_pid(int arg_num,char **s,float *args);
void set_fly(int arg_num,char **s,float * args);
void set_stop(int arg_num,char **s,float * args);

#endif