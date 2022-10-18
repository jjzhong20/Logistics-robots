#ifndef _PID_H
#define _PID_H

#include "main.h"


#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))


typedef struct  //电机pid
{
  float kp;
  float ki;
  float kd;
  float i_max;
  float out_max;
  
  float ref;      // target value
  float fdb;      // feedback value
  float err[2];   // error and last error

  float p_out;
  float i_out;
  float d_out;
  float output;
}pid_struct_t;



void pid_init
(			  
	pid_struct_t *pid,
    float kp,
    float ki,
    float kd,
    float i_max,
    float out_max
);  //电机pid初始化
    	
float  pid_calc(pid_struct_t *pid, float ref, float fdb);  //电机PID输出
			  
float  pid_calc1(pid_struct_t *pid, float ref, float fdb);  //电机PID输出



#endif
