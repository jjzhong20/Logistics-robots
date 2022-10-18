#ifndef _PID_H
#define _PID_H

#include "main.h"


#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))


typedef struct  //���pid
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
);  //���pid��ʼ��
    	
float  pid_calc(pid_struct_t *pid, float ref, float fdb);  //���PID���
			  
float  pid_calc1(pid_struct_t *pid, float ref, float fdb);  //���PID���



#endif
