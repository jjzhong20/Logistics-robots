#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "main.h"

extern pid_struct_t motor_pid[4];  //pid结构体
extern pid_struct_t tof_pid[3];  //tof pid
extern pid_struct_t xun_pid[4];  //pid结构体
extern pid_struct_t jy931_pid;

extern int adj[5];

void data_send(void);  //电机数据发送
void speed_set(int xx,int yy, int yww);  //电机速度设定
void speed_stop(void);  //电机急停
void speed_set_without_pid(int xx,int yy,int yww);  //电机速度设定无pid
void speed_input(int16_t x,int16_t y,int16_t yw);  //麦轮速度计算
void motor_pid_init(void); //电机速度pid初始化 
void motor_pid_set(void);  //pid输出

extern int suduout[2];

void qianjin(void);
void houtui(void);
void zuoyi(void);
void youyi(void);

void qianjin_half(void);
void houtui_half(void);
void zuoyi_half(void);
void youyi_half(void);

void qianjin_total(void);
void houtui_total(void);
void zuoyi_total(void);
void youyi_total(void);
#endif
