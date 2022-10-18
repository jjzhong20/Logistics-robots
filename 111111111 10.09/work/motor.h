#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "main.h"

extern pid_struct_t motor_pid[4];  //pid�ṹ��
extern pid_struct_t tof_pid[3];  //tof pid
extern pid_struct_t xun_pid[4];  //pid�ṹ��
extern pid_struct_t jy931_pid;

extern int adj[5];

void data_send(void);  //������ݷ���
void speed_set(int xx,int yy, int yww);  //����ٶ��趨
void speed_stop(void);  //�����ͣ
void speed_set_without_pid(int xx,int yy,int yww);  //����ٶ��趨��pid
void speed_input(int16_t x,int16_t y,int16_t yw);  //�����ٶȼ���
void motor_pid_init(void); //����ٶ�pid��ʼ�� 
void motor_pid_set(void);  //pid���

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
