#ifndef __SERVO_H
#define __SERVO_H
#include "stdio.h"
#include "can.h"
#include "stdlib.h"
#include "math.h"


extern double K_X, K_Y, K_ser,K_1;
extern uint8_t K_2;

uint8_t Kinematic_Analysis(double val_x, double val_y, double val_z, double pitch,uint8_t zhuazhi);//����ͷ���봦������
																									//������߶ȣ��ײ����ƫ�򣬶�����еצ������,��еצ�ź�

void servoP_T(uint32_t *ServoID,int angle1,int time1,int angle2,int time2,int angle3,int time3,
int angle4,int time4,int angle5,int time5,int angle6,int time6);   //���CAN�ߴ������ͺ���

#endif
