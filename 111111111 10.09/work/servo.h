#ifndef __SERVO_H
#define __SERVO_H
#include "stdio.h"
#include "can.h"
#include "stdlib.h"
#include "math.h"


extern double K_X, K_Y, K_ser,K_1;
extern uint8_t K_2;

uint8_t Kinematic_Analysis(double val_x, double val_y, double val_z, double pitch,uint8_t zhuazhi);//摄像头传入处理函数，
																									//距离与高度，底部电机偏向，顶部机械爪俯仰角,机械爪张合

void servoP_T(uint32_t *ServoID,int angle1,int time1,int angle2,int time2,int angle3,int time3,
int angle4,int time4,int angle5,int time5,int angle6,int time6);   //舵机CAN线处理并发送函数

#endif
