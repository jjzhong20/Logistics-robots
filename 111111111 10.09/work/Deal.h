#ifndef __DEAL_H__
#define __DEAL_H__

#include "main.h"
extern int servo_x,servo_y,servo_z;//��ʼ��е��λ��
extern int Three[3];
extern uint8_t QR_code_Order[6];

void Dael_Coordinate(uint8_t shuju[]);
void correct(int Three[3]);
#endif
