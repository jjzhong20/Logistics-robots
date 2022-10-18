#include "math.h"
#include "servo.h"   //��е�������ļ�
#include "pid.h"
#include "motor.h"
#include "xunji.h"
#include "deal.h"

pid_struct_t motor_pid[4];  //pid�ṹ��
pid_struct_t tof_pid[3];  //tof pid

int adj[5];


void speed_input(int16_t x,int16_t y,int16_t yw)  //�����ٶȼ���
{
	int b=12,a=12;
	
	speed_data[0] = y-x+yw * (a+b);
	speed_data[1] = y+x+yw * (a+b);
	speed_data[2] = 0-(y-x-yw * (b+a));
	speed_data[3] = 0-(y+x-yw * (a+b));
	
	
}


void data_send(void)  //������ݷ���
{
	//��������
	  for(i=0;i<4;i++)
	  {
		CANx_SendExtData(ID[i],message[i],8);
		HAL_Delay(10);
	  }

}

void speed_set(int xx,int yy,int yww)  //����ٶ��趨��ǰ����������ת
{
	  x=xx;
	  y=yy;
	  yw=yww;
	  speed_input(x,y,yw); //xyz	 �Կ��ط���Ϊy���� ǰ����Ϊx��������Ϊx�����򣬺�Ϊy������  
	  	
	  motor_pid_set();
		  
	//����ٶȶ���
	  for(i=0;i<4;i++)
	  {
		motor_msg[i]=speed_data1[i] + 32000;
		message[i][0]=motor_msg[i] %256;
		message[i][1]=motor_msg[i] /256;
	  }
	
}

void speed_stop(void)  //�����ͣ
{
	speed_input (0,0,0);
	for(i=0;i<4;i++)
	{
		motor_msg[i]=speed_data[i] + 32000;
		message[i][0]=motor_msg[i] %256;
		message[i][1]=motor_msg[i] /256;
	}
	motor_send();
}

void speed_set_without_pid(int xx,int yy,int yww)  //����ٶ��趨��pid
{
	  x=xx;
	  y=yy;
	  yw=yww;
	  speed_input(x,y,yw); //xyz	 �Կ��ط���Ϊy���� ǰ����Ϊx��������Ϊx�����򣬺�Ϊy������  
	  	
	  //motor_pid_set();
		  
	//����ٶȶ���
	  for(i=0;i<4;i++)
	  {
		motor_msg[i]=speed_data[i] + 32000;
		message[i][0]=motor_msg[i] %256;
		message[i][1]=motor_msg[i] /256;
	  }

}



void motor_pid_init(void)   //����ٶ�pid��ʼ��
{	
	pid_init(&motor_pid[0],1.4, 0.1, 0, 2000, 2000);
	pid_init(&motor_pid[1],1.4, 0.1, 0, 2000, 2000);
	pid_init(&motor_pid[2],1.4, 0.1, 0, 2000, 2000);
	pid_init(&motor_pid[3],1.4, 0.1, 0, 2000, 2000);
}

void motor_pid_set(void)  //pid���
{
		speed_data1[0] = (int16_t)pid_calc(&motor_pid[0], speed_data[0], v_back[0]);
		speed_data1[1] = (int16_t)pid_calc(&motor_pid[1], speed_data[1], v_back[1]);
		speed_data1[2] = (int16_t)pid_calc(&motor_pid[2], speed_data[2], v_back[2]);
		speed_data1[3] = (int16_t)pid_calc(&motor_pid[3], speed_data[3], v_back[3]);
	
}





