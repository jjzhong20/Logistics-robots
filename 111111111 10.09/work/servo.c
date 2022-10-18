#include "servo.h"

uint32_t ServoID[3]={0x00412010,0x00413010,0x00414010};

uint8_t Kinematic_Analysis(double val_x, double val_y, double val_z, double pitch,uint8_t zhuazhi)
{  
	
		double Bx, By, cosbeta,beta,alpha,theta1,costheta2,theta2,theta3;
		double pi = 3.1415926;
		double a1 = 130, a2 = 100, a3 = 70;
		double angle0,angle1,angle2,angle3,angle4;
//		double z;
		
//		angle0 = 1500;//底盘1号舵机
		angle0 = val_z;
		if (angle0 >= 1500)
		{
			angle0 = angle0 - 100;
		}
		else if(angle0 <= 1300)
		{
			angle0 = angle0 + 100;
		}

		//调节z值，后期放外面进行调节底盘方向
		
		Bx = val_x - a3 * cos(pitch);
		By = val_y - a3 * sin(pitch); 

		cosbeta = (Bx * Bx + By * By + a1 * a1 - a2 * a2) / (2* a1 * sqrt(Bx * Bx + By * By));

		beta = acos(cosbeta) * 180 / pi;

		alpha = atan2(By,Bx) * 180 / pi;

		theta1 = alpha + beta;

		costheta2 = -(Bx * Bx + By * By - a1 * a1 - a2 * a2)/(2 * a1 *a2);

		theta2 = - (180 - acos(costheta2) * 180 / pi);

		theta3 = pitch - theta1- theta2;//xy平面运动解算
		
		angle1 = 2500 - (7.5 * (theta1));//2号舵机
		angle2 = 1453 - (7.4 * theta2);//三号舵机
		angle3 =  1550 - (8.0 * theta3);//四号舵机
		if (zhuazhi == 1)
		{
			angle4 = 2100;	//1为机械爪开				
		}
		else if(zhuazhi == 0) 
		{
			angle4 = 2400;//机械爪闭
		}
	
				

		servoP_T(ServoID,(int )angle0,1000,(int )angle1,1000,(int )angle2,1000,(int )angle3,1000,(int )angle4,500,1500,1000);//发送舵机状态
//		HAL_Delay(1000);
	 
		return 0;
}




void servoP_T(uint32_t *ServoID,int angle1,int time1,int angle2,int time2,int angle3,int time3,
							int angle4,int time4,int angle5,int time5,int angle6,int time6)//舵机CAN线处理函数
{
	uint8_t servoX1[8];
	uint8_t servoX2[8];
	uint8_t servoX3[8];
	
	servoX1[0] = angle1%256;
	servoX1[1] = angle1/256;
	servoX1[2] = time1%256;
	servoX1[3] = time1/256;
	
	servoX1[4] = angle2%256;
	servoX1[5] = angle2/256;
	servoX1[6] = time2%256;
	servoX1[7] = time2/256;//1、2口数据
	
	CANx_SendExtData(ServoID[0],servoX1,8);
//	CANx_SendExtData(ServoID[0],servoX1);
	
	servoX2[0] = angle3%256;
	servoX2[1] = angle3/256;
	servoX2[2] = time3%256;
	servoX2[3] = time3/256;
	
	servoX2[4] = angle4%256;
	servoX2[5] = angle4/256;
	servoX2[6] = time4%256;
	servoX2[7] = time4/256;//3、4口数据
	
//	CANx_SendExtData(ServoID[1],servoX2);
	CANx_SendExtData(ServoID[1],servoX2,8);

	servoX3[0] = angle5%256;
	servoX3[1] = angle5/256;
	servoX3[2] = time5%256;
	servoX3[3] = time5/256;
	
	servoX3[4] = angle6%256;
	servoX3[5] = angle6/256;
	servoX3[6] = time6%256;
	servoX3[7] = time6/256;//5、6口数据
	
//	CANx_SendExtData(ServoID[2],servoX3);
	CANx_SendExtData(ServoID[2],servoX3,8);
	
}

