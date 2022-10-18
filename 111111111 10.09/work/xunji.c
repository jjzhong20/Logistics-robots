#include "math.h"
#include "servo.h"   //机械臂所在文件
#include "pid.h"
#include "motor.h"
#include "xunji.h"
#include "can.h"
#include "deal.h"
#include "main.h"


// 大概500个脉冲前进1格，400个脉冲电机转动一圈
int countdiffer=0;


int rx_data_count(int data)  //计算8路返回有几路1与0
{
	int f;
    int count = 0;
    
    for( f =0;  f< 8; f++)
    {
        if( (data>>f) & 1)
		{
			count ++;
		}
    }
	return count;
	
}


void xunji_set(void)  //寻迹AD模式设置
{
	int g;
//	uint8_t xunji_data1[8]={0,0,0,0,0,0,0,0};
	uint8_t xunji_data2[8]={3,0,0,0,0,0,0,0};
	int xunji_ID[4]={0x00310010,0x00320010,0x00330010,0x00340010};
	
	for(g=0;g<4;g++)
	{
		CANx_SendExtData(xunji_ID[g],xunji_data2,8);
	}	
}


void bmq_count(car_count direction) //利用编码器返回与方向进行移动
{	
	bmq_Aver();
	last .Aver_count = now .Aver_count ;
	while(countdiffer  < direction .Aver_count )
	{
		bmq_Aver();	
		switch(direction.dir)
		{
			case 1:
				speed_set_without_pid(0,250,0);
				motor_send ();
				break ;
			case 2:
				speed_set_without_pid(0,-250,0);
				motor_send ();
				break ;
			case 3:
				speed_set(-250,0,0);
				motor_send ();
				break ;
			case 4:
				speed_set(250,0,0);
				motor_send ();
				break ;
			case 5:
				speed_set_without_pid(0,0,10);
				motor_send ();
				break ;
			case 6:
				speed_set_without_pid (0,0,-10);
				motor_send ();
				break ;
			case 7:
				qianjin ();
				motor_send ();
				break ;
			case 8:
				houtui ();
				motor_send ();
				break ;
		}
//		  LCD_ShowNum(0,16,(int)bianmaqi_back[0],6,16);
//		  LCD_ShowNum(0,32,bianmaqi_back[1],6,16);
//		  LCD_ShowNum(0,48,bianmaqi_back[2],6,16);
//		  LCD_ShowNum(0,64,bianmaqi_back[3],6,16);
//		  LCD_ShowNum(0,80,countdiffer ,6,16);
//		  LCD_ShowNum(0,96,direction .Aver_count,6,16);
//		  LCD_ShowNum(50,48,now.Aver_count,6,16);
//		  LCD_ShowNum(50,64,last.Aver_count ,6,16);
//
	}
	

}
void bmq_Init(void) //利用编码器初始化
{
	
	now.Aver_count = 0;
	last.Aver_count =0;
}

void bmq_Aver(void)  //编码器求平均
{
	now.count1 = bianmaqi_back [0];
	now.count2 =bianmaqi_back [1];
	now.count3 =bianmaqi_back [2];
	now.count4 =bianmaqi_back [3];
	now.Aver_count =(now.count1 + now.count2 + now.count3 + now.count4 ) / 4;
	countdiffer = now.Aver_count - last.Aver_count ;
}

