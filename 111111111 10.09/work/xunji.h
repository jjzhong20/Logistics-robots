#ifndef __XUNJI_H__
#define __XUNJI_H__

#include "main.h"
#include "pid.h"

typedef struct 
{
	int data;
	int count;
	int xun_startflag;
	int xun_endflag;
	int xun_countflag;
	int xun_mid1;
	int xun_mid2;
}Heibai;


typedef struct 
{
	int count1;
	int count2;
	int count3;
	int count4;
	int Aver_count;
	int dir;
}car_count;

typedef struct Point{
	int x;
	int y;
	int yaw;
} Point;  //坐标点结构体,x为横，y为纵轴


//黑白传感器4
extern Heibai heibai_1;
extern Heibai heibai_2;
extern Heibai heibai_3;
extern Heibai heibai_4;

//结束区
extern Point finish;

//原料区
extern Point Point_RawMaterialArea;  //巡线至6,2点
extern Point Point_RawMaterial_Init;  //移动6,1点进行机械臂抓取

extern Point Sec1_RawMaterial_Init;
extern Point Sec2_RawMaterial_Init;

//完成区
extern Point Point_FinishedAeraGreen;  //完成区中心物料地址
extern Point Point_FinishedAeraGreen2;

//加工区
extern Point Point_ProcessingAera;  //粗加工区中心物料地址
extern Point Point_ProcessingAeraEndMV;  //加工区结束点，移动到此处自旋
extern Point Point_ProcessingAeraEndMV2;
extern Point Point_ProcessingAera2;
extern Point Point_ProcessingAera3;

//二维码区
extern Point Point_QRCodeAera;  //2维码区在x轴2,3之间

//出发区
extern Point Point_InitAera;  //初始坐标点0，0

//出发初始化
extern Point start_Init;  //每次出发坐标初始化

extern Point car;

//速度x向300，y向250  // 大概500个脉冲前进1格，400个脉冲电机转动一圈
extern car_count count_InitAera;  //出发位置
extern car_count count_QRCodeAera;  //2维码位置
extern car_count count_RawMaterialArea;  //原料区
extern car_count count_firstconor1;    //原料区向粗加工区转弯1
extern car_count count_firstconor2;    //原料区向粗加工区转弯2
extern car_count count_ProcessingAera;  //粗加工区中心物料地址
extern car_count count_secondconor1;    //粗加工区向完成区转弯1
extern car_count count_secondconor2;    //粗加工区向完成区转弯2
extern car_count count_FinishedAer;  //完成区中心物料地址
extern car_count count_third1conor;    //完成区返回原料区1
extern car_count count_third2conor;    //完成区返回原料区2
extern car_count count_third3conor;    //完成区返回原料区3

extern car_count qianjin_one;
extern car_count now; //实时脉冲
extern car_count last; //上次脉冲数

extern int countdiffer;

void migong_gezi(Point now, Point next);  //巡格子
void migong_line(Point now, Point next);  //巡线
void count(Heibai *Heibaix ,int p);  //过线+1
void count_1(void);
void count_2(void);
void count_middle1(void);
void count_middle2(void);
void count_middle3(void);
int rx_data_count(int data);  //寻迹返回确认布尔量个数（0与1）
void xunji_set(void);  		  //寻迹AD模式设置

void bmq_count(car_count direction); //利用编码器返回与方向进行移动
void bmq_Init(void);				 //利用编码器初始化
void bmq_Aver(void);  				 //编码器求平均


#endif
