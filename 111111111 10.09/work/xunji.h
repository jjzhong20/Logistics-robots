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
} Point;  //�����ṹ��,xΪ�ᣬyΪ����


//�ڰ״�����4
extern Heibai heibai_1;
extern Heibai heibai_2;
extern Heibai heibai_3;
extern Heibai heibai_4;

//������
extern Point finish;

//ԭ����
extern Point Point_RawMaterialArea;  //Ѳ����6,2��
extern Point Point_RawMaterial_Init;  //�ƶ�6,1����л�е��ץȡ

extern Point Sec1_RawMaterial_Init;
extern Point Sec2_RawMaterial_Init;

//�����
extern Point Point_FinishedAeraGreen;  //������������ϵ�ַ
extern Point Point_FinishedAeraGreen2;

//�ӹ���
extern Point Point_ProcessingAera;  //�ּӹ����������ϵ�ַ
extern Point Point_ProcessingAeraEndMV;  //�ӹ��������㣬�ƶ����˴�����
extern Point Point_ProcessingAeraEndMV2;
extern Point Point_ProcessingAera2;
extern Point Point_ProcessingAera3;

//��ά����
extern Point Point_QRCodeAera;  //2ά������x��2,3֮��

//������
extern Point Point_InitAera;  //��ʼ�����0��0

//������ʼ��
extern Point start_Init;  //ÿ�γ��������ʼ��

extern Point car;

//�ٶ�x��300��y��250  // ���500������ǰ��1��400��������ת��һȦ
extern car_count count_InitAera;  //����λ��
extern car_count count_QRCodeAera;  //2ά��λ��
extern car_count count_RawMaterialArea;  //ԭ����
extern car_count count_firstconor1;    //ԭ������ּӹ���ת��1
extern car_count count_firstconor2;    //ԭ������ּӹ���ת��2
extern car_count count_ProcessingAera;  //�ּӹ����������ϵ�ַ
extern car_count count_secondconor1;    //�ּӹ����������ת��1
extern car_count count_secondconor2;    //�ּӹ����������ת��2
extern car_count count_FinishedAer;  //������������ϵ�ַ
extern car_count count_third1conor;    //���������ԭ����1
extern car_count count_third2conor;    //���������ԭ����2
extern car_count count_third3conor;    //���������ԭ����3

extern car_count qianjin_one;
extern car_count now; //ʵʱ����
extern car_count last; //�ϴ�������

extern int countdiffer;

void migong_gezi(Point now, Point next);  //Ѳ����
void migong_line(Point now, Point next);  //Ѳ��
void count(Heibai *Heibaix ,int p);  //����+1
void count_1(void);
void count_2(void);
void count_middle1(void);
void count_middle2(void);
void count_middle3(void);
int rx_data_count(int data);  //Ѱ������ȷ�ϲ�����������0��1��
void xunji_set(void);  		  //Ѱ��ADģʽ����

void bmq_count(car_count direction); //���ñ����������뷽������ƶ�
void bmq_Init(void);				 //���ñ�������ʼ��
void bmq_Aver(void);  				 //��������ƽ��


#endif
