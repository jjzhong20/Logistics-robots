#include "Deal.h"
#include "servo.h"
#include "stdio.h"
#include "lcd.h"

void Dael_Coordinate(uint8_t shuju[])//�������ݴ���
{
//	char Three[3][3];//��������ͷ����������ά����x,y,z
//	char Color[3];//�����ɫ
//	char QR_code_Order[6];//��ά���ϵ�ץȡ˳������˳����123+321��
//���ݵ�10λ����
/*T����ά����x,y,z
	C: ɫ����ɫ
	Q����ά���ϵ�ץȡ˳��
	L��TOF�����
*/
	
	//12λ����$R020066081TA
	if (shuju[0] == '$' && shuju[10] == 'T' &&  shuju[11] == 'A' )//���������ά�������굽������
	{
		Three[0] = (shuju[2] - '0') * 100 +(shuju[3] - '0') * 10 + (shuju[4] - '0');//x��
		Three[1] = (shuju[5] - '0') * 100 +(shuju[6] - '0') * 10 + (shuju[7] - '0');//y��
		Three[2] = (shuju[8] - '0') * 100 +(shuju[9] - '0') * 10 + (shuju[10] - '0');//z��
	}
//	if (shuju[0] == '$' && shuju[10] == 'C' &&  shuju[11] == 'A' )//����ɫ����ɫ�������굽������
//	{
//		Color[0] = shuju[2];
//		Color[1] = shuju[3];
//		Color[2] = shuju[4];
//	}
	
	if (shuju[0] == '$' && shuju[10] == 'Q' &&  shuju[11] == 'A' )//�����ά��ץȡ˳���������굽������
	{
		QR_code_Order[0] = shuju[1] - '0';
		QR_code_Order[1] = shuju[2] - '0';
		QR_code_Order[2] = shuju[3] - '0';
		QR_code_Order[3] = shuju[4] - '0';
		QR_code_Order[4] = shuju[5] - '0';
		QR_code_Order[5] = shuju[6] - '0';
	}
}

