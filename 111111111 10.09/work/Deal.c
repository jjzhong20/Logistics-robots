#include "Deal.h"
#include "servo.h"
#include "stdio.h"
#include "lcd.h"

void Dael_Coordinate(uint8_t shuju[])//传入数据处理
{
//	char Three[3][3];//储存摄像头传入的物块三维坐标x,y,z
//	char Color[3];//物块颜色
//	char QR_code_Order[6];//二维码上的抓取顺序（两组顺序，例123+321）
//数据第10位代表：
/*T：三维坐标x,y,z
	C: 色环颜色
	Q：二维码上的抓取顺序
	L：TOF测距离
*/
	
	//12位数据$R020066081TA
	if (shuju[0] == '$' && shuju[10] == 'T' &&  shuju[11] == 'A' )//传入物块三维数据坐标到数组中
	{
		Three[0] = (shuju[2] - '0') * 100 +(shuju[3] - '0') * 10 + (shuju[4] - '0');//x轴
		Three[1] = (shuju[5] - '0') * 100 +(shuju[6] - '0') * 10 + (shuju[7] - '0');//y轴
		Three[2] = (shuju[8] - '0') * 100 +(shuju[9] - '0') * 10 + (shuju[10] - '0');//z轴
	}
//	if (shuju[0] == '$' && shuju[10] == 'C' &&  shuju[11] == 'A' )//传入色环颜色数据坐标到数组中
//	{
//		Color[0] = shuju[2];
//		Color[1] = shuju[3];
//		Color[2] = shuju[4];
//	}
	
	if (shuju[0] == '$' && shuju[10] == 'Q' &&  shuju[11] == 'A' )//传入二维码抓取顺序数据坐标到数组中
	{
		QR_code_Order[0] = shuju[1] - '0';
		QR_code_Order[1] = shuju[2] - '0';
		QR_code_Order[2] = shuju[3] - '0';
		QR_code_Order[3] = shuju[4] - '0';
		QR_code_Order[4] = shuju[5] - '0';
		QR_code_Order[5] = shuju[6] - '0';
	}
}

