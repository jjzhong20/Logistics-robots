/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "math.h"
#include "servo.h"   //机械臂所在文件
#include "pid.h"
#include "motor.h"
#include "xunji.h"
#include "Deal.h"
#include "text.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t speed_data[4];   //电机速度存放数组
int16_t speed_data1[4];  //电机速度存放数组
int16_t motor_msg[4];    //电机速度存放数组
int x=0,y=0,yw=0;        //电机xy速度值与旋转分量，用于麦轮解算时输入速度分量

int i=0; 				 //电机速度定义与CAN发送数据的循环变量

int16_t v_back[4]={0};       //CAN线电机返回值，接收电机驱动返回的相对速度
int16_t bianmaqi_back[4];    //编码器返回数组
uint8_t message[7][8]={0};   //can发送数据的存放数组
int ID[4]={0x00210010,0x00220010,0x00230010,0x00240010};  //ID号数组前四位为电机ID号

int servo_x = 120,servo_y = 150,servo_z = 1500,zhua = 0;  //初始机械臂位置
int Three[3] = {0,0,0};                    //储存摄像头传入的物块三维坐标x,y,z
char Color[3]; 											  //物块颜色
uint8_t QR_code_Order[6] = {0,0,0,0,0,0}; 					  //二维码上的抓取顺序（两组顺序，例123+321）
uint8_t pData[30] = {0};



//迷宫变量定义
int xunji_count[4];		  //寻迹返回，存放寻迹灰度返回个数
int xunji[4];			  //寻迹返回值存放，4路 ，8寻迹值在0-255之间，对应1字节
int xunji_flag[4]={0};	  //寻迹标志位，用于迷宫解算时标志哪边先碰线，以此更改自身坐标

//串口2维码变量定义
int len[5]={0};
uint8_t USART1_RX_BUF[200];  //串口数据接收
uint8_t USART2_RX_BUF[200];  //串口数据接收
uint8_t USART3_RX_BUF[200];  //串口数据接收
uint8_t USART4_RX_BUF[200];  //串口数据接收
uint8_t USART5_RX_BUF[200];  //串口数据接收

int tof[3]={0};              //tof数据存放数组 单位cm
int tof_adj[3]={0};			 //tof数据滤波后存放，但因回传速率不够等暂停使用
int tof_count[3] = {0};
int tof_sum[3] ={0};

int yaw_931=0; 				 //陀螺仪返回数据

int suduout[2];				 //寻迹pid输出
pid_struct_t xun_pid[4];     //寻迹pid结构体
int xun_flag=0; 			 //寻迹标志位，由前后左右移控制，对应寻迹时的判断及寻迹pid引起的速度变化

////四个8路寻迹   黑白结构体，用来判断小车移动过线
Heibai heibai_1={0,0,0,0,0};
Heibai heibai_2={0,0,0,0,0};
Heibai heibai_3={0,0,0,0,0};
Heibai heibai_4={0,0,0,0,0};

//暂时没用											// 大概500个脉冲前进1格，400个脉冲电机转动一圈
car_count count_InitAera={0,0,0,0,700,1};    	 	//出发位置
car_count count_QRCodeAera={0,0,0,0,1000,3};  		//2维码位置
car_count count_RawMaterialArea={0,0,0,0,1900,3};   //原料区
car_count count_firstconor1={0,0,0,0,250,3};  	    //原料区向粗加工区转弯1
car_count count_firstconor2={0,0,0,0,500,5};   	    //原料区向粗加工区转弯2
car_count count_ProcessingAera={0,0,0,0,1200,3};    //粗加工区中心物料地址
car_count count_secondconor1={0,0,0,0,1250,3};      //粗加工区向完成区转弯1
car_count count_secondconor2={0,0,0,0,610,5};    	//粗加工区向完成区转弯2
car_count count_FinishedAera={0,0,0,0,1250,3};  	//完成区中心物料地址
car_count count_third1conor={0,0,0,0,2500,1};    	//完成区返回原料区1
car_count count_third2conor={0,0,0,0,1250,5};    	//完成区返回原料区2
car_count count_third3conor={0,0,0,0,1100,3};    	//完成区返回原料区3
car_count now={0,0,0,0,0,0}; 						//实时脉冲
car_count last={0,0,0,0,0,0}; 						//上次脉冲数
car_count zuoyi_one={0,0,0,0,500,3};
car_count zero={0,0,0,0,0,1};
car_count qianjing_half={0,0,0,0,250,7};
car_count tui_half={0,0,0,0,250,2};
car_count xuanzhuan90={0,0,0,0,600,5};
car_count tui_half_half={0,0,0,0,150,2};

//定时器，用于小车旋转
int time_compare[3]={32,30,30};
int time=0;
int time_flag[3] = {1,1,1};

//定点
Point Point_0={0,0,0}; 
Point Point_1={2,0,0};  
Point Point_2={6,0,0};
Point Point_3={8,0,0};
Point Point_4={10,0,0};  
Point Point_5={12,0,0};  
Point Point_6={12,4,0};
Point Point_7={14,4,0};

Point Point_8={0,0,0};
Point Point_9={3,0,0};  
Point Point_10={0,0,0};  

Point car={0,0,0};


//舵机运行动作组
//二维码区
uint8_t ER_go[8] = {4,0,0,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t ER_stop[8] = {8,0,0,0,0,0,0,0};  // 1位为4开始运行，为8，停止运行
uint8_t ER_back[8] = {4,0,1,0,0,0,0,0}; 
uint8_t ER_back_stop[8] = {8,0,1,0,0,0,0,0}; 
//原料区
//上层
uint8_t first_left_up[8] = {4,0,2,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t first_left_up_done[8] = {8,0,2,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t first_middle_up[8] = {4,0,3,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t first_middle_up_done[8] = {8,0,3,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t first_right_up[8] = {4,0,4,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t first_right_up_done[8] = {8,0,4,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
//下层
uint8_t first_left_down[8] = {4,0,5,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t first_left_down_done[8] = {8,0,5,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t first_middle_down[8] = {4,0,6,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t first_middle_down_done[8] = {8,0,6,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t first_right_down[8] = {4,0,7,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t first_right_down_done[8] = {8,0,7,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号

//粗加工区
uint8_t second_left[8] = {4,0,8,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t second_left_done[8] = {8,0,8,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t second_middle[8] = {4,0,9,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t second_middle_done[8] = {8,0,9,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t second_right[8] = {4,0,10,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t second_right_done[8] = {8,0,10,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号

//成品区

//下层
uint8_t third_left_up[8] = {4,0,11,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t third_left_up_done[8] = {8,0,11,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t third_middle_up[8] = {4,0,12,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t third_middle_up_done[8] = {8,0,12,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t third_right_up[8] = {4,0,13,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t third_right_up_done[8] = {8,0,13,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
//码垛
uint8_t third_left_down[8] = {4,0,14,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t third_left_down_done[8] = {8,0,14,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t third_middle_down[8] = {4,0,15,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t third_middle_down_done[8] = {8,0,15,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t third_right_down[8] = {4,0,16,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号
uint8_t third_right_down_done[8] = {8,0,16,0,0,0,0,0}; //0位为动作组运行开始与结束  2位为动作组编号


uint16_t  k210_state = 0;


//15, 15, 500,0,'F'
double K_X = 15, K_Y = 15, K_ser =600,K_1 = 0;
uint8_t K_2 = 1;
/*
与k210的数据接收

1.
	摄像头上电，车子MCU state状态为0，开启12位串口接收中断
2.
	车子移动到指定位置，通过串口5发送123给k210，
3.
	k210返回数据给MCU，在串口中断回调里面进行处理，若成功接收到数据，则state状态为1，并开启下一次串口接收中断
若没有接收到数据，则继续发送123
4.
	根据二维码的数据，进行抓取时的顺序处理
定义
	红 111
	绿 222
	蓝 333
	如果接收到二维码是   123+123
	
	则先通过串口发送111给k210，k210返回红色物块的XY坐标和距离
	
	MCU每发送111请求一次，k210返回一次，串口对其数据进行处理，判断坐标是否在中心位置
	若在则改变状态state为2，若不在则state仍为1，同时请求下一次发送，建议发送的间隔在100ms
	
	state为2后，串口发送222，后面同上


*/

Point Point0={0,0,0}; 
Point Point1={2,0,0};  
Point Point2={5,0,0}; 
Point Point3={7,0,0};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_DMA_Init();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_CAN_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  
//  HAL_UART_Receive_IT (&huart5,USART5_RX_BUF ,12);	//串口接收中断，接收陀螺仪数据
  LCD_Init ();		  //LCD初始化
  motor_pid_init();   //电机pid初始化  
  xun_pid_init(); 	  //寻迹pid初始化
  CAN_filter_Init();  //CAN过滤器初始化
  
  HAL_TIM_Base_Start_IT(&htim2 ); //定时器2与3中断开启
  HAL_TIM_Base_Start_IT(&htim3 );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	LCD_ShowNum (0,0,0,3,16,BLACK );
	
	while(tof[2] < 8)  //移动
	{
		youyi_total();
	 }
	 
	
	while( (( (tof[0] + tof[1]) / 2) < 30)  )    
	{
		if(( (tof[0] + tof[1]) / 2) < 30)
		{
			qianjin_total();
		}
		LCD_ShowNum (20,200,tof[0],3,16,BLACK );
		LCD_ShowNum (90,200,tof[1],3,16,BLACK );
		LCD_ShowNum (160,200,tof[2],3,16,BLACK );
	}
	
	while ((xunji[1] < 15 || xunji [1] > 30) && (xunji[3] < 15 || xunji [3] > 30)) 
	{
		if((xunji [1] < 15) && (xunji[3] < 15 ))
		{
			qianjin_half();
		}
		else if((xunji [1] > 30) && (xunji[3] > 30 ))
		{
			houtui_half ();	
		}
	}



	migong_gezi(Point0 ,Point1 ); //移动二维码区
	speed_stop();
	motor_send ();
	HAL_Delay (1000);
	
	
	migong_gezi(Point1 ,Point2 );  //移动物料区
	while ((xunji[0] < 15 || xunji [0] > 30) && (xunji[2] < 15 || xunji [2] > 30)) //物料区位置微调
	{
		if((xunji [0] < 15) && (xunji[2] < 15 ))
		{
			youyi_half();
		}		
		if((xunji [0] < 15) && (xunji[2] > 30 ))
		{
			zuoyi_half();
		}
		else 
		{
			youyi_half();
		}
	}
	speed_stop();
	motor_send ();
	HAL_Delay (200);
		
	while ((xunji[0] < 15 || xunji [0] > 30) && (xunji[2] < 15 || xunji [2] > 30)) //物料区位置微调
	{		
		if((xunji [0] < 15) && (xunji[2] > 30 ))
		{
			zuoyi_half();
		}
		else 
		{
			youyi_half();
		}
	}	
	
	while ((xunji[1] < 40 || xunji [1] > 80) && (xunji[3] < 40 || xunji [3] > 80)) //物料区位置微调
	{		
		if((xunji [1] < 40) && (xunji[3] > 80 ))
		{
			qianjin_half();
		}
		else 
		{
			houtui_half();
		}
	}
	HAL_Delay(300);
	
	time = 0;  //旋转 
	time_flag[0] = 1;
	HAL_TIM_Base_Start_IT(&htim4 );
	while(time_flag[0])
	{	
		speed_set_without_pid(0,0,3);
		motor_send();
	}
	HAL_TIM_Base_Stop_IT (&htim4 );
	speed_stop ();
	
	migong_gezi(Point2,Point3);  //粗加工区
	while ((xunji[0] < 15 || xunji [0] > 30) && (xunji[2] < 15 || xunji [2] > 30)) //物料区位置微调
	{
		if((xunji [0] < 15) && (xunji[2] < 15 ))
		{
			youyi_half();
		}		
		if((xunji [0] < 15) && (xunji[2] > 30 ))
		{
			zuoyi_half();
		}
		else 
		{
			youyi_half();
		}
	}
	speed_stop();
	motor_send ();
	HAL_Delay (200);
		
	while ((xunji[0] < 15 || xunji [0] > 30) && (xunji[2] < 15 || xunji [2] > 30)) //物料区位置微调
	{		
		if((xunji [0] < 15) && (xunji[2] > 30 ))
		{
			zuoyi_half();
		}
		else 
		{
			youyi_half();
		}
	}	
	
	while ((xunji[1] < 40 || xunji [1] > 80) && (xunji[3] < 40 || xunji [3] > 80)) //物料区位置微调
	{		
		if((xunji [1] < 40) && (xunji[3] > 80 ))
		{
			qianjin_half();
		}
		else 
		{
			houtui_half();
		}
	}
	
	
	
	while(1)  //程序运行卡死
	{
		LCD_ShowNum (20,200,tof[0],3,16,BLACK );
		LCD_ShowNum (90,200,tof[1],3,16,BLACK );
		LCD_ShowNum (160,200,tof[2],3,16,BLACK );

	}
		 
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void correct(int Three[3])
{	
//	LCD_ShowNum (180,0,K_ser ,3,16,BLACK );

	if (Three[1] > 200)
	{
		K_ser = K_ser + 25;
//		Kinematic_Analysis(x, y, ser,0,'F');
	}
	else if(Three[1] < 100)
	{
		K_ser = K_ser - 25;
//		Kinematic_Analysis(x, y, ser,0,'F');
	}

}

void k210_handle(void)
{
	switch(k210_state)
	{
		case 1:
			switch(QR_code_Order[0])
			{
				case 1:
					HAL_UART_Receive_IT(&huart5, USART5_RX_BUF ,12); //接收摄像头数据	
					HAL_UART_Transmit (&huart5 ,"111",3,0xff);
					break;
				case 2:
					HAL_UART_Receive_IT(&huart5, USART5_RX_BUF ,12); //接收摄像头数据	
					HAL_UART_Transmit (&huart5 ,"222",3,0xff);
					break;
				case 3:
					HAL_UART_Receive_IT(&huart5, USART5_RX_BUF ,12); //接收摄像头数据	
					HAL_UART_Transmit (&huart5 ,"333",3,0xff);
					break;
			}
			break ;
		case 2:
			switch(QR_code_Order[1])
			{
				case 1:
					HAL_UART_Receive_IT(&huart5, USART5_RX_BUF ,12); //接收摄像头数据	
					HAL_UART_Transmit (&huart5 ,"111",3,0xff);
					break;
				case 2:
					HAL_UART_Receive_IT(&huart5, USART5_RX_BUF ,12); //接收摄像头数据	
					HAL_UART_Transmit (&huart5 ,"222",3,0xff);
					break;
				case 3:
					HAL_UART_Receive_IT(&huart5, USART5_RX_BUF ,12); //接收摄像头数据	
					HAL_UART_Transmit (&huart5 ,"333",3,0xff);
					break;
			}
			break ;
		case 3:
			switch(QR_code_Order[2])
			{
				case 1:
					HAL_UART_Receive_IT(&huart5, USART5_RX_BUF ,12); //接收摄像头数据	
					HAL_UART_Transmit (&huart5 ,"111",3,0xff);
					break;
				case 2:
					HAL_UART_Receive_IT(&huart5, USART5_RX_BUF ,12); //接收摄像头数据	
					HAL_UART_Transmit (&huart5 ,"222",3,0xff);
					break;
				case 3:
					HAL_UART_Receive_IT(&huart5, USART5_RX_BUF ,12); //接收摄像头数据	
					HAL_UART_Transmit (&huart5 ,"333",3,0xff);
					break;
			}
			break ;

	}
}
void count(Heibai *Heibaix ,int p) //单线 以单线为点，进行运动判断
{
	Heibaix->count =xunji_count [p];
	if(Heibaix->count >= 5) 
	{
		Heibaix->xun_startflag = 1;  
	}
	if((Heibaix->xun_startflag == 1) && (Heibaix->count <= 2)) 
	{
		Heibaix->xun_endflag =1;
		
	}
	if((Heibaix->xun_endflag ==1 ) && (Heibaix->xun_startflag ==1))
	{
		Heibaix->xun_countflag =1;
		Heibaix->xun_endflag =0;
		Heibaix->xun_startflag =0;
	}	
	
}

void count_middle1(void)  //y轴 每过一单线，相应坐标加1
{
	
	count (&heibai_1 ,0);
	count (&heibai_3 ,2);
	
	if((xunji_flag [1] == 1) && (heibai_3 .xun_countflag == 1) && (xunji [1] >= 15))
	{
		car.y ++;
		heibai_3 .xun_countflag  = 0;
	}
	else if((xunji_flag [1] == 2) && (heibai_1 .xun_countflag == 1) && (xunji [1] >= 15))
	{
		car.y --;
		heibai_1 .xun_countflag  = 0;
	}
}

void count_middle2(void)  //x轴 每过一单线，相应坐标加1
{
	count (&heibai_2 ,1);
	
	if((xunji_flag [0] == 1) && (heibai_2 .xun_countflag == 1) && (xunji [2] >= 15))
	{
		car.x ++;
		heibai_2 .xun_countflag  = 0;
	}

}
void count_middle3(void)  //x轴 每过一单线，相应坐标加1
{
	count (&heibai_4 ,3);
	
	if((xunji_flag [0] == 2) && (heibai_4 .xun_countflag == 1) && (xunji [2] >= 15))
	{
		car.x --;
		heibai_4 .xun_countflag  = 0;
	}
}
void migong_gezi(Point now, Point next)  //迷宫格子  基于双线
{
	if((next .y - now .y)  > 0)
	{
		xunji_flag [1]=1;
		while(now.y != next .y )
		{
			now .y =car.y ;
			
			qianjin ();
			motor_send ();
			count_2 ();
		}
		
	}
	else
	{
		xunji_flag [1]=2;
		while(now.y != next .y )
		{
			now .y =car.y ;
			houtui ();
			motor_send ();
			count_2 ();
		}

	}

	if((next.x - now.x) > 0 )
	{
		xunji_flag [0]=1;
		while(now .x != next .x )
		{
			now.x =car .x ;
			youyi ();
			motor_send ();
			count_1 ();
		}
	}	
	else 
	{
		xunji_flag [0]=2;
		while(now .x != next .x)
		{
			now.x = car .x ;
			zuoyi ();
			motor_send ();
			count_1 ();
		}
	}	

}

void migong_line(Point now, Point next)  //迷宫线  基于单线
{
	if((next .y - now .y)  > 0)
	{
		xunji_flag [1]=1;
		while(now.y != next .y )
		{
			now .y =car.y ;
			qianjin ();
			motor_send ();
			count_middle1 ();
		}
		
	}
	
	else
	{
		xunji_flag [1]=2;
		while(now.y != next .y )
		{
			now .y =car.y ;
			houtui ();
			motor_send ();
			count_middle1 ();
		}

	}

	if((next.x - now.x) > 0 )
	{
		xunji_flag [0]=1;
		while(now .x != next .x )
		{
			now.x =car .x ;
			youyi ();
			motor_send ();
			count_middle2 ();
		}
	}	
	else 
	{
		xunji_flag [0]=2;
		while(now .x != next .x)
		{
			now.x = car .x ;
			zuoyi ();
			motor_send ();
			count_middle3 ();
		}
	}	

}

void count_1(void)  //双线x 每过一格，相应坐标加1
{

	count (&heibai_2 ,1);
	count (&heibai_4 ,3);
		
	if((heibai_2 .xun_countflag ==1) && (heibai_4 .xun_countflag == 1))
	{
		if(xunji_flag [0] ==1)
		{
			car.x ++;
		}
		else 
		{
			car.x --;
		}
		heibai_2 .xun_countflag =0;
		heibai_4 .xun_countflag =0;
	}
}

void count_2(void)  //双线y 每过一格，相应坐标加1
{
	
	count (&heibai_1 ,0);
	count (&heibai_3 ,2);
	
	if((heibai_1 .xun_countflag ==1) && (heibai_3 .xun_countflag == 1))
	{
		
		if(xunji_flag [1] ==1)
		{
			car.y ++;
		}
		else 
		{
			car.y --;
		}
		heibai_1 .xun_countflag =0;
		heibai_3 .xun_countflag =0;
	}
	
	
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  //定时器中断回传
{
	if(htim == &htim2 )   //200ms 进行寻迹pid及速度纠正
	{
		xunxian_pid(xun_flag);
//		Kinematic_Analysis(K_X, K_Y, K_ser,K_1,K_2);

	}
	if(htim == &htim3)    //50ms 进行tof接收 每50ms进行DMA中断刷新
	{
		DMA_receive2 ();  //DMA接收，下同
		DMA_receive3 ();
		DMA_receive4 ();
		tof_handle ();	  //接收数据处理
	}
	if(htim == &htim4)	  //转向时时间计算
	{
		int kk=0;
		time++;
		for(kk=0;kk<3;kk++)
		{
			if(time == time_compare[ kk])
			{
				time_flag[kk]= 0;
			}
		}
	}
	if(htim == &htim5)	  //k210
	{
		k210_handle();
	}

}
void tof_handle(void)     //tof数据处理
{	
	if(USART2_RX_BUF[26] == ' ')
	{
		if(USART2_RX_BUF [27] == ' ')
		{
			USART2_RX_BUF [26] =0;
			USART2_RX_BUF [27] =0;	
		}
		else 
		{
			USART2_RX_BUF [26] =0;
			USART2_RX_BUF [27] =USART2_RX_BUF [27] - '0';
		}	
	}
	tof[0] = USART2_RX_BUF [26] *100 + USART2_RX_BUF [27] * 10 + (USART2_RX_BUF[28] - '0');
	tof[0] = tof[0] - 2;
	if(USART3_RX_BUF[26] == ' ')
	{
		if(USART3_RX_BUF [27] == ' ')
		{
			USART3_RX_BUF [26] =0;
			USART3_RX_BUF [27] =0;	
		}
		else 
		{
			USART3_RX_BUF [26] =0;
			USART3_RX_BUF [27] =USART3_RX_BUF [27] - '0';
		}	
	}
	tof[1] = USART3_RX_BUF [26] *100 + USART3_RX_BUF [27] * 10 + (USART3_RX_BUF[28] - '0');
	tof[1] = tof[1] - 4;
	if(USART4_RX_BUF[26] == ' ')
	{
		if(USART4_RX_BUF [27] == ' ')
		{
			USART4_RX_BUF [26] =0;
			USART4_RX_BUF [27] =0;	
		}
		else 
		{
			USART4_RX_BUF [26] =0;
			USART4_RX_BUF [27] =USART4_RX_BUF [27] - '0';
		}	
	}
	tof[2] = USART4_RX_BUF [26] *100 + USART4_RX_BUF [27] * 10 + (USART4_RX_BUF[28] - '0');
}



void xun_pid_init(void)   //巡线速度pid初始化
{
	pid_init(&xun_pid[0],-3, 0, 2, 2000, 2000);
	pid_init(&xun_pid[1],-3, 0, 2.1, 2000, 2000);
	pid_init(&xun_pid[2],-3, 0, 2, 2000, 2000);
	pid_init(&xun_pid[3],-3, 0, 2.1, 2000, 2000);
}

void xunxian_pid(int xx)  //寻迹pid，对寻迹的返回值进行比例处理
{
	
	switch (xx)
	{
		case 1:	
				if(xunji [0] > 24)
				{
					xun_pid[0].kp = -1.5;
				}
				else 
				{
					xun_pid[0].kp =-3;
				}
				suduout[0]  = (int16_t)pid_calc1(&xun_pid[0], 24, xunji[0]);
				suduout[1]  = (int16_t)pid_calc1(&xun_pid[2], 24, xunji[2]);
				break ;
		case 2:
				if((xunji [1] > 24) || (xunji[3] > 24))
				{
					xun_pid[1].kp = -2.4;
					xun_pid[3].kp = -2.4;

				}
				else 
				{
					xun_pid[1].kp = -3.5;
					xun_pid[3].kp = -3.5;
				}
				if( (xunji[1] > 220) ||  (xunji[3] > 220))
				{
					break ;
				}
				suduout[0]  = (int16_t)pid_calc1(&xun_pid[1], 24, xunji[1]);
				suduout[1]  = (int16_t)pid_calc1(&xun_pid[3], 24, xunji[3]);

				break ;
		case 3:
				if((xunji [0] > 24) || (xunji[2] > 24))
				{
					xun_pid[0].kp = -2.5;
					xun_pid[2].kp = -2.5;
				}
				else 
				{
					xun_pid[0].kp =-4.2;
					xun_pid[2].kp =-4.2;
				}
				if( (xunji[0] > 220) ||  (xunji[2] > 220))
				{
					break ;
				}

				suduout[0]  = (int16_t)pid_calc1(&xun_pid[2], 24, xunji[2]);
				suduout[1]  = (int16_t)pid_calc1(&xun_pid[0], 24, xunji[0]);
				break ;
		case 4:
				if(xunji [3] > 24)
				{
					xun_pid[3].kp = -2;
				}
				else 
				{
					xun_pid[3].kp =-3;
				}

				suduout[0]  = (int16_t)pid_calc1(&xun_pid[3], 24, xunji[3]);
				suduout[1]  = (int16_t)pid_calc1(&xun_pid[1], 24, xunji[1]);

				break ;

		case 5:
				if(xunji [0] > 24)
				{
					xun_pid[0].kp = -2;
				}
				else 
				{
					xun_pid[0].kp =-3;
				}
				adj[0] = (int16_t)pid_calc1(&xun_pid[0], 24, xunji[0]);
				adj[1] = (int16_t)pid_calc1(&xun_pid[0], 24, xunji[2]);
				break ;
		case 6:
				if(xunji [1] > 24)
				{
					xun_pid[1].kp = -2;
				}
				else 
				{
					xun_pid[1].kp =-3;
				}
				adj[0] = (int16_t)pid_calc1(&xun_pid[1], 24, xunji[1]);
				adj[1] = (int16_t)pid_calc1(&xun_pid[1], 24, xunji[3]);
				break ;

				
	}

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //接收中断回调
{
	if(huart == &huart5 )
	{
		Dael_Coordinate(USART5_RX_BUF);
	}
}

int fputc(int ch ,FILE *f)  //printf重定义
{
	uint8_t temp[1] = {ch};
	HAL_UART_Transmit(&huart3, temp, 1, 2); //huart1需要根据你的配置修改
	return ch;
}


void DMA_receive2(void) 
{
	int j=0;
	
	if(recv_end_flag2 == 1)  
	{
		if(rx_len2 == 35)
		{
			for(j=0;j<rx_len2;j++)
			{
				USART2_RX_BUF[j]=rx_buffer2[j];
			}
		}
//		len[2] = rx_len2 ;
		rx_len2 = 0;
		recv_end_flag2 = 0;		
	}		
	HAL_UART_Receive_DMA(&huart2,rx_buffer2,200);
}
void DMA_receive3(void)  
{
	int n=0;
	if(recv_end_flag3 == 1)  
	{
		if(rx_len3 == 35)
		{
			for(n=0;n<rx_len3;n++)
			{
				USART3_RX_BUF[n]=rx_buffer3[n];
			}
		}
//		len[3] = rx_len3 ;
		rx_len3 = 0;
		recv_end_flag3 = 0;	
	}
	
//	while(HAL_UART_GetState(&huart2 )== HAL_UART_STATE_BUSY_TX );	
//	HAL_UART_Transmit (&huart2 ,"12345678" ,8,0xff);	
	HAL_UART_Receive_DMA(&huart3,rx_buffer3,200);

}
void DMA_receive4(void) 
{
	int m=0;
	if(recv_end_flag4 == 1)  
	{
		if(rx_len4 == 35)
		{
			for(m=0;m<rx_len4;m++)
			{
				USART4_RX_BUF[m]=rx_buffer4[m];
			}
		}
//		len[4] = rx_len4 ;
		rx_len4 = 0;
		recv_end_flag4 = 0;		
	}
//	while(HAL_UART_GetState(&huart5 )== HAL_UART_STATE_BUSY_TX );
//	HAL_UART_Transmit (&huart5 ,usart4 ,30,0xffff);
	
	HAL_UART_Receive_DMA(&huart4,rx_buffer4,200);

}

void qianjin_half(void)
{
	xun_flag = 3;
	speed_data [0]=-100+(suduout[0]+suduout [1]) / 4;
	speed_data [1]=-100+(suduout[0]+suduout [1]) / 4;
	speed_data [2]=100+(suduout[0]+suduout [1]) / 4;
	speed_data [3]=100+(suduout[0]+suduout [1]) / 4;

	for(i=0;i<4;i++)
	{
		motor_msg[i]=speed_data[i] + 32000;
		message[i][0]=motor_msg[i] %256;
		message[i][1]=motor_msg[i] /256;
	}
	motor_send ();
}
void houtui_half(void)  //half 为寻迹移动，但速度较慢
{
	xun_flag =3;
	speed_data [0]=50+(suduout[0]+suduout [1]) / 4;
	speed_data [1]=50+(suduout[0]+suduout [1]) / 4;
	speed_data [2]=-50+(suduout[0]+suduout [1]) / 4;
	speed_data [3]=-50+(suduout[0]+suduout [1]) / 4;

	for(i=0;i<4;i++)
	{
		motor_msg[i]=speed_data[i] + 32000;
		message[i][0]=motor_msg[i] %256;
		message[i][1]=motor_msg[i] /256;
	}
	motor_send ();
}
void zuoyi_half(void)
{
	xun_flag =2;
	speed_data [0]=50+(suduout[0]+suduout [1]) / 4;
	speed_data [1]=-50+(suduout[0]+suduout [1]) / 4;
	speed_data [2]=-50+(suduout[0]+suduout [1]) / 4;
	speed_data [3]=50+(suduout[0]+suduout [1]) / 4;
	
	
	for(i=0;i<4;i++)
	{
		motor_msg[i]=speed_data[i] + 32000;
		message[i][0]=motor_msg[i] %256;
		message[i][1]=motor_msg[i] /256;
	}
	motor_send ();

}
void youyi_half()
{
	xun_flag =2;
	speed_data [0]=-50+(suduout[0]+suduout [1]) / 4;
	speed_data [1]=50+(suduout[0]+suduout [1]) /4;
	speed_data [2]=50+(suduout[0]+suduout [1]) / 4;
	speed_data [3]=-50+(suduout[0]+suduout [1]) / 4;
	
	
	for(i=0;i<4;i++)
	{
		motor_msg[i]=speed_data[i] + 32000;
		message[i][0]=motor_msg[i] %256;
		message[i][1]=motor_msg[i] /256;
	}
	motor_send ();

}
void qianjin_total(void) //total为直接移动
{
	speed_set (0,-250,0);
	motor_send ();
	
}
void houtui_total(void)
{
	speed_set (0,150,0);
	motor_send ();
	
}

void youyi_total(void)
{
	xun_flag =2;
	speed_data [0]=-100;
	speed_data [1]=100;
	speed_data [2]=100;
	speed_data [3]=-100;
	
	
	for(i=0;i<4;i++)
	{
		motor_msg[i]=speed_data[i] + 32000;
		message[i][0]=motor_msg[i] %256;
		message[i][1]=motor_msg[i] /256;
	}
	motor_send ();
}
void adjust(void)  		 //tof的校准，通过两个tof的距离差进行自转纠正
{
	int yyy;
	int err;
	
	err = tof[0] - tof[1];
	if(err == 0)
	{
		yyy = 0;
	}
	else if(err > 5)
	{
		yyy = 1;
	}
	else if(err < -5)
	{
		yyy = 2;
	}
	else if(err > 2)
	{
//		yyy = 3;
		yyy = 1;

	}
	else if(err < -2)
	{
//		yyy = 4;
		yyy = 2;

	}

	switch(yyy)
	{
		case 0:
			speed_stop();
			break;
		case 1:
			speed_set_without_pid (0,0,-3);
			motor_send();
			break;	
		case 2:
			speed_set_without_pid(0,0,3);
			motor_send();
			break;	
		case 3:
			speed_set_without_pid(0,0,-1);
			motor_send();
			break;	
		case 4:
			speed_set_without_pid(0,0,1);
			motor_send();
			break;	
	}
	
	
	
}

void qianjin(void)
{
	xun_flag = 3;
	speed_data [0]=-250+(suduout[0]+suduout [1]) / 2;
	speed_data [1]=-250+(suduout[0]+suduout [1]) / 2;
	speed_data [2]=250+(suduout[0]+suduout [1]) / 2;
	speed_data [3]=250+(suduout[0]+suduout [1]) / 2;
	
	
	for(i=0;i<4;i++)
	{
		motor_msg[i]=speed_data[i] + 32000;
		message[i][0]=motor_msg[i] %256;
		message[i][1]=motor_msg[i] /256;
	}
	motor_send ();

}
void houtui(void)  //寻迹移动，速度较快
{
	xun_flag =3;
	speed_data [0]=250+(suduout[0]+suduout [1]) / 2;
	speed_data [1]=250+(suduout[0]+suduout [1]) / 2;
	speed_data [2]=-250+(suduout[0]+suduout [1]) / 2;
	speed_data [3]=-250+(suduout[0]+suduout [1]) / 2;
	
	
	for(i=0;i<4;i++)
	{
		motor_msg[i]=speed_data[i] + 32000;
		message[i][0]=motor_msg[i] %256;
		message[i][1]=motor_msg[i] /256;
	}
	motor_send ();

}
void zuoyi(void)
{
	xun_flag =2;
	speed_data [0]=300+(suduout[0]+suduout [1]) / 2;
	speed_data [1]=-300+(suduout[0]+suduout [1]) / 2;
	speed_data [2]=-300+(suduout[0]+suduout [1]) / 2;
	speed_data [3]=300+(suduout[0]+suduout [1]) / 2;
	
	
	for(i=0;i<4;i++)
	{
		motor_msg[i]=speed_data[i] + 32000;
		message[i][0]=motor_msg[i] %256;
		message[i][1]=motor_msg[i] /256;
	}
	motor_send ();

}
void youyi(void)
{
	xun_flag =2;
	speed_data [0]=-300+(suduout[0]+suduout [1]) / 2;
	speed_data [1]=300+(suduout[0]+suduout [1]) / 2;
	speed_data [2]=300+(suduout[0]+suduout [1]) / 2;
	speed_data [3]=-300+(suduout[0]+suduout [1]) / 2;
	
	
	for(i=0;i<4;i++)
	{
		motor_msg[i]=speed_data[i] + 32000;
		message[i][0]=motor_msg[i] %256;
		message[i][1]=motor_msg[i] /256;
	}
	motor_send ();

}


void move_ER(void) 
{
	while( ( (tof[0] + tof[1]) / 2) < 37)    //移动至45cm处
	{
		qianjin_total();
	}
	
	migong_gezi (Point_0,Point_1);  //移动至2维码区
	speed_stop ();
	motor_send ();
	HAL_Delay (1000);

}
void move_first(void)
{
	migong_line (Point_1,Point_2);  //移动至物料区		
	speed_stop ();
	motor_send ();
	HAL_Delay (200);

	while ((xunji[0] < 15 || xunji [0] > 30) && (xunji[2] < 15 || xunji [2] > 30)) //物料区位置微调
	{
		if((xunji [0] < 15) && (xunji[2] > 30 ))
		{
			zuoyi_half();
		}
		else 
		{
			youyi_half ();
		}
	}
	
	while( tof[0] > 39 )  //物料区距离微调
	{
		houtui_half();
	}
	speed_stop();
	HAL_Delay(1000);	
}
void move_second(void)
{
	time = 0;  //旋转 
	time_flag[0] = 1;
	HAL_TIM_Base_Start_IT(&htim4 );
	while(time_flag[0])
	{	
		speed_set_without_pid(0,0,3);
		motor_send();
	}
	HAL_TIM_Base_Stop_IT (&htim4 );
	speed_stop ();
	
	migong_line(Point_2,Point_3);  //粗加工区
	speed_stop ();
	motor_send ();
	HAL_Delay (500);
	
	while ((xunji[0] < 15 || xunji [0] > 30) && (xunji[2] < 15 || xunji [2] > 30)) //粗加工区位置微调
	{
		if((xunji [0] < 15) && (xunji[2] > 30 ))
		{
			zuoyi_half();
		}
		else 
		{
			youyi_half ();
		}
	}
	
	while( tof[0] > 40 )  //粗加工距离调整
	{
		houtui_half();
	}
	speed_stop();
	HAL_Delay(500);
}
void move_third(void)
{
	while ((xunji[1] < 15 || xunji [1] > 30) && (xunji[3] < 15 || xunji [3] > 30))  //退后
	{
		qianjin_half ();
	}
	
	migong_line (Point_3,Point_4);  //移动转弯
	speed_stop ();
	motor_send ();
	
	while ((xunji[0] < 15 || xunji [0] > 30) && (xunji[2] < 15 || xunji [2] > 30))  //转弯对齐
	{
		if((xunji [0] < 15) && (xunji[2] > 30 ))
		{
			zuoyi_half();
		}
		else 
		{
			youyi_half ();
		}
	}

	time_flag[1] = 1;   //旋转
	time = 0;
	HAL_TIM_Base_Start_IT(&htim4 );
	while(time_flag[1])
	{	
		speed_set_without_pid(0,0,3);
		motor_send();
	}
	HAL_TIM_Base_Stop_IT (&htim4 );
	speed_stop ();
	HAL_Delay (100);
		
	migong_line (Point_4,Point_5);   //成品区
	speed_stop ();
	motor_send ();	
	while ((xunji[0] < 15 || xunji [0] > 30) && (xunji[2] < 15 || xunji [2] > 30))  //成品区位置微调
	{
		if((xunji [0] < 15) && (xunji[2] > 30 ))
		{
			zuoyi_half();
		}
		else 
		{
			youyi_half ();
		}
	}
	speed_stop();
	HAL_Delay(1000);	
}
void move_fourth(void)
{
	migong_line (Point_5,Point_6);  //移动至转弯处
	speed_stop ();
	HAL_Delay (500);
	while ((xunji[1] < 15 || xunji [1] > 30) && (xunji[3] < 15 || xunji [3] > 30)) //转弯处对齐
	{
		if((xunji [1] > 30) && (xunji[3] < 15 ) )
		{
			qianjin_half();
		}
		else if((xunji [1] < 15) && (xunji[3] > 30 ))
		{
			houtui_half();
		}
	}
	
	time_flag[2] = 1;   //旋转
	time = 0;
	HAL_TIM_Base_Start_IT(&htim4 );
	while(time_flag[2])
	{	
		speed_set_without_pid(0,0,7);
		motor_send();
	}
	HAL_TIM_Base_Stop_IT (&htim4 );
	speed_stop ();

	migong_line (Point_6,Point_7);  
	speed_stop ();
	motor_send ();
//	HAL_Delay (500);
	
	while ((xunji[0] < 15 || xunji [0] > 30) && (xunji[2] < 15 || xunji [2] > 30)) 
	{
		if((xunji [0] < 15) && (xunji[2] > 30 ))
		{
			zuoyi_half();
		}
		else 
		{
			youyi_half ();
		}
	}
	speed_stop();
	HAL_Delay (500);
		
	while( tof[0] > 40 )  //物料区微调
	{
		houtui_half();
	}
	speed_stop();
	HAL_Delay(1000);


}
void move_end(void)	
{	
	car .x =0;
	car .y =0;
	migong_line(Point_8 ,Point_9 );
	speed_stop ();
	motor_send ();
	HAL_Delay (500);
	
	while(tof[2] > 7)
	{
		youyi_total();
	}
	while(tof [0] > 6)
	{
		houtui_total ();
	}
	speed_stop();
}
void total(void)
{


//	LCD_ShowString (200,0,31,10,24,BLACK ,"on");
//	LCD_ShowNum (200,240,0,3,16,BLACK );
//	  
//	while(k210_state == 0)
//	{
//		HAL_UART_Receive_IT(&huart5, USART5_RX_BUF  ,12);	//串口接收中断，接收陀螺仪数据
//		HAL_UART_Transmit (&huart5 ,"123",3,0xff);
//		LCD_ShowNum (100,240,k210_state,3,16,BLACK );
//		if(QR_code_Order[0] != 0)
//		{
//			k210_state = 1;
//			break;
//		}
//	}
	
//	LCD_ShowNum (100,240,k210_state,3,16,BLACK );
//	HAL_GPIO_WritePin  (led_GPIO_Port ,led_Pin ,GPIO_PIN_RESET );
//	LCD_ShowNum (0,120,(uint32_t)QR_code_Order[0],3,16,BLACK );
//	LCD_ShowNum (45,120,(uint32_t)QR_code_Order[1],3,16,BLACK );
//	LCD_ShowNum (90,120,(uint32_t)QR_code_Order[2],3,16,BLACK );
//	LCD_ShowNum (0,150,(uint32_t)QR_code_Order[3],3,16,BLACK );
//	LCD_ShowNum (45,150,(uint32_t)QR_code_Order[4],3,16,BLACK );
//	LCD_ShowNum (90,150,(uint32_t)QR_code_Order[5],3,16,BLACK );
//显示二维码信息
	
//	HAL_Delay (1000);

//	while( k210_state == 1)
//	{
//		LCD_ShowNum (100,240,k210_state,3,16,BLACK );
//		HAL_TIM_Base_Start_IT(&htim5 ); //定时器2与3中断开启
//		LCD_ShowNum (0,120,(uint32_t)QR_code_Order[0],3,16,BLACK );
//		LCD_ShowNum (45,120,(uint32_t)QR_code_Order[1],3,16,BLACK );
//		LCD_ShowNum (90,120,(uint32_t)QR_code_Order[2],3,16,BLACK );
//		LCD_ShowNum (0,150,(uint32_t)QR_code_Order[3],3,16,BLACK );
//		LCD_ShowNum (45,150,(uint32_t)QR_code_Order[4],3,16,BLACK );
//		LCD_ShowNum (90,150,(uint32_t)QR_code_Order[5],3,16,BLACK );

//		LCD_ShowNum (0,0,Three[0],5,16,BLACK );
//		LCD_ShowNum (45,0,Three[1],5,16,BLACK );
//		LCD_ShowNum (90,0,Three[2],5,16,BLACK );
////显示二维码与 物块坐标

//		correct(Three);
////移动Z轴
//	}
//	HAL_TIM_Base_Stop_IT(&htim5 ); //定时器2与3中断开启
	
	
	
	 while(HAL_GPIO_ReadPin (key2_GPIO_Port ,key1_Pin ) == 0) //按键1开关
	 {
		 
		LCD_ShowString (0,0,30,10,16,BLACK ,"off");
		 
		LCD_ShowNum (50,50,yaw_931/100,1,16,BLACK );
		LCD_ShowNum (70,50,yaw_931/10 %10,1,16,BLACK );
		LCD_ShowNum (90,50,yaw_931%10,1,16,BLACK );
		 
		LCD_ShowChar (80,100,USART1_RX_BUF[6],16,BLACK ,0);

		LCD_ShowNum (80,120,USART1_RX_BUF[6] - 0,3,16,BLACK );
		LCD_ShowNum (120,120,USART1_RX_BUF[7] - 0,3,16,BLACK );

		LCD_ShowNum (175,0,31,3,24,BLACK );

		LCD_ShowNum (80,150,xunji [0],3,16,BLACK );
		LCD_ShowNum (120,150,xunji [2],3,16,BLACK );
		LCD_ShowNum (20,200,tof[0],3,16,BLACK );
		LCD_ShowNum (90,200,tof[1],3,16,BLACK );
		LCD_ShowNum (160,200,tof[2],3,16,BLACK );
	 }
	 

	 
//运动开始	 
	 while(1)
	 {
		LCD_ShowString (0,0,31,10,16,BLACK ,"on");
		 
/*运动至二维码区
*/		 
		move_ER();
		 
/*机械臂伸出识别2维码
*/		 
//		CANx_SendExtData(0x00415010,ER_go,8);
//		HAL_Delay(5000);
//		CANx_SendExtData(0x00415010,ER_stop,8);//发送停止 
		 
/*等待二维码数据返回
开启定时器5中断，state = 0	 
*/
//		while(k210_state == 0)
//		{
//			HAL_UART_Transmit (&huart5 ,"123",3,0xff);
//			LCD_ShowNum (100,150,k210_state,3,16,BLACK );
//		}
//		HAL_GPIO_WritePin  (led_GPIO_Port ,led_Pin ,GPIO_PIN_RESET );
//		LCD_ShowNum (0,120,(uint32_t)QR_code_Order[0],3,16,BLACK );
//		LCD_ShowNum (45,120,(uint32_t)QR_code_Order[1],3,16,BLACK );
//		LCD_ShowNum (90,120,(uint32_t)QR_code_Order[2],3,16,BLACK );
//		LCD_ShowNum (0,150,(uint32_t)QR_code_Order[3],3,16,BLACK );
//		LCD_ShowNum (45,150,(uint32_t)QR_code_Order[4],3,16,BLACK );
//		LCD_ShowNum (90,150,(uint32_t)QR_code_Order[5],3,16,BLACK );

		 
/*机械臂缩回
*/
//		CANx_SendExtData(0x00415010,ER_back ,8);
//		HAL_Delay(5000);
//		CANx_SendExtData(0x00415010,ER_back_stop ,8);//发送停止 
		
		HAL_Delay(200);
/*移动物料区
*/
		move_first();


/*  舵机抓取		
*/
//		CANx_SendExtData(0x00415010,first_left_up,8);
//		HAL_Delay(8000);
//		HAL_Delay(2000);
//		CANx_SendExtData(0x00415010,first_left_up_done,8);//发送停止 
//		
//		CANx_SendExtData(0x00415010,first_middle_up,8);
//		HAL_Delay(8000);
//		HAL_Delay(2000);
//		CANx_SendExtData(0x00415010,first_middle_up_done,8);//发送停止 
//		
//		
//		CANx_SendExtData(0x00415010,first_right_up,8);
//		HAL_Delay(8000);
//		HAL_Delay(2000);
//		CANx_SendExtData(0x00415010,first_right_up_done,8);//发送停止 
		
		
		HAL_Delay(1000);
		
/*移动粗加工区
*/
		move_second();
		
/* 粗加工放置
*/
//		CANx_SendExtData(0x00415010,second_left,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,second_left_done,8);//发送停止 
//		
//		CANx_SendExtData(0x00415010,second_middle,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,second_middle_done,8);//发送停止 

//		CANx_SendExtData(0x00415010,second_right,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,second_right_done,8);//发送停止 
//		
//		HAL_Delay(1000);
		
/*移动成品区
*/
		move_third();
		
		
/*  成品区放置		
*/
//		CANx_SendExtData(0x00415010,third_left_up,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,third_left_up_done,8);//发送停止 

//		CANx_SendExtData(0x00415010,third_middle_up,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,third_middle_up_done,8);//发送停止 

//		CANx_SendExtData(0x00415010,third_right_up ,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,third_right_up_done,8);//发送停止 
//	
//	
//		HAL_Delay (1000);
		
/*移动原料区
*/
		move_fourth();
		
		
/*  舵机抓取
*/
//		CANx_SendExtData(0x00415010,first_left_down,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,first_left_down_done,8);//发送停止 
//		
//		CANx_SendExtData(0x00415010,first_middle_down,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,first_middle_down_done,8);//发送停止 
//		
//		
//		CANx_SendExtData(0x00415010,first_right_down,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,first_right_down_done,8);//发送停止 
//		
//		HAL_Delay (1000);
		
/*移动粗加工区
*/
		car.x = 6;
		car.y = 0;
		time_compare [0] = 30;
		move_second();
/* 粗加工放置		
*/
//		CANx_SendExtData(0x00415010,second_left,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,second_left_done,8);//发送停止 
//		
//		CANx_SendExtData(0x00415010,second_middle,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,second_middle_done,8);//发送停止 

//		CANx_SendExtData(0x00415010,second_right,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,second_right_done,8);//发送停止 
//		
//		HAL_Delay(1000);
		
/*移动成品区
*/
		move_third();
		
/*  成品区放置		
*/	
//		CANx_SendExtData(0x00415010,third_left_down,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,third_left_down_done,8);//发送停止 

//		CANx_SendExtData(0x00415010,third_middle_down,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,third_middle_down_done,8);//发送停止 

//		CANx_SendExtData(0x00415010,third_right_down ,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,third_right_down_done,8);//发送停止 
//	
//	
//		HAL_Delay (1000);
		
/*移动结束
*/
		move_end();
// 		

		while(1)
		{
			HAL_GPIO_TogglePin (led_GPIO_Port ,led_Pin );
			HAL_Delay(500);
			LCD_ShowString (0,0,30,10,16,BLACK ," on");
			
			LCD_ShowNum (50,50,yaw_931/100,1,16,BLACK );
			LCD_ShowNum (70,50,yaw_931/10 %10,1,16,BLACK );
			LCD_ShowNum (90,50,yaw_931%10,1,16,BLACK );

			LCD_ShowChar (80,100,USART1_RX_BUF[6],16,BLACK ,0);

			LCD_ShowNum (80,120,USART1_RX_BUF[6] - 0,3,16,BLACK );
			LCD_ShowNum (120,120,USART1_RX_BUF[7] - 0,3,16,BLACK );
			
			LCD_ShowNum (80,150,xunji [0],3,16,BLACK );
			LCD_ShowNum (120,150,xunji [2],3,16,BLACK );
			LCD_ShowNum (20,200,tof[0],3,16,BLACK );
			LCD_ShowNum (90,200,tof[1],3,16,BLACK );
			LCD_ShowNum (160,200,tof[2],3,16,BLACK );
		}
	 }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

