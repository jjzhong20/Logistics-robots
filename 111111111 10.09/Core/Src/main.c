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
#include "servo.h"   //��е�������ļ�
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
int16_t speed_data[4];   //����ٶȴ������
int16_t speed_data1[4];  //����ٶȴ������
int16_t motor_msg[4];    //����ٶȴ������
int x=0,y=0,yw=0;        //���xy�ٶ�ֵ����ת�������������ֽ���ʱ�����ٶȷ���

int i=0; 				 //����ٶȶ�����CAN�������ݵ�ѭ������

int16_t v_back[4]={0};       //CAN�ߵ������ֵ�����յ���������ص�����ٶ�
int16_t bianmaqi_back[4];    //��������������
uint8_t message[7][8]={0};   //can�������ݵĴ������
int ID[4]={0x00210010,0x00220010,0x00230010,0x00240010};  //ID������ǰ��λΪ���ID��

int servo_x = 120,servo_y = 150,servo_z = 1500,zhua = 0;  //��ʼ��е��λ��
int Three[3] = {0,0,0};                    //��������ͷ����������ά����x,y,z
char Color[3]; 											  //�����ɫ
uint8_t QR_code_Order[6] = {0,0,0,0,0,0}; 					  //��ά���ϵ�ץȡ˳������˳����123+321��
uint8_t pData[30] = {0};



//�Թ���������
int xunji_count[4];		  //Ѱ�����أ����Ѱ���Ҷȷ��ظ���
int xunji[4];			  //Ѱ������ֵ��ţ�4· ��8Ѱ��ֵ��0-255֮�䣬��Ӧ1�ֽ�
int xunji_flag[4]={0};	  //Ѱ����־λ�������Թ�����ʱ��־�ı������ߣ��Դ˸�����������

//����2ά���������
int len[5]={0};
uint8_t USART1_RX_BUF[200];  //�������ݽ���
uint8_t USART2_RX_BUF[200];  //�������ݽ���
uint8_t USART3_RX_BUF[200];  //�������ݽ���
uint8_t USART4_RX_BUF[200];  //�������ݽ���
uint8_t USART5_RX_BUF[200];  //�������ݽ���

int tof[3]={0};              //tof���ݴ������ ��λcm
int tof_adj[3]={0};			 //tof�����˲����ţ�����ش����ʲ�������ͣʹ��
int tof_count[3] = {0};
int tof_sum[3] ={0};

int yaw_931=0; 				 //�����Ƿ�������

int suduout[2];				 //Ѱ��pid���
pid_struct_t xun_pid[4];     //Ѱ��pid�ṹ��
int xun_flag=0; 			 //Ѱ����־λ����ǰ�������ƿ��ƣ���ӦѰ��ʱ���жϼ�Ѱ��pid������ٶȱ仯

////�ĸ�8·Ѱ��   �ڰ׽ṹ�壬�����ж�С���ƶ�����
Heibai heibai_1={0,0,0,0,0};
Heibai heibai_2={0,0,0,0,0};
Heibai heibai_3={0,0,0,0,0};
Heibai heibai_4={0,0,0,0,0};

//��ʱû��											// ���500������ǰ��1��400��������ת��һȦ
car_count count_InitAera={0,0,0,0,700,1};    	 	//����λ��
car_count count_QRCodeAera={0,0,0,0,1000,3};  		//2ά��λ��
car_count count_RawMaterialArea={0,0,0,0,1900,3};   //ԭ����
car_count count_firstconor1={0,0,0,0,250,3};  	    //ԭ������ּӹ���ת��1
car_count count_firstconor2={0,0,0,0,500,5};   	    //ԭ������ּӹ���ת��2
car_count count_ProcessingAera={0,0,0,0,1200,3};    //�ּӹ����������ϵ�ַ
car_count count_secondconor1={0,0,0,0,1250,3};      //�ּӹ����������ת��1
car_count count_secondconor2={0,0,0,0,610,5};    	//�ּӹ����������ת��2
car_count count_FinishedAera={0,0,0,0,1250,3};  	//������������ϵ�ַ
car_count count_third1conor={0,0,0,0,2500,1};    	//���������ԭ����1
car_count count_third2conor={0,0,0,0,1250,5};    	//���������ԭ����2
car_count count_third3conor={0,0,0,0,1100,3};    	//���������ԭ����3
car_count now={0,0,0,0,0,0}; 						//ʵʱ����
car_count last={0,0,0,0,0,0}; 						//�ϴ�������
car_count zuoyi_one={0,0,0,0,500,3};
car_count zero={0,0,0,0,0,1};
car_count qianjing_half={0,0,0,0,250,7};
car_count tui_half={0,0,0,0,250,2};
car_count xuanzhuan90={0,0,0,0,600,5};
car_count tui_half_half={0,0,0,0,150,2};

//��ʱ��������С����ת
int time_compare[3]={32,30,30};
int time=0;
int time_flag[3] = {1,1,1};

//����
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


//������ж�����
//��ά����
uint8_t ER_go[8] = {4,0,0,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t ER_stop[8] = {8,0,0,0,0,0,0,0};  // 1λΪ4��ʼ���У�Ϊ8��ֹͣ����
uint8_t ER_back[8] = {4,0,1,0,0,0,0,0}; 
uint8_t ER_back_stop[8] = {8,0,1,0,0,0,0,0}; 
//ԭ����
//�ϲ�
uint8_t first_left_up[8] = {4,0,2,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t first_left_up_done[8] = {8,0,2,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t first_middle_up[8] = {4,0,3,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t first_middle_up_done[8] = {8,0,3,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t first_right_up[8] = {4,0,4,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t first_right_up_done[8] = {8,0,4,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
//�²�
uint8_t first_left_down[8] = {4,0,5,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t first_left_down_done[8] = {8,0,5,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t first_middle_down[8] = {4,0,6,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t first_middle_down_done[8] = {8,0,6,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t first_right_down[8] = {4,0,7,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t first_right_down_done[8] = {8,0,7,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������

//�ּӹ���
uint8_t second_left[8] = {4,0,8,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t second_left_done[8] = {8,0,8,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t second_middle[8] = {4,0,9,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t second_middle_done[8] = {8,0,9,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t second_right[8] = {4,0,10,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t second_right_done[8] = {8,0,10,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������

//��Ʒ��

//�²�
uint8_t third_left_up[8] = {4,0,11,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t third_left_up_done[8] = {8,0,11,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t third_middle_up[8] = {4,0,12,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t third_middle_up_done[8] = {8,0,12,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t third_right_up[8] = {4,0,13,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t third_right_up_done[8] = {8,0,13,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
//���
uint8_t third_left_down[8] = {4,0,14,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t third_left_down_done[8] = {8,0,14,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t third_middle_down[8] = {4,0,15,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t third_middle_down_done[8] = {8,0,15,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t third_right_down[8] = {4,0,16,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������
uint8_t third_right_down_done[8] = {8,0,16,0,0,0,0,0}; //0λΪ���������п�ʼ�����  2λΪ��������


uint16_t  k210_state = 0;


//15, 15, 500,0,'F'
double K_X = 15, K_Y = 15, K_ser =600,K_1 = 0;
uint8_t K_2 = 1;
/*
��k210�����ݽ���

1.
	����ͷ�ϵ磬����MCU state״̬Ϊ0������12λ���ڽ����ж�
2.
	�����ƶ���ָ��λ�ã�ͨ������5����123��k210��
3.
	k210�������ݸ�MCU���ڴ����жϻص�������д������ɹ����յ����ݣ���state״̬Ϊ1����������һ�δ��ڽ����ж�
��û�н��յ����ݣ����������123
4.
	���ݶ�ά������ݣ�����ץȡʱ��˳����
����
	�� 111
	�� 222
	�� 333
	������յ���ά����   123+123
	
	����ͨ�����ڷ���111��k210��k210���غ�ɫ����XY����;���
	
	MCUÿ����111����һ�Σ�k210����һ�Σ����ڶ������ݽ��д����ж������Ƿ�������λ��
	������ı�״̬stateΪ2����������state��Ϊ1��ͬʱ������һ�η��ͣ����鷢�͵ļ����100ms
	
	stateΪ2�󣬴��ڷ���222������ͬ��


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
  
//  HAL_UART_Receive_IT (&huart5,USART5_RX_BUF ,12);	//���ڽ����жϣ���������������
  LCD_Init ();		  //LCD��ʼ��
  motor_pid_init();   //���pid��ʼ��  
  xun_pid_init(); 	  //Ѱ��pid��ʼ��
  CAN_filter_Init();  //CAN��������ʼ��
  
  HAL_TIM_Base_Start_IT(&htim2 ); //��ʱ��2��3�жϿ���
  HAL_TIM_Base_Start_IT(&htim3 );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	LCD_ShowNum (0,0,0,3,16,BLACK );
	
	while(tof[2] < 8)  //�ƶ�
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



	migong_gezi(Point0 ,Point1 ); //�ƶ���ά����
	speed_stop();
	motor_send ();
	HAL_Delay (1000);
	
	
	migong_gezi(Point1 ,Point2 );  //�ƶ�������
	while ((xunji[0] < 15 || xunji [0] > 30) && (xunji[2] < 15 || xunji [2] > 30)) //������λ��΢��
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
		
	while ((xunji[0] < 15 || xunji [0] > 30) && (xunji[2] < 15 || xunji [2] > 30)) //������λ��΢��
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
	
	while ((xunji[1] < 40 || xunji [1] > 80) && (xunji[3] < 40 || xunji [3] > 80)) //������λ��΢��
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
	
	time = 0;  //��ת 
	time_flag[0] = 1;
	HAL_TIM_Base_Start_IT(&htim4 );
	while(time_flag[0])
	{	
		speed_set_without_pid(0,0,3);
		motor_send();
	}
	HAL_TIM_Base_Stop_IT (&htim4 );
	speed_stop ();
	
	migong_gezi(Point2,Point3);  //�ּӹ���
	while ((xunji[0] < 15 || xunji [0] > 30) && (xunji[2] < 15 || xunji [2] > 30)) //������λ��΢��
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
		
	while ((xunji[0] < 15 || xunji [0] > 30) && (xunji[2] < 15 || xunji [2] > 30)) //������λ��΢��
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
	
	while ((xunji[1] < 40 || xunji [1] > 80) && (xunji[3] < 40 || xunji [3] > 80)) //������λ��΢��
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
	
	
	
	while(1)  //�������п���
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
					HAL_UART_Receive_IT(&huart5, USART5_RX_BUF ,12); //��������ͷ����	
					HAL_UART_Transmit (&huart5 ,"111",3,0xff);
					break;
				case 2:
					HAL_UART_Receive_IT(&huart5, USART5_RX_BUF ,12); //��������ͷ����	
					HAL_UART_Transmit (&huart5 ,"222",3,0xff);
					break;
				case 3:
					HAL_UART_Receive_IT(&huart5, USART5_RX_BUF ,12); //��������ͷ����	
					HAL_UART_Transmit (&huart5 ,"333",3,0xff);
					break;
			}
			break ;
		case 2:
			switch(QR_code_Order[1])
			{
				case 1:
					HAL_UART_Receive_IT(&huart5, USART5_RX_BUF ,12); //��������ͷ����	
					HAL_UART_Transmit (&huart5 ,"111",3,0xff);
					break;
				case 2:
					HAL_UART_Receive_IT(&huart5, USART5_RX_BUF ,12); //��������ͷ����	
					HAL_UART_Transmit (&huart5 ,"222",3,0xff);
					break;
				case 3:
					HAL_UART_Receive_IT(&huart5, USART5_RX_BUF ,12); //��������ͷ����	
					HAL_UART_Transmit (&huart5 ,"333",3,0xff);
					break;
			}
			break ;
		case 3:
			switch(QR_code_Order[2])
			{
				case 1:
					HAL_UART_Receive_IT(&huart5, USART5_RX_BUF ,12); //��������ͷ����	
					HAL_UART_Transmit (&huart5 ,"111",3,0xff);
					break;
				case 2:
					HAL_UART_Receive_IT(&huart5, USART5_RX_BUF ,12); //��������ͷ����	
					HAL_UART_Transmit (&huart5 ,"222",3,0xff);
					break;
				case 3:
					HAL_UART_Receive_IT(&huart5, USART5_RX_BUF ,12); //��������ͷ����	
					HAL_UART_Transmit (&huart5 ,"333",3,0xff);
					break;
			}
			break ;

	}
}
void count(Heibai *Heibaix ,int p) //���� �Ե���Ϊ�㣬�����˶��ж�
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

void count_middle1(void)  //y�� ÿ��һ���ߣ���Ӧ�����1
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

void count_middle2(void)  //x�� ÿ��һ���ߣ���Ӧ�����1
{
	count (&heibai_2 ,1);
	
	if((xunji_flag [0] == 1) && (heibai_2 .xun_countflag == 1) && (xunji [2] >= 15))
	{
		car.x ++;
		heibai_2 .xun_countflag  = 0;
	}

}
void count_middle3(void)  //x�� ÿ��һ���ߣ���Ӧ�����1
{
	count (&heibai_4 ,3);
	
	if((xunji_flag [0] == 2) && (heibai_4 .xun_countflag == 1) && (xunji [2] >= 15))
	{
		car.x --;
		heibai_4 .xun_countflag  = 0;
	}
}
void migong_gezi(Point now, Point next)  //�Թ�����  ����˫��
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

void migong_line(Point now, Point next)  //�Թ���  ���ڵ���
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

void count_1(void)  //˫��x ÿ��һ����Ӧ�����1
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

void count_2(void)  //˫��y ÿ��һ����Ӧ�����1
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  //��ʱ���жϻش�
{
	if(htim == &htim2 )   //200ms ����Ѱ��pid���ٶȾ���
	{
		xunxian_pid(xun_flag);
//		Kinematic_Analysis(K_X, K_Y, K_ser,K_1,K_2);

	}
	if(htim == &htim3)    //50ms ����tof���� ÿ50ms����DMA�ж�ˢ��
	{
		DMA_receive2 ();  //DMA���գ���ͬ
		DMA_receive3 ();
		DMA_receive4 ();
		tof_handle ();	  //�������ݴ���
	}
	if(htim == &htim4)	  //ת��ʱʱ�����
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
void tof_handle(void)     //tof���ݴ���
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



void xun_pid_init(void)   //Ѳ���ٶ�pid��ʼ��
{
	pid_init(&xun_pid[0],-3, 0, 2, 2000, 2000);
	pid_init(&xun_pid[1],-3, 0, 2.1, 2000, 2000);
	pid_init(&xun_pid[2],-3, 0, 2, 2000, 2000);
	pid_init(&xun_pid[3],-3, 0, 2.1, 2000, 2000);
}

void xunxian_pid(int xx)  //Ѱ��pid����Ѱ���ķ���ֵ���б�������
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //�����жϻص�
{
	if(huart == &huart5 )
	{
		Dael_Coordinate(USART5_RX_BUF);
	}
}

int fputc(int ch ,FILE *f)  //printf�ض���
{
	uint8_t temp[1] = {ch};
	HAL_UART_Transmit(&huart3, temp, 1, 2); //huart1��Ҫ������������޸�
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
void houtui_half(void)  //half ΪѰ���ƶ������ٶȽ���
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
void qianjin_total(void) //totalΪֱ���ƶ�
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
void adjust(void)  		 //tof��У׼��ͨ������tof�ľ���������ת����
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
void houtui(void)  //Ѱ���ƶ����ٶȽϿ�
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
	while( ( (tof[0] + tof[1]) / 2) < 37)    //�ƶ���45cm��
	{
		qianjin_total();
	}
	
	migong_gezi (Point_0,Point_1);  //�ƶ���2ά����
	speed_stop ();
	motor_send ();
	HAL_Delay (1000);

}
void move_first(void)
{
	migong_line (Point_1,Point_2);  //�ƶ���������		
	speed_stop ();
	motor_send ();
	HAL_Delay (200);

	while ((xunji[0] < 15 || xunji [0] > 30) && (xunji[2] < 15 || xunji [2] > 30)) //������λ��΢��
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
	
	while( tof[0] > 39 )  //����������΢��
	{
		houtui_half();
	}
	speed_stop();
	HAL_Delay(1000);	
}
void move_second(void)
{
	time = 0;  //��ת 
	time_flag[0] = 1;
	HAL_TIM_Base_Start_IT(&htim4 );
	while(time_flag[0])
	{	
		speed_set_without_pid(0,0,3);
		motor_send();
	}
	HAL_TIM_Base_Stop_IT (&htim4 );
	speed_stop ();
	
	migong_line(Point_2,Point_3);  //�ּӹ���
	speed_stop ();
	motor_send ();
	HAL_Delay (500);
	
	while ((xunji[0] < 15 || xunji [0] > 30) && (xunji[2] < 15 || xunji [2] > 30)) //�ּӹ���λ��΢��
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
	
	while( tof[0] > 40 )  //�ּӹ��������
	{
		houtui_half();
	}
	speed_stop();
	HAL_Delay(500);
}
void move_third(void)
{
	while ((xunji[1] < 15 || xunji [1] > 30) && (xunji[3] < 15 || xunji [3] > 30))  //�˺�
	{
		qianjin_half ();
	}
	
	migong_line (Point_3,Point_4);  //�ƶ�ת��
	speed_stop ();
	motor_send ();
	
	while ((xunji[0] < 15 || xunji [0] > 30) && (xunji[2] < 15 || xunji [2] > 30))  //ת�����
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

	time_flag[1] = 1;   //��ת
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
		
	migong_line (Point_4,Point_5);   //��Ʒ��
	speed_stop ();
	motor_send ();	
	while ((xunji[0] < 15 || xunji [0] > 30) && (xunji[2] < 15 || xunji [2] > 30))  //��Ʒ��λ��΢��
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
	migong_line (Point_5,Point_6);  //�ƶ���ת�䴦
	speed_stop ();
	HAL_Delay (500);
	while ((xunji[1] < 15 || xunji [1] > 30) && (xunji[3] < 15 || xunji [3] > 30)) //ת�䴦����
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
	
	time_flag[2] = 1;   //��ת
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
		
	while( tof[0] > 40 )  //������΢��
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
//		HAL_UART_Receive_IT(&huart5, USART5_RX_BUF  ,12);	//���ڽ����жϣ���������������
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
//��ʾ��ά����Ϣ
	
//	HAL_Delay (1000);

//	while( k210_state == 1)
//	{
//		LCD_ShowNum (100,240,k210_state,3,16,BLACK );
//		HAL_TIM_Base_Start_IT(&htim5 ); //��ʱ��2��3�жϿ���
//		LCD_ShowNum (0,120,(uint32_t)QR_code_Order[0],3,16,BLACK );
//		LCD_ShowNum (45,120,(uint32_t)QR_code_Order[1],3,16,BLACK );
//		LCD_ShowNum (90,120,(uint32_t)QR_code_Order[2],3,16,BLACK );
//		LCD_ShowNum (0,150,(uint32_t)QR_code_Order[3],3,16,BLACK );
//		LCD_ShowNum (45,150,(uint32_t)QR_code_Order[4],3,16,BLACK );
//		LCD_ShowNum (90,150,(uint32_t)QR_code_Order[5],3,16,BLACK );

//		LCD_ShowNum (0,0,Three[0],5,16,BLACK );
//		LCD_ShowNum (45,0,Three[1],5,16,BLACK );
//		LCD_ShowNum (90,0,Three[2],5,16,BLACK );
////��ʾ��ά���� �������

//		correct(Three);
////�ƶ�Z��
//	}
//	HAL_TIM_Base_Stop_IT(&htim5 ); //��ʱ��2��3�жϿ���
	
	
	
	 while(HAL_GPIO_ReadPin (key2_GPIO_Port ,key1_Pin ) == 0) //����1����
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
	 

	 
//�˶���ʼ	 
	 while(1)
	 {
		LCD_ShowString (0,0,31,10,16,BLACK ,"on");
		 
/*�˶�����ά����
*/		 
		move_ER();
		 
/*��е�����ʶ��2ά��
*/		 
//		CANx_SendExtData(0x00415010,ER_go,8);
//		HAL_Delay(5000);
//		CANx_SendExtData(0x00415010,ER_stop,8);//����ֹͣ 
		 
/*�ȴ���ά�����ݷ���
������ʱ��5�жϣ�state = 0	 
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

		 
/*��е������
*/
//		CANx_SendExtData(0x00415010,ER_back ,8);
//		HAL_Delay(5000);
//		CANx_SendExtData(0x00415010,ER_back_stop ,8);//����ֹͣ 
		
		HAL_Delay(200);
/*�ƶ�������
*/
		move_first();


/*  ���ץȡ		
*/
//		CANx_SendExtData(0x00415010,first_left_up,8);
//		HAL_Delay(8000);
//		HAL_Delay(2000);
//		CANx_SendExtData(0x00415010,first_left_up_done,8);//����ֹͣ 
//		
//		CANx_SendExtData(0x00415010,first_middle_up,8);
//		HAL_Delay(8000);
//		HAL_Delay(2000);
//		CANx_SendExtData(0x00415010,first_middle_up_done,8);//����ֹͣ 
//		
//		
//		CANx_SendExtData(0x00415010,first_right_up,8);
//		HAL_Delay(8000);
//		HAL_Delay(2000);
//		CANx_SendExtData(0x00415010,first_right_up_done,8);//����ֹͣ 
		
		
		HAL_Delay(1000);
		
/*�ƶ��ּӹ���
*/
		move_second();
		
/* �ּӹ�����
*/
//		CANx_SendExtData(0x00415010,second_left,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,second_left_done,8);//����ֹͣ 
//		
//		CANx_SendExtData(0x00415010,second_middle,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,second_middle_done,8);//����ֹͣ 

//		CANx_SendExtData(0x00415010,second_right,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,second_right_done,8);//����ֹͣ 
//		
//		HAL_Delay(1000);
		
/*�ƶ���Ʒ��
*/
		move_third();
		
		
/*  ��Ʒ������		
*/
//		CANx_SendExtData(0x00415010,third_left_up,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,third_left_up_done,8);//����ֹͣ 

//		CANx_SendExtData(0x00415010,third_middle_up,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,third_middle_up_done,8);//����ֹͣ 

//		CANx_SendExtData(0x00415010,third_right_up ,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,third_right_up_done,8);//����ֹͣ 
//	
//	
//		HAL_Delay (1000);
		
/*�ƶ�ԭ����
*/
		move_fourth();
		
		
/*  ���ץȡ
*/
//		CANx_SendExtData(0x00415010,first_left_down,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,first_left_down_done,8);//����ֹͣ 
//		
//		CANx_SendExtData(0x00415010,first_middle_down,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,first_middle_down_done,8);//����ֹͣ 
//		
//		
//		CANx_SendExtData(0x00415010,first_right_down,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,first_right_down_done,8);//����ֹͣ 
//		
//		HAL_Delay (1000);
		
/*�ƶ��ּӹ���
*/
		car.x = 6;
		car.y = 0;
		time_compare [0] = 30;
		move_second();
/* �ּӹ�����		
*/
//		CANx_SendExtData(0x00415010,second_left,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,second_left_done,8);//����ֹͣ 
//		
//		CANx_SendExtData(0x00415010,second_middle,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,second_middle_done,8);//����ֹͣ 

//		CANx_SendExtData(0x00415010,second_right,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,second_right_done,8);//����ֹͣ 
//		
//		HAL_Delay(1000);
		
/*�ƶ���Ʒ��
*/
		move_third();
		
/*  ��Ʒ������		
*/	
//		CANx_SendExtData(0x00415010,third_left_down,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,third_left_down_done,8);//����ֹͣ 

//		CANx_SendExtData(0x00415010,third_middle_down,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,third_middle_down_done,8);//����ֹͣ 

//		CANx_SendExtData(0x00415010,third_right_down ,8);
//		HAL_Delay(8000);
//		CANx_SendExtData(0x00415010,third_right_down_done,8);//����ֹͣ 
//	
//	
//		HAL_Delay (1000);
		
/*�ƶ�����
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

