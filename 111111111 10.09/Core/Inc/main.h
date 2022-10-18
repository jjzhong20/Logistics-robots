/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "xunji.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern int16_t speed_data[4];  //电机速度存放数组
extern int16_t speed_data1[4];  //电机速度存放数组
extern  int x,y,yw;  //电机xy与旋转分量
extern int key_up;  //按键消抖
extern int i; //电机速度定义与CAN发送数据的循环变量
extern uint8_t message[7][8];  //can发送数据的数组 4电机3舵机
extern int ID[4];  //ID号数组
extern int16_t v_back[4];  //电机速度返回存放数组
extern int16_t bianmaqi_back[4];  //编码器返回数组
extern int16_t motor_msg[4];  

extern int xunji_count[4];  //判断寻迹个数数组
extern int xunji[4];  //寻迹返回数组  8代表都是黑线，0代表都是白线
extern int xunji_flag[4];

extern int servo_x ,servo_y ,servo_z ,zhua;//初始机械臂位置


//串口
int fputc(int ch, FILE *f);  //printf重定义
extern int len[5];
extern uint8_t USART1_RX_BUF[200];  //串口数据
extern uint8_t USART2_RX_BUF[200];  //串口数据 
extern uint8_t USART3_RX_BUF[200];  //串口数据
extern uint8_t USART4_RX_BUF[200];  //串口数据
extern uint8_t USART5_RX_BUF[200];  //串口数据
void DMA_receive2(void);
void DMA_receive3(void);
void DMA_receive4(void);

extern uint16_t  k210_state;  //k210状态
void k210_handle(void);   //k210处理函数


//tof
extern int16_t tof_speed[3];
void tof_handle(void);
extern int tof_count[3],tof_sum[3];

//寻线
void xun_pid_init(void);   //巡线速度pid初始化
void xunxian_pid(int xx);
extern int xun_flag; //巡线方向flag
extern int tof[3];
extern int tof_adj[3];

//姿态
extern int yaw_931;
void adjust(void);

void move_ER(void);
void move_first(void);
void move_second(void);
void move_third(void);
void move_fourth(void);
void move_end(void);

void total(void);
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define key1_Pin GPIO_PIN_2
#define key1_GPIO_Port GPIOC
#define key2_Pin GPIO_PIN_3
#define key2_GPIO_Port GPIOC
#define key3_Pin GPIO_PIN_4
#define key3_GPIO_Port GPIOC
#define key4_Pin GPIO_PIN_5
#define key4_GPIO_Port GPIOC
#define LCD_CS_Pin GPIO_PIN_12
#define LCD_CS_GPIO_Port GPIOB
#define LCD_SCL_Pin GPIO_PIN_13
#define LCD_SCL_GPIO_Port GPIOB
#define LCD_SDO_Pin GPIO_PIN_14
#define LCD_SDO_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_15
#define LCD_SDA_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_6
#define LCD_RS_GPIO_Port GPIOC
#define LCD_BLK_Pin GPIO_PIN_7
#define LCD_BLK_GPIO_Port GPIOC
#define led_Pin GPIO_PIN_8
#define led_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
