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
extern int16_t speed_data[4];  //����ٶȴ������
extern int16_t speed_data1[4];  //����ٶȴ������
extern  int x,y,yw;  //���xy����ת����
extern int key_up;  //��������
extern int i; //����ٶȶ�����CAN�������ݵ�ѭ������
extern uint8_t message[7][8];  //can�������ݵ����� 4���3���
extern int ID[4];  //ID������
extern int16_t v_back[4];  //����ٶȷ��ش������
extern int16_t bianmaqi_back[4];  //��������������
extern int16_t motor_msg[4];  

extern int xunji_count[4];  //�ж�Ѱ����������
extern int xunji[4];  //Ѱ����������  8�����Ǻ��ߣ�0�����ǰ���
extern int xunji_flag[4];

extern int servo_x ,servo_y ,servo_z ,zhua;//��ʼ��е��λ��


//����
int fputc(int ch, FILE *f);  //printf�ض���
extern int len[5];
extern uint8_t USART1_RX_BUF[200];  //��������
extern uint8_t USART2_RX_BUF[200];  //�������� 
extern uint8_t USART3_RX_BUF[200];  //��������
extern uint8_t USART4_RX_BUF[200];  //��������
extern uint8_t USART5_RX_BUF[200];  //��������
void DMA_receive2(void);
void DMA_receive3(void);
void DMA_receive4(void);

extern uint16_t  k210_state;  //k210״̬
void k210_handle(void);   //k210������


//tof
extern int16_t tof_speed[3];
void tof_handle(void);
extern int tof_count[3],tof_sum[3];

//Ѱ��
void xun_pid_init(void);   //Ѳ���ٶ�pid��ʼ��
void xunxian_pid(int xx);
extern int xun_flag; //Ѳ�߷���flag
extern int tof[3];
extern int tof_adj[3];

//��̬
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
