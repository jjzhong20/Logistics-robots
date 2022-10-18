/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */
extern uint8_t rx_data[8];  //返回的数值存放位置
extern int flag_id;  //甄别判断返回值

void rx_divide(int rx_id);  //返回值甄别
void CAN_filter_Init(void);  //过滤器初始化
uint8_t CANx_SendExtData(uint32_t ID,uint8_t *pData,uint16_t Len);  //发送函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);  //中断回调

void motor_send(void);  //单独电机发送函数


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

