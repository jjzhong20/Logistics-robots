/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "servo.h"
#include "xunji.h"
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN1_2();

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN_filter_Init(void)  //过滤器初始化配置
{

	CAN_FilterTypeDef CAN_Filter;
	
	CAN_Filter.FilterBank =0;
	CAN_Filter.FilterIdHigh =0X0000;
	CAN_Filter.FilterIdLow =0X0000;
	CAN_Filter.FilterMaskIdHigh =0X0000;
	CAN_Filter.FilterMaskIdLow =0X0000;
	CAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN_Filter.FilterActivation = ENABLE;
	CAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK ;
	CAN_Filter.FilterScale =CAN_FILTERSCALE_32BIT ;
	CAN_Filter.SlaveStartFilterBank =14;
	if(HAL_CAN_ConfigFilter(&hcan,&CAN_Filter) != HAL_OK )
	{
		Error_Handler();
	}
	if(HAL_CAN_Start(&hcan) != HAL_OK )  //开启can
	{
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)  //CAN接收中断
	{
		/* Notification Error */
		Error_Handler();
	}
}

/**
 * @brief  发送扩展ID的数据帧
 * @param  hcan     CAN的句柄
 * @param  ID       数据帧ID
 * @param  pData    数组指针
 * @param  Len      数据长度0~8
 */
uint8_t CANx_SendExtData(uint32_t ID,uint8_t *pData,uint16_t Len)
{
	static CAN_TxHeaderTypeDef   Tx_Header;

	uint32_t txmailbox;
	
	Tx_Header.RTR=CAN_RTR_DATA;
	Tx_Header.DLC=Len;
	Tx_Header.ExtId=ID;
	Tx_Header.IDE=CAN_ID_EXT;
	Tx_Header .TransmitGlobalTime =DISABLE ;
		
	HAL_CAN_AddTxMessage(&hcan, &Tx_Header, pData , &txmailbox);

	return 1;	
	
} 

void motor_send(void)  //单独电机发送函数
{
	static CAN_TxHeaderTypeDef   Tx_Header;
    uint32_t            TxMailbox;
	
    Tx_Header.IDE   = CAN_ID_EXT;
    Tx_Header.RTR   = CAN_RTR_DATA;
    Tx_Header.DLC   = 8;	
	
    Tx_Header.ExtId = 0x00210010;
	HAL_CAN_AddTxMessage(&hcan, &Tx_Header, message[0], &TxMailbox);
	HAL_Delay(0);
	
    Tx_Header.ExtId = 0x00220010;
	HAL_CAN_AddTxMessage(&hcan, &Tx_Header, message[1], &TxMailbox);
	HAL_Delay(0);
	
    Tx_Header.ExtId = 0x00230010;
	HAL_CAN_AddTxMessage(&hcan, &Tx_Header, message[2], &TxMailbox);	
	HAL_Delay(0);
	
	Tx_Header.ExtId = 0x00240010;
	HAL_CAN_AddTxMessage(&hcan, &Tx_Header, message[3], &TxMailbox);
	HAL_Delay(0);
}


/*找到空的发送邮箱，把数据发送出去*/
//	if(HAL_CAN_AddTxMessage(&hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
//	{
//		if(HAL_CAN_AddTxMessage(&hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
//		{
//			HAL_CAN_AddTxMessage(&hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX2);
//		}
//	}	


/**
 * @brief  CAN FIFO0的中断回调函数，在里面完成数据的接收
 * @param  hcan     CAN的句柄
 */
 
int flag_id = 0;
uint8_t rx_data[8];

 
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	
	CAN_RxHeaderTypeDef Rx_Header;
		
	HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING );
	
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0 ,&Rx_Header , rx_data);
	
//	rx_divide(Rx_Header .ExtId);
	
	if (Rx_Header.ExtId == 0x0ff00021) 
	{
		v_back[0] = ((rx_data[1] << 8) | rx_data[0]) - 32000;
		bianmaqi_back [0] =((rx_data[5] << 8) | rx_data[4]);
	}
	if (Rx_Header.ExtId == 0x0ff00022) 
	{
		v_back[1] = ((rx_data[1] << 8) | rx_data[0]) - 32000;
		bianmaqi_back [1] =((rx_data[5] << 8) | rx_data[4]);
	}
	if (Rx_Header.ExtId == 0x0ff00023) 
	{
		v_back[2] = ((rx_data[1] << 8) | rx_data[0]) - 32000;
		bianmaqi_back [2] =((rx_data[5] << 8) | rx_data[4]);
	}
	if (Rx_Header.ExtId == 0x0ff00024) 
	{
		v_back[3] = ((rx_data[1] << 8) | rx_data[0]) - 32000;
		bianmaqi_back [3] =((rx_data[5] << 8) | rx_data[4]);
	}
	//寻迹返回
	if (Rx_Header.ExtId == 0x0ff00031) 
	{
		xunji_count[0] = rx_data_count(rx_data[0] );
		xunji[0]=rx_data [0];
	}
	if (Rx_Header.ExtId == 0x0ff00032) 
	{
		xunji_count[1] = rx_data_count (rx_data[0] );
		xunji [1] =rx_data [0];
	}
	if (Rx_Header.ExtId == 0x0ff00033) 
	{
		xunji_count[2] = rx_data_count (rx_data[0] );
		xunji[2] = rx_data [0];
	}
	if (Rx_Header.ExtId == 0x0ff00034) 
	{
		xunji_count[3] = rx_data_count (rx_data[0] );
		xunji [3] = rx_data [0];
	}
	
}

void rx_divide(int rx_id)  //返回值甄别
{	
	switch (rx_id)
	{
		case 0X0FF00021 :
			flag_id =1;
			break ;
		case 0X0FF00022 :
			flag_id =2;
			break ;
		case 0X0FF00023 :
			flag_id =3;
			break ;
		case 0X0FF00024 :
			flag_id =4;
			break ;
		
//		case 0x0ff00031 :  //寻迹返回布尔量（0，1）
//			flag_id =5;
//			break ;
//		case 0x0ff00032 :
//			flag_id =6;
//			break ;
//		case 0x0ff00033 :
//			flag_id =7;
//			break ;
//		case 0x0ff00034 :
//			flag_id =8;
//			break ;
		
		case 0x0ff01031 :  //寻迹返回AD值
			flag_id =5;
			break ;
		case 0x0ff01032 :
			flag_id =6;
			break ;
		case 0x0ff01033 :
			flag_id =7;
			break ;
		case 0x0ff01034 :
			flag_id =8;
			break ;
		case 0x00100031 :
			flag_id =9;
			break ;
	}
	
	
}





/* USER CODE END 1 */
