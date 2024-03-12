/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
 
#include "bsp_can.h"

#include "string.h"

OutputData_s OutputData;
moto_info_t motor_info[MOTOR_MAX_NUM];
uint16_t can_cnt;
//uint8_t rx_gmibal_data[6];


/**
  * @brief  init can filter, start can, enable can rx interrupt
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
	//CAN过滤器初始化
	void can_user_init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;  // 使能CAN1时钟
  RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;  // 使能CAN2时钟
  CAN_FilterTypeDef  can_filter;
	
  can_filter.FilterActivation = ENABLE;     // enable can filter
  can_filter.FilterBank = 0;                       // filter 0
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // mask mode
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x00;
  can_filter.FilterIdLow  = 0x00;
  can_filter.FilterMaskIdHigh = 0x00;
  can_filter.FilterMaskIdLow  = 0x00;                // set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0
 
  can_filter.SlaveStartFilterBank  = 14;          // only meaningful in dual can mode
   if (HAL_CAN_ConfigFilter(&hcan1,&can_filter ) != HAL_OK)
    {
        Error_Handler();
    }
     can_filter.FilterBank =14;   
		
		if (HAL_CAN_ConfigFilter(&hcan2,&can_filter ) != HAL_OK)
    {
        Error_Handler();
    }
   if( HAL_CAN_Start(&hcan1)!=HAL_OK   )                       // start can1
   {
 	   Error_Handler();
   }
	 if( HAL_CAN_Start(&hcan2)!=HAL_OK   )                       // start can1
   {
 	   Error_Handler();
   }
	if( HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_RX_FIFO1_MSG_PENDING)!=HAL_OK)
	{
		 Error_Handler();
	}
	if( HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_RX_FIFO1_MSG_PENDING)!=HAL_OK)
	{
		 Error_Handler();
	}  
	
}


/**
  * @brief  can rx callback, get motor feedback info
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */

//CAN接受中断
/*void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t             rx_data[8];
 
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can data
  
 // if ((rx_header.StdId >= CAN_3508_M1_ID)
   //&& (rx_header.StdId <  CAN_3508_M1_ID + MOTOR_MAX_NUM))                  // judge the can id
	switch (rx_header.StdId)
	{	
		//case CAN_3508_M1_ID:
		//case CAN_3508_M2_ID:
    //case CAN_3508_M3_ID:
    case yaw_moto_id:
    case pich_moto_id:			
   {
    can_cnt ++;
    uint8_t index = rx_header.StdId - CAN_3508_M1_ID;                  // get motor index by can_id 解析数据
    motor_info[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info[index].temp           =   rx_data[6];
		break;
   }
		case 0x222:
		{
			for(int i=0;i<=5;i++)
			{
				rx_gmibal_data[i]=rx_data[i]*10;
			}
		}
	 default:
        {
            break;
        }
}
}*/
MI_Motor_s MI_Motor[5];//小米电机结构体,0号不用

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
   RxCAN_info_s RxCAN_info;//用于存储小米电机反馈的数据
  uint8_t             rx_data[8];
 
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can data
  
 // if ((rx_header.StdId >= CAN_3508_M1_ID)
	 HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);//接收数据
	 
   //&& (rx_header.StdId <  CAN_3508_M1_ID + MOTOR_MAX_NUM))                  // judge the can id
	
	switch (rx_header.StdId)
	{	
	  case 0x201:
    case 0x202:
    case 0x203:
	
    case 0x205:
   {
    can_cnt ++;
    uint8_t index = rx_header.StdId - CAN_3508_M1_ID;                  // get motor index by can_id 解析数据
    motor_info[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info[index].temp           =   rx_data[6];
		break;
   }
		
	 default:
        {
            break;
        }
    }
	//////////////////////////////////
		 memcpy(&RxCAN_info,&rx_header.ExtId,4);//将扩展标识符的内容解码到缓存区，获取通信类型

    if(RxCAN_info.communication_type == 0){//通信类型0的反馈帧解码
        RxCAN_info_type_0_s RxCAN_info_type_0;
        memcpy(&RxCAN_info_type_0,&rx_header.ExtId,4);//将扩展标识符的内容解码成通信类型0的对应内容
        memcpy(&RxCAN_info_type_0.MCU_id,rx_data,8);//获取MCU标识符
        OutputData.data_3 = RxCAN_info_type_0.motor_id;
    }else if(RxCAN_info.communication_type == 2){//通信类型2的反馈帧解码
        RxCAN_info_type_2_s RxCAN_info_type_2;
        memcpy(&RxCAN_info_type_2,&rx_header.ExtId,4);//将扩展标识符的内容解码成通信类型2的对应内容
        MI_motor_RxDecode(&RxCAN_info_type_2,rx_data);//通信类型2的数据解码

        MI_Motor[RxCAN_info_type_2.motor_id].RxCAN_info = RxCAN_info_type_2;
        MI_Motor[RxCAN_info_type_2.motor_id].motor_mode_state = RxCAN_info_type_2.mode_state;

    }else if(RxCAN_info.communication_type == 17){//通信类型17的反馈帧解码
        RxCAN_info_type_17_s RxCAN_info_type_17;
        memcpy(&RxCAN_info_type_17,&rx_header.ExtId,4);//将扩展标识符的内容解码成通信类型17的对应内容
        memcpy(&RxCAN_info_type_17.index,&rx_data[0],2);//获取查找的参数索引码
        memcpy(&RxCAN_info_type_17.param,&rx_data[4],4);//获取查找的参数信息
        OutputData.data_3 = RxCAN_info_type_17.motor_id;
        OutputData.data_5 = RxCAN_info_type_17.index;
        OutputData.data_6 = RxCAN_info_type_17.param;
    }
    OutputData.data_1 = MI_Motor[1].RxCAN_info.angle;
    OutputData.data_2 = RxCAN_info.communication_type;


  }

/**
  * @brief  send motor control message through can bus
  * @param  id_range to select can control id 0x1ff or 0x2ff
  * @param  motor voltage 1,2,3,4 or 5,6,7
  * @retval None
  */

//CAN线发送，设置电压值
void set_motor_voltage3508(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
 
	tx_header.StdId = (id_range == 0)?(0x200):(0x1ff);//
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;

  tx_data[0] = (v1>>8)&0xff;
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0); 
}
void set_motor_voltage6020(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
 
	tx_header.StdId = (id_range == 0)?(0x1ff):(0x2ff);//
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;

  tx_data[0] = (v1>>8)&0xff;
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0); 
}
