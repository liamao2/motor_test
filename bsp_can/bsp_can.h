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

#ifndef __BSP_CAN
#define __BSP_CAN

#include "can.h"
#include "MI_motor_drive.h"

//3508
//#define FEEDBACK_ID_BASE      0x201
//#define CAN_CONTROL_ID_BASE   0x200
//#define CAN_CONTROL_ID_EXTEND 0x1ff
#define yaw_moto_id 0x205
#define pich_moto_id 0x206
//#define shexiang_moto_id 0x204
#define CAN_3508_M1_ID  0x201
#define CAN_3508_M2_ID  0x202
#define CAN_3508_M3_ID  0x203
#define CAN_3508_M4_ID  0x204

//6020
//#define FEEDBACK_ID_BASE      0x205
//#define CAN_CONTROL_ID_BASE   0x1ff
//#define CAN_CONTROL_ID_EXTEND 0x2ff  
  #define MOTOR_MAX_NUM         7

extern MI_Motor_s MI_Motor[5];
typedef struct
{
    uint16_t can_id;
    int16_t  set_voltage;
    uint16_t rotor_angle;
    int16_t  rotor_speed;
    int16_t  torque_current;
    uint8_t  temp;
}moto_info_t;
void can_user_init(void);
//void can_user_init2(void);
void set_motor_voltage3508(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void set_motor_voltage6020(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void set_chassis_tx(uint8_t v1,uint8_t v2,uint8_t v3,uint8_t v4,uint8_t v5,uint8_t v6,uint8_t v7,uint8_t v8);
void set_chassis_angle(uint8_t v1,uint8_t v2,uint8_t v3,uint8_t v4,uint8_t v5,uint8_t v6);

typedef struct{
  uint8_t header;
  uint16_t length;
  uint8_t name_1[10];
  uint8_t type_1;
  float data_1;
  uint8_t name_2[10];
  uint8_t type_2;
  uint32_t data_2;
  uint8_t name_3[10];
  uint8_t type_3;
  uint32_t data_3;
  uint8_t name_4[10];
  uint8_t type_4;
  float data_4;
  uint8_t name_5[10];
  uint8_t type_5;
  float data_5;
  uint8_t name_6[10];
  uint8_t type_6;
  float data_6;
  uint8_t name_7[10];
  uint8_t type_7;
  uint32_t data_7;
  uint8_t name_8[10];
  uint8_t type_8;
  uint32_t data_8;
  uint8_t name_9[10];
  uint8_t type_9;
  uint32_t data_9;
  uint8_t name_10[10];
  uint8_t type_10;
  float data_10;
  uint16_t checksum;
} __attribute__((packed)) OutputData_s;
extern OutputData_s OutputData;

#endif
