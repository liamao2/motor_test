/**
  ****************************(C) COPYRIGHT 2023 POLARBEAR****************************
  * @file       MI_motor_drive.h
  * @brief      С�׵��CyberGear����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-11-2023     С���           1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 POLARBEAR****************************
  */

#ifndef MI_MOTOR_DRIVE_H
#define MI_MOTOR_DRIVE_H
#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"

/* Public defines -----------------------------------------------------------*/
#define MI_CAN_1 hcan1
#define MI_CAN_2 hcan2

/* Private defines -----------------------------------------------------------*/
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f

#define IQ_REF_MIN -27.0f
#define IQ_REF_MAX 27.0f
#define SPD_REF_MIN -30.0f
#define SPD_REF_MAX 30.0f
#define LIMIT_TORQUE_MIN 0.0f
#define LIMIT_TORQUE_MAX 12.0f
#define CUR_FILT_GAIN_MIN 0.0f
#define CUR_FILT_GAIN_MAX 1.0f
#define LIMIT_SPD_MIN 0.0f
#define LIMIT_SPD_MAX 30.0f
#define LIMIT_CUR_MIN 0.0f
#define LIMIT_CUR_MAX 27.0f

typedef enum
{
    OK                 = 0,//�޹���
    BAT_LOW_ERR        = 1,//Ƿѹ����
    OVER_CURRENT_ERR   = 2,//����
    OVER_TEMP_ERR      = 3,//����
    MAGNETIC_ERR       = 4,//�ű������
    HALL_ERR_ERR       = 5,//HALL�������
    NO_CALIBRATION_ERR = 6//δ�궨
}motor_state_e;//���״̬��������Ϣ��

typedef enum
{
    CONTROL_MODE  = 0, //�˿�ģʽ
    LOCATION_MODE = 1, //λ��ģʽ
    SPEED_MODE    = 2, //�ٶ�ģʽ
    CURRENT_MODE  = 3  //����ģʽ
} motor_run_mode_e;//�������ģʽ

typedef enum
{
    IQ_REF        = 0X7006,//����ģʽIqָ��
    SPD_REF       = 0X700A,//ת��ģʽת��ָ��
    LIMIT_TORQUE  = 0X700B,//ת������
    CUR_KP        = 0X7010,//������ Kp 
    CUR_KI        = 0X7011,//������ Ki 
    CUR_FILT_GAIN = 0X7014,//�����˲�ϵ��filt_gain
    LOC_REF       = 0X7016,//λ��ģʽ�Ƕ�ָ��
    LIMIT_SPD     = 0X7017,//λ��ģʽ�ٶ�����
    LIMIT_CUR     = 0X7018 //�ٶ�λ��ģʽ��������
} motor_index_e;//���������

typedef enum
{
    RESET_MODE = 0,//Resetģʽ[��λ]
    CALI_MODE  = 1,//Cali ģʽ[�궨]
    RUN_MODE   = 2 //Motorģʽ[����]
} motor_mode_state_e;//���ģʽ״̬

typedef struct
{
    uint32_t motor_id : 8; // ֻռ8λ
    uint32_t data : 16;
    uint32_t mode : 5;
    uint32_t res : 3;
} __attribute__((packed)) EXT_ID_t; // 32λ��չID�����ṹ��

typedef struct
{
    uint32_t info : 24;
    uint32_t communication_type : 5;
    uint32_t res : 3;
}__attribute__((packed)) RxCAN_info_s;// �������ݻ���

typedef struct
{
    uint32_t FE : 8;
    uint32_t motor_id : 16;
    uint32_t communication_type : 5;
    uint32_t res : 3;
    uint32_t MCU_id;
}__attribute__((packed)) RxCAN_info_type_0_s;// ͨ������0��������

typedef struct
{
    uint32_t master_can_id : 8;
    uint32_t motor_id : 8;
    uint32_t under_voltage_fault : 1;
    uint32_t over_current_fault : 1;
    uint32_t over_temperature_fault : 1;
    uint32_t magnetic_encoding_fault : 1;
    uint32_t HALL_encoding_failure : 1;
    uint32_t unmarked : 1;
    uint32_t mode_state : 2;
    uint32_t communication_type : 5;
    uint32_t res : 3;
    float angle;//(rad)
    float speed;//(rad/s)
    float torque;//(N*m)
    float temperature;//(��)
} __attribute__((packed)) RxCAN_info_type_2_s; // ͨ������2��������

typedef struct
{
    uint32_t motor_id : 8;
    uint32_t master_can_id : 16;
    uint32_t communication_type : 5;
    uint32_t res : 3;
    uint16_t index;
    float param;
}__attribute__((packed)) RxCAN_info_type_17_s;// ͨ������17��������

typedef struct
{
    CAN_HandleTypeDef *phcan;
    motor_state_e motor_state;
    motor_mode_state_e  motor_mode_state;
    EXT_ID_t EXT_ID;
    uint8_t motor_id;
    uint8_t txdata[8];
    RxCAN_info_type_2_s RxCAN_info;
} MI_Motor_s;

/**********************Functions**************************/

void MI_motor_Init(MI_Motor_s* hmotor, CAN_HandleTypeDef *phcan, uint8_t motor_id);


void MI_motor_GetID(MI_Motor_s* hmotor);
void MI_motor_RxDecode(RxCAN_info_type_2_s* RxCAN_info,uint8_t rx_data[8]);
void MI_motor_Enable(MI_Motor_s *hmotor);
void MI_motor_Stop(MI_Motor_s *hmotor);
void MI_motor_SetMechPositionToZero(MI_Motor_s *hmotor);
void MI_motor_ChangeID(MI_Motor_s* hmotor,uint8_t Now_ID,uint8_t Target_ID);
void MI_motor_ReadParam(MI_Motor_s* hmotor, uint16_t index);


void MI_motor_TorqueControl(MI_Motor_s* hmotor, float torque);
void MI_motor_LocationControl(MI_Motor_s* hmotor, float location, float kp, float kd);
void MI_motor_SpeedControl(MI_Motor_s* hmotor, float speed, float kd);
// void MI_motor_CurrentMode(MI_Motor_s* hmotor, float iq_ref);


extern uint8_t MI_MASTERID;


#endif
 
#ifdef __cplusplus
}
#endif
