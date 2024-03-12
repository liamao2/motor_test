/**
  ****************************(C)SWJTU_ROBOTCON****************************
  * @file       cybergear.c/h
  * @brief      С�׵��������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     1-10-2023       ZDYukino        1. done
  *
  @verbatim
  =========================================================================
  =========================================================================
  @endverbatim
  ****************************(C)SWJTU_ROBOTCON****************************
  **/
#include "main.h"
#include "can.h"
//���Ʋ�����ֵ����������
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
#define MAX_P 720
#define MIN_P -720
//����CANID����
#define Master_CAN_ID 0x00                      //����ID
//��������궨��
#define Communication_Type_GetID 0x00           //��ȡ�豸��ID��64λMCUΨһ��ʶ��
#define Communication_Type_MotionControl 0x01 	//�������������Ϳ���ָ��
#define Communication_Type_MotorRequest 0x02	//���������������������״̬
#define Communication_Type_MotorEnable 0x03	    //���ʹ������
#define Communication_Type_MotorStop 0x04	    //���ֹͣ����
#define Communication_Type_SetPosZero 0x06	    //���õ����е��λ
#define Communication_Type_CanID 0x07	        //���ĵ�ǰ���CAN_ID
#define Communication_Type_Control_Mode 0x12
#define Communication_Type_GetSingleParameter 0x11	//��ȡ��������
#define Communication_Type_SetSingleParameter 0x12	//�趨��������
#define Communication_Type_ErrorFeedback 0x15	    //���Ϸ���֡
//������ȡ�궨��
#define Run_mode 0x7005	
#define Iq_Ref   0x7006
#define Spd_Ref  0x700A
#define Limit_Torque 0x700B
#define Cur_Kp 0x7010
#define Cur_Ki 0x7011
#define Cur_Filt_Gain 0x7014
#define Loc_Ref 0x7016
#define Limit_Spd 0x7017
#define Limit_Cur 0x7018

#define Gain_Angle 720/32767.0
#define Bias_Angle 0x8000
#define Gain_Speed 30/32767.0
#define Bias_Speed 0x8000
#define Gain_Torque 12/32767.0
#define Bias_Torque 0x8000
#define Temp_Gain   0.1

#define Motor_Error 0x00
#define Motor_OK 0X01

enum CONTROL_MODE   //����ģʽ����
{
    Motion_mode = 0,//�˿�ģʽ  
    Position_mode=1,  //λ��ģʽ
    Speed_mode=2,     //�ٶ�ģʽ  
    Current_mode=3    //����ģʽ
};
enum ERROR_TAG      //����ش�����
{
    OK                 = 0,//�޹���
    BAT_LOW_ERR        = 1,//Ƿѹ����
    OVER_CURRENT_ERR   = 2,//����
    OVER_TEMP_ERR      = 3,//����
    MAGNETIC_ERR       = 4,//�ű������
    HALL_ERR_ERR       = 5,//HALL�������
    NO_CALIBRATION_ERR = 6//δ�궨
};

typedef struct{           //С�׵���ṹ��
	uint8_t CAN_ID;       //CAN ID
    uint8_t MCU_ID;       //MCUΨһ��ʶ��[��8λ����64λ]
	float Angle;          //�ش��Ƕ�
	float Speed;          //�ش��ٶ�
	float Torque;         //�ش�����
	float Temp;
	
	uint16_t set_current;
	uint16_t set_speed;
	uint16_t set_position;
	
	uint8_t error_code;
	
	float Angle_Bias;
	
}MI_Motor;
extern MI_Motor mi_motor[4];//Ԥ�ȶ����ĸ�С�׵��

 void chack_cybergear(uint8_t ID);
uint32_t Get_Motor_ID(uint32_t CAN_ID_Frame);
 void start_cybergear(MI_Motor *Motor);
 void stop_cybergear(MI_Motor *Motor, uint8_t clear_error);
 void set_mode_cybergear(MI_Motor *Motor, uint8_t Mode);
 void set_current_cybergear(MI_Motor *Motor, float Current);
 void set_zeropos_cybergear(MI_Motor *Motor);
 void set_CANID_cybergear(MI_Motor *Motor, uint8_t CAN_ID);
 void init_cybergear(MI_Motor *Motor, uint8_t Can_Id, uint8_t mode);
 void motor_controlmode(MI_Motor *Motor,float torque, float MechPosition, float speed, float kp, float kd);
void Motor_Data_Handler(MI_Motor *Motor,uint8_t DataFrame[8],uint32_t IDFrame);

