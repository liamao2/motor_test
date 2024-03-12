
#include "chassistask.h"
#include "main.h"
#include "pid1.h"
#include "bsp_can.h"
#include "struct_typedef.h"
#include "dma.h"
#include "usart.h"
#include "receivedata.h"
#include "gpio.h"
//#include "cybergear.h"
#include "cmsis_os.h"
#include "MI_motor_drive.h"

/*extern moto_info_t motor_info[MOTOR_MAX_NUM];
int16_t led_cnt;
pid_struct_t motor_pid[7];
pid_struct_t position_pid_outer[7];
//前进后退
extern float target_speed0;
extern float target_speed1;
extern float target_speed2;
extern float target_speed3;
//左右平移
extern float target_speed0p;
extern float target_speed2p;
uint8_t v1,v2,v3,v4,v5,v6;
//float target_position=3510;
float target_position=3510;
uint8_t dbus_buf[18];
extern Rc_ctrl ctl;
extern uint8_t rx_gmibal_data[6];
extern fp32 INS_angle[3];
int a=0;
int b;
//float accel[2];
int fuhao;
*/
extern moto_info_t motor_info[MOTOR_MAX_NUM];

pid_struct_t moca_left_pid;
pid_struct_t moca_right_pid;
pid_struct_t bodan_2006_pid;
pid_struct_t yaw_moto_pid;
pid_struct_t pitch_position_pid;
pid_struct_t pitch_current_pid;

pid_struct_t yaw_motor_position_pid;

extern Rc_ctrl ctl;

uint8_t dbus_buf[18];

float moca_left_speed;
float moca_right_speed;
float bodan_2006_speed;
float yaw_motor_speed;
float yaw_motor_position=3000;
float pitch_targetcurrent;
float pitch_targetposition=0;
float pitch_targetspeed;
extern fp32 INS_angle[3];
float b;
float pitch_angle;
//extern MI_Motor_s MI_Motor[5];


void chassis_task(void const * argument)
{
	
  can_user_init(); //CAN过滤器初始化
	pid_init(&moca_left_pid,5,0,5,1000,1000);
	pid_init(&moca_right_pid,5,0,5,1000,1000);
	pid_init(&bodan_2006_pid,5,0,5,3000,3000);
	pid_init(&yaw_moto_pid,5,0,5,8000,8000);    //yaw 6020
	pid_init(&yaw_motor_position_pid,10,0,1,4000,4000);
	pid_init(&pitch_current_pid,3,0,45,3000,3000);
	pid_init(&pitch_position_pid,5,0,40,10000,10000);
	


   

	while(1)
	{
		
		HAL_UART_Receive_DMA(&huart3,dbus_buf,18);
		
		moca_left_speed=-1000;
		moca_right_speed=1000;
		bodan_2006_speed=1000;
		
		
			//摩擦轮+拨弹轮速度处理
		motor_info[0].set_voltage=pid_calc(&moca_left_pid,moca_left_speed,motor_info[0].rotor_speed);
		motor_info[1].set_voltage=pid_calc(&moca_right_pid,moca_right_speed,motor_info[1].rotor_speed);
		motor_info[2].set_voltage=pid_calc(&bodan_2006_pid,bodan_2006_speed,motor_info[2].rotor_speed);
	
		//yaw电机期望位置+过零处理
		if(ctl.rc.ch0>1024)
		{
			yaw_motor_position+=1;
			if(yaw_motor_position>8190)
			{
				yaw_motor_position-=8180;
			}
		}
		else if(ctl.rc.ch0<1024)
		{
			yaw_motor_position-=1;
			if(yaw_motor_position<5)
			{
				yaw_motor_position+=8180;
			}
		}
		//////////////////////////////
		//pitch电机期望位置+过零处理
		 pitch_angle=INS_angle[2]*1000;
		if(ctl.rc.ch1>1024)
		{
			pitch_targetposition+=1;
			//if(pitch_targetposition>6282)
			//{
			//	pitch_targetposition-=6282;
			//}
		}
		else if(ctl.rc.ch1<1024)
		{
			pitch_targetposition-=1;
			//if(pitch_targetposition<10)
			//{
			//	pitch_targetposition+=6282;
			//}
		}
		//yaw过0处理
		if(yaw_motor_position-motor_info[4].rotor_angle>4095)
			{
				motor_info[4].rotor_angle+=8160;
			}
			else if(yaw_motor_position-motor_info[4].rotor_angle<-4095)
			{
				motor_info[4].rotor_angle-=8160;
			}
			////yaw pid
		yaw_motor_speed=pid_calc(&yaw_motor_position_pid,yaw_motor_position,motor_info[4].rotor_angle);
		motor_info[4].set_voltage=pid_calc(&yaw_moto_pid,yaw_motor_speed,motor_info[4].rotor_speed);
			//pitch pid
		
		//条件控制，can发送信息
		
		if(ctl.rc.s1==2)
		{
		
			set_motor_voltage3508(0,
			                      motor_info[0].set_voltage,
			                      motor_info[1].set_voltage,
			                      motor_info[2].set_voltage,
			                      0);
		set_motor_voltage6020(0,
		                         motor_info[4].set_voltage,
														 0,
														 0,
														 0);
		}
		else if(ctl.rc.s1!=2)
		
		{
			set_motor_voltage3508(1,0,0,0,0);
			set_motor_voltage3508(0,0,0,0,0);
			set_motor_voltage6020(1,0,0,0,0);
			set_motor_voltage6020(0,0,0,0,0);
	
		}
		
		
		 osDelay(1);
  }
			
	
 }


