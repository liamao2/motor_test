

#include "dma.h"
#include "usart.h"
#include "receivedata.h"

extern uint8_t dbus_buf[18];
//float target_position;
//前后平移
float target_speed0;
float target_speed1;
float target_speed2;
float target_speed3;
//左右转弯
float target_speed0r;
float target_speed1r;
float target_speed2r;
float target_speed3r;
//左右平移
float target_speed0p;
float target_speed1p;
float target_speed2p;
float target_speed3p;
Rc_ctrl ctl;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	ctl.rc.ch0 = (dbus_buf[0] | (dbus_buf[1] << 8))&0x07ff;
	ctl.rc.ch1 =((dbus_buf[1] >>3) | (dbus_buf[2]<<5))&0x07ff;
	ctl.rc.ch2 =((dbus_buf[2] >>6) | (dbus_buf[3]<<2) |(dbus_buf[4]<<10))&0x07ff;
	ctl.rc.ch3 =((dbus_buf[4] >>1) | (dbus_buf[5] <<7))&0x07ff;
	ctl.rc.s1 =((dbus_buf[5] >>4) & 0x0003);
	ctl.rc.s2 = ((dbus_buf[5] >>4) & 0x000c)>>2;
	//target_position=((float)ctl.rc.ch3-1024)*5.3f+4500;
	//target_speed0r=((float)ctl.rc.ch0-1024)*2.12f;
	//target_speed1r=((float)ctl.rc.ch0-1024)*2.12f;
	//target_speed2r=((float)ctl.rc.ch0-1024)*2.12f;
	//target_speed3r=((float)ctl.rc.ch0-1024)*2.12f;
	  target_speed2p=((float)ctl.rc.ch1-1024)*5.12f;
	  target_speed0p=((float)ctl.rc.ch0-1024)*1.12f;
		
	
 target_speed0=-((float)ctl.rc.ch3-1024)*2.12f+((float)ctl.rc.ch2-1024)*2.12f+((float)ctl.rc.ch0-1024)*2.12f;
	 target_speed1=-((float)ctl.rc.ch3-1024)*2.12f-((float)ctl.rc.ch2-1024)*2.12f+((float)ctl.rc.ch0-1024)*2.12f;
	 target_speed2=((float)ctl.rc.ch3-1024)*2.12f-((float)ctl.rc.ch2-1024)*2.12f+((float)ctl.rc.ch0-1024)*2.12f;
	 target_speed3=((float)ctl.rc.ch3-1024)*2.12f+((float)ctl.rc.ch2-1024)*2.12f+((float)ctl.rc.ch0-1024)*2.12f;
	
	//target_speed0=-((float)ctl.rc.ch3-1024)*2.12f+((float)ctl.rc.ch2-1024)*2.12f;
	//target_speed1=-((float)ctl.rc.ch3-1024)*2.12f-((float)ctl.rc.ch2-1024)*2.12f;
	//target_speed2=((float)ctl.rc.ch3-1024)*2.12f-((float)ctl.rc.ch2-1024)*2.12f;
	//target_speed3=((float)ctl.rc.ch3-1024)*2.12f+((float)ctl.rc.ch2-1024)*2.12f;
	
	//target_speed0p=((float)ctl.rc.ch0-1024)*2.12f;
	//target_speed1p=((float)ctl.rc.ch0-1024)*2.12f;
	//target_speed2p=((float)ctl.rc.ch0-1024)*2.12f;
	//target_speed3p=((float)ctl.rc.ch0-1024)*2.12f;
}

