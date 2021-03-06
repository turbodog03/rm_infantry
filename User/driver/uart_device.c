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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

/**
  *********************** (C) COPYRIGHT 2018 DJI **********************
  * @update
  * @history
  * Version     Date              Author           Modification
  * V1.0.0      January-15-2018   ric.luo
  * @verbatim
  *********************** (C) COPYRIGHT 2018 DJI **********************
  */ 
 


#include "uart_device.h"
#include "detect_task.h"
#include "calibrate.h"
#include "sys.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "pid.h"

extern pid_t pid_yaw_speed,pid_pit_speed,pid_pit,pid_yaw,pid_chassis_angle;
extern global_cali_t glb_cali_data;
extern gimbal_yaw_t gim;
extern float yaw_angle_ref,pit_angle_ref;
extern uint8_t send_flag;
char print_buf[50];

//ext_power_heat_data_t heat_data;

/* 解析后的遥控器数据 */
rc_type_t rc;
/* 接收到的遥控器原始数据 */
uint8_t   dbus_recv[DBUS_FRAME_SIZE];
uint8_t		nuc_recv[NUC_FRAME_SIZE];
uint8_t		referee_recv[REFEREE_FRAME_SIZE];
uint8_t		bluetooth_recv[BLUETOOTH_FRAME_SIZE];
//float pc_angle[2]={0.0,0.0};
float amplitude=50.0f;
/**
  * @brief     遥控器中断回调函数，在设置 UART 接收时注册
  */
void dbus_uart_callback(void)
{
  remote_data_handle(&rc, dbus_recv);
  err_detector_hook(REMOTE_CTRL_OFFLINE);
}
/**
  * @brief     自瞄中断回调函数，在设置 UART 接收时注册
  */
extern uint16_t crc16_check(uint8_t* data, uint32_t length);
recv_frame data_recv;
void nuc_uart_callback(void)
{
	//HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_1);
//	send_flag=1;
	if(gim.ctrl_mode==GIMBAL_AUTO)
	{
		if(((uint16_t*)nuc_recv)[0]==0xaaaa)
		{
			memcpy((void*)&data_recv,nuc_recv,sizeof(data_recv));
		}
//		memcpy(data_recv,jetson_recv,sizeof(pc_angle));
//		yaw_angle_ref=pc_angle[0];
//		pit_angle_ref=pc_angle[1];
	}
}
void referee_uart_callback(void)
{
//	if(referee_recv[5]==0x02&&referee_recv[6]==0x02)
//	{
//		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_3);
//		memcpy((void*)&heat_data,referee_recv+7,14);
//	}
//	if(RESET != __HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))   //判断是否是空闲中断
//	{
//			__HAL_UART_CLEAR_IDLEFLAG(&huart6);                     //清楚空闲中断标志（否则会一直不断进入中断）
//			HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_3);
//	}
}
void bluetooth_uart_callback(void)
{
	float value=0.0f;
	char type=0;
	
	bluetooth_recv[BLUETOOTH_FRAME_SIZE-1]='\n';
	sscanf((const char *)bluetooth_recv,"%c%f",&type,&value);
	switch(type)
	{
//		case 'A':
//			amplitude=value;
		case 'p':
			pid_pit.p=value;
			break;
		case 'i':
			pid_pit.i=value;
			break;
		case 'd':
			pid_pit.d=value;
			break;
		case 'P':
			pid_pit_speed.p=value;
			break;
		case 'I':
			pid_pit_speed.i=value;
			break;
		case 'D':
			pid_pit_speed.d=value;
			break;
//大写字母调内环，小写字母调外环
	}
	//memcpy(nmb,bluetooth_recv,sizeof(nmb));
	//if (glb_cali_data.gimbal_cali_data.calied_flag != CALIED_FLAG)
	//glb_cali_data.gimbal_cali_data.cali_cmd = 1;
//	sprintf(mmp_buf,"%f\n",pid_yaw_speed.p);
//	write_uart(BLUETOOTH_UART,(uint8_t *)mmp_buf,20);
//	sprintf(mmp_buf,"%f\n",pid_yaw_speed.i);
//	write_uart(BLUETOOTH_UART,(uint8_t *)mmp_buf,20);
//	sprintf(mmp_buf,"%f\n",pid_yaw_speed.d);
//	write_uart(BLUETOOTH_UART,(uint8_t *)mmp_buf,20);

//	if(nmb[0]>0)
//	{
//		pid_yaw.p=nmb[0];
//		pid_yaw.i=nmb[1];
//		pid_yaw.d=nmb[2];
//	}
}
/**
  * @brief     解析遥控器数据
  * @param     rc: 解析后的遥控器数据结构体指针
  * @param     buff: 串口接收到的遥控器原始数据指针
  */
static void remote_data_handle(rc_type_t *rc, uint8_t *buff)
{
  /* 下面是正常遥控器数据的处理 */
  rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
  rc->ch1 -= 1024;
  rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
  rc->ch2 -= 1024;
  rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
  rc->ch3 -= 1024;
  rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
  rc->ch4 -= 1024;

  /* 防止遥控器零点有偏差 */
  if(rc->ch1 <= 5 && rc->ch1 >= -5)
    rc->ch1 = 0;
  if(rc->ch2 <= 5 && rc->ch2 >= -5)
    rc->ch2 = 0;
  if(rc->ch3 <= 5 && rc->ch3 >= -5)
    rc->ch3 = 0;
  if(rc->ch4 <= 5 && rc->ch4 >= -5)
    rc->ch4 = 0;

  /* 拨杆值获取 */
  rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
  rc->sw2 = (buff[5] >> 4) & 0x0003;
  
  /* 遥控器异常值处理，函数直接返回 */
  if ((abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch3) > 660) || \
      (abs(rc->ch4) > 660))
  {
    memset(rc, 0, sizeof(rc_type_t));
    return ;
  }

  /* 鼠标移动速度获取 */
  rc->mouse.x = buff[6] | (buff[7] << 8);
  rc->mouse.y = buff[8] | (buff[9] << 8);
  
  /* 鼠标左右按键键值获取 */
  rc->mouse.l = buff[12];
  rc->mouse.r = buff[13];

  /* 键盘按键键值获取 */
  rc->kb.key_code = buff[14] | buff[15] << 8;
  
  /* 遥控器左侧上方拨轮数据获取，和遥控器版本有关，有的无法回传此项数据 */
  rc->wheel = buff[16] | buff[17] << 8;
  rc->wheel -= 1024;
}
