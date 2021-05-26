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
 



/************************************ 用户对云台自定义控制 *************************************/
#include "chassis_task.h"
#include "can_device.h"
#include "uart_device.h"
#include "pid.h"
#include "sys.h"

extern char mmp_buf[];

/* 云台电机期望角度(degree) */
float yaw_angle_ref;
float pit_angle_ref;
/* 云台电机期望角速度(degree/s) */
float yaw_speed_ref;
float pit_speed_ref;
/* 云台电机电流 */
int16_t yaw_moto_current;
int16_t pit_moto_current;


/* 云台控制信号获取，将输入信号积分成云台的角度degree */
void gimbal_yaw_control(void)
{
 //yaw轴的角度累加，单位degree
  yaw_angle_ref += -rc.ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW
                   -rc.mouse.x * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW;
	
}
void gimbal_pitch_control(void)
{
  //pitch轴的角度累加，单位degree
  pit_angle_ref += rc.ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT
                 - rc.mouse.y * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;
}


/* 云台电机转速的闭环控制 */
void gimbal_custom_control(void)
{
	static int16_t last_current = 0; 
//	float a[5]={pid_yaw.p,pid_yaw.i,pid_yaw.d,yaw_angle_ref,pit_angle_ref};
//	int16_t b[2]={yaw_moto_current,pit_moto_current};
//	write_uart(BLUETOOTH_UART,(uint8_t*)b,4*sizeof(uint8_t));
//	write_uart(BLUETOOTH_UART,(uint8_t*)"\n",1);
  //闭环计算云台电机的输出电流
  /* yaw轴预期速度计算，单位degree/s */
  yaw_speed_ref  = pid_calc(&pid_yaw, yaw_angle_fdb, yaw_angle_ref);    //degree
  /* yaw轴电机电流计算 */
  yaw_moto_current = pid_calc(&pid_yaw_speed, imu.gyro_z, yaw_speed_ref+(pid_yaw.err[NOW] - pid_yaw.err[LAST])); //degree/s
  /* pitch轴俯仰角度限制 */
  VAL_LIMIT(pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
  /* pitch轴预期速度计算，单位degree/s */
  pit_speed_ref    = pid_calc(&pid_pit, pit_angle_fdb, pit_angle_ref);    //degree
  /* pitch轴电机电压计算 */

	pit_moto_current = pid_calc(&pid_pit_speed, imu.gyro_y, pit_speed_ref); //degree/s
	//滤波
	pit_moto_current = 0.5*last_current + 0.5*pit_moto_current;
	last_current = pit_moto_current;


  //发送电流到云台电机电调
  send_gimbal_moto_current(yaw_moto_current, pit_moto_current);
	

}
//云台自瞄处理
void gimbal_auto_control()
{
	float rate=0.3,a,b;
	static float last_p=0.0f,last_y=0.0f;
	if(rc.kb.bit.G)
	{
		if(rc.kb.bit.SHIFT)
			color_flag=1;
		else
			color_flag=0;
	}
//	HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_2);
//	a	=	data_recv.yaw_angle		*	0.2+(1-0.2)*last_y;
//	b	=	data_recv.pitch_angle	*	rate+(1-rate)*last_p;
//	pit_angle_ref=pid_calc(&pid_pit_auto,data_recv.pitch_angle,0);
//	yaw_angle_ref=pid_calc(&pid_yaw_auto,data_recv.yaw_angle,0);
	pit_angle_ref=data_recv.pitchAngleSet;
	yaw_angle_ref=data_recv.yawAngleSet;
	gimbal_yaw_control();
	//gimbal_pitch_control();
	//pit_angle_ref=pit_relative_angle+b;

	 //限制pit轴的活动角度
	if ((pit_angle_ref >= PIT_ANGLE_MAX) && (pit_angle_ref <= PIT_ANGLE_MIN))
	{
		VAL_LIMIT(yaw_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
	}
	if ((yaw_angle_ref >= 170) && (yaw_angle_ref <= -170))
	{
		VAL_LIMIT(yaw_angle_ref, -170, 170);
	}
	last_p=data_recv.pitchAngleSet;
	last_y=data_recv.yawAngleSet;
	//计算pitch轴相对角度差
	pit_angle_fdb = pit_relative_angle;
	//计算yaw轴相对角度差
	yaw_angle_fdb = yaw_relative_angle;	
}
