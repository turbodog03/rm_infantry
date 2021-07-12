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
 



#include "can_device.h"
#include "detect_task.h"
#include "sys.h"
#include "shoot_task.h"

/* 云台电机 */
moto_measure_t moto_pit;
moto_measure_t moto_yaw;
/* 拨弹电机 */
moto_measure_t moto_trigger;
/* 底盘电机 */
moto_measure_t moto_chassis[4];
/* 3508摩擦轮电机 */
moto_measure_t moto_shoot[2];//0左，1右；
/* 外围模块测试电机 */
moto_measure_t moto_test;
//电容功率数据
float PowerData[4];
//发射状态，平时为0，每成功发射一发跳变一次
int shoot_status = 0;
int last_state = 0;
//用于使用摩擦轮速度突变判断是否射出子弹
int shoot_cnt = 0;
int last_cnt = 0;

/**
  * @brief     CAN1 中断回调函数，在程序初始化时注册
  * @param     recv_id: CAN1 接收到的数据 ID
  * @param     data: 接收到的 CAN1 数据指针
  */
void can1_recv_callback(uint32_t recv_id, uint8_t data[])
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
  switch (recv_id)
  {
    case CAN_3508_M1_ID:
    {
      moto_chassis[0].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[0], data) : \
      encoder_data_handle(&moto_chassis[0], data);
      err_detector_hook(CHASSIS_M1_OFFLINE);
    }
    break;
    case CAN_3508_M2_ID:
    {
      moto_chassis[1].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[1], data) : \
      encoder_data_handle(&moto_chassis[1], data);
      err_detector_hook(CHASSIS_M2_OFFLINE);
    }
    break;

    case CAN_3508_M3_ID:
    {
      moto_chassis[2].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[2], data) : \
      encoder_data_handle(&moto_chassis[2], data);
      err_detector_hook(CHASSIS_M3_OFFLINE);
    }
    break;
    case CAN_3508_M4_ID:
    {
      moto_chassis[3].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[3], data) : \
      encoder_data_handle(&moto_chassis[3], data);
      err_detector_hook(CHASSIS_M4_OFFLINE);
    }
    break;
    case CAN_YAW_MOTOR_ID:
    {
      encoder_data_handle(&moto_yaw, data);
      err_detector_hook(GIMBAL_YAW_OFFLINE);
    }
    break;
    case CAN_PIT_MOTOR_ID:
    {
      encoder_data_handle(&moto_pit, data);
      err_detector_hook(GIMBAL_PIT_OFFLINE);
    }
    break;
		case CAN_SUPERCAP_RECV:
			PowerDataResolve(data);
    default:
    {
    }
    break;
  }
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_RESET);
}
  
/**
  * @brief     CAN2 中断回调函数，在程序初始化时注册
  * @param     recv_id: CAN2 接收到的数据 ID
  * @param     data: 接收到的 CAN2 数据指针
  */
void can2_recv_callback(uint32_t recv_id, uint8_t data[])
{
  switch (recv_id)
  {
    //case CAN2 device handle
		case CAN_3508_M1_ID:
    {
      moto_shoot[0].msg_cnt++ <= 50 ? get_moto_offset(&moto_shoot[0], data) : \
      encoder_data_handle(&moto_shoot[0], data);
			//通过转速变化判断是否有子弹射出,同时计算射出数量
			last_state = shoot_status;
			//判断是否产生转速突变
			if(-moto_shoot[0].speed_rpm < SHOT_SUCCESS_FRIC_WHEEL_SPEED && -moto_shoot[0].speed_rpm > SHOT_ABLE_FRIC_WHEEL_SPEED){
				shoot_status = 1;
			}
			else{
				shoot_status = 0;
			}
			if(last_state == 0 && shoot_status == 1){
				shoot_cnt++;
			}

      err_detector_hook(AMMO_BOOSTER1_OFFLINE);
    }
    break;
    case CAN_3508_M2_ID:
    {
      moto_shoot[1].msg_cnt++ <= 50 ? get_moto_offset(&moto_shoot[1], data) : \
      encoder_data_handle(&moto_shoot[1], data);
      err_detector_hook(AMMO_BOOSTER2_OFFLINE);
    }
		break;
		//拨弹电机
    case CAN_3508_M3_ID:
    {
      moto_trigger.msg_cnt++;
      moto_trigger.msg_cnt <= 10 ? get_moto_offset(&moto_trigger, data) : encoder_data_handle(&moto_trigger, data);
      err_detector_hook(TRIGGER_MOTO_OFFLINE);
    }
    break;
    default:
    {
    }
    break;
  }
}

/**
  * @brief     获得电机初始偏差
  * @param     ptr: 电机参数 moto_measure_t 结构体指针
  * @param     data: 接收到的电机 CAN 数据指针
  */
static void get_moto_offset(moto_measure_t *ptr, uint8_t data[])
{
  ptr->ecd        = (uint16_t)(data[0] << 8 | data[1]);
  ptr->offset_ecd = ptr->ecd;
}

/**
  * @brief     计算电机的转速rmp 圈数round_cnt 
  *            总编码器数值total_ecd 总旋转的角度total_angle
  * @param     ptr: 电机参数 moto_measure_t 结构体指针
  * @param     data: 接收到的电机 CAN 数据指针
  */
static void encoder_data_handle(moto_measure_t *ptr, uint8_t data[])
{
  int32_t temp_sum = 0;
  
  ptr->last_ecd      = ptr->ecd;
	//转子机械角度
  ptr->ecd           = (uint16_t)(data[0] << 8 | data[1]);
  //转子转速
  ptr->speed_rpm     = (int16_t)(data[2] << 8 | data[3]);
	//转矩电流没有处理
  if (ptr->ecd - ptr->last_ecd > 4096)
  {
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
  else if (ptr->ecd - ptr->last_ecd < -4096)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  ptr->total_angle = ptr->total_ecd * 360 / 8192;
  
  
  ptr->rate_buf[ptr->buf_cut++] = ptr->ecd_raw_rate;
  if (ptr->buf_cut >= FILTER_BUF)
    ptr->buf_cut = 0;
  for (uint8_t i = 0; i < FILTER_BUF; i++)
  {
    temp_sum += ptr->rate_buf[i];
  }
  ptr->filter_rate = (int32_t)(temp_sum/FILTER_BUF);
}

/**
  * @brief     发送底盘电机电流数据到电调
  */
void send_chassis_moto_current(int16_t current[])
{
  static uint8_t data[8];
  
  data[0] = current[0] >> 8;
  data[1] = current[0];
  data[2] = current[1] >> 8;
  data[3] = current[1];
  data[4] = current[2] >> 8;
  data[5] = current[2];
  data[6] = current[3] >> 8;
  data[7] = current[3];
	
//	data[0] = 0;
//  data[1] = 0;
//  data[2] = 0;
//  data[3] = 0;
//  data[4] = 0;
//  data[5] = 0;
//  data[6] = 0;
//  data[7] = 0;
  
  write_can(CHASSIS_CAN, CAN_CHASSIS_ID, data);
}
void send_chassis_moto_zero_current(void)
{
  static uint8_t data[8];
  
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;
  
  write_can(CHASSIS_CAN, CAN_CHASSIS_ID, data);
}

/**
  * @brief     发送云台电机电流数据到电调
  */
extern int16_t trigger_moto_current;
void send_gimbal_moto_current(int16_t yaw_current, int16_t pit_current)
{
  static uint8_t data[8];
	
  data[0] = yaw_current >> 8;
  data[1] = yaw_current;
  data[2] = pit_current >> 8;
  data[3] = pit_current;
  //data[4] = trigger_current >> 8;
  //data[5] = trigger_current;
  data[6] = 0;
  data[7] = 0;
  
  write_can(GIMBAL_CAN, CAN_GIMBAL_ID, data);
}
void send_gimbal_moto_zero_current(void)
{
  static uint8_t data[8];
  
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;
  
  write_can(GIMBAL_CAN, CAN_GIMBAL_ID, data);
}

void send_shoot_moto_current(int16_t left_current,int16_t right_current, int16_t trigger_current)
{
  static uint8_t data[8];
	
  data[0] = left_current >> 8;
  data[1] = left_current;
  data[2] = right_current >> 8;
  data[3] = right_current;
  data[4] = trigger_current >> 8;
  data[5] = trigger_current;
  data[6] = 0;
  data[7] = 0;
  
  write_can(USER_CAN2, CAN_CHASSIS_ID, data);
}
void sendSuperCap(void)
{
	uint16_t temPower =9000;//功率设定步进0.01W，范围为3000-13000（30W-130W）
	uint8_t sendbuf[8];//发送的数据内容
	sendbuf[0]=temPower >> 8;
	sendbuf[1]=temPower;
	write_can(USER_CAN1, CAN_SUPER_CAP_ID, sendbuf);
	
}
void PowerDataResolve(uint8_t data[])
{
	extern float PowerData[4];
	uint16_t *pPowerData =(uint16_t *) data;
	PowerData[0]=(float)pPowerData[0]/100.f;//输入电压
	PowerData[1]=(float)pPowerData[1]/100.f;//电容电压
	PowerData[2]=(float)pPowerData[2]/100.f;//输入电流
	PowerData[3]=(float)pPowerData[3]/100.f;//设定功率
}

