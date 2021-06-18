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
 



#include "gimbal_task.h"
#include "chassis_task.h"
#include "detect_task.h"

#include "can_device.h"
#include "uart_device.h"

#include "calibrate.h"
#include "keyboard.h"
#include "pid.h"
#include "ramp.h"
#include "sys.h"

#include "cmsis_os.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"


/* 以下是云台任务所使用变量的定义，请勿修改 */
/* gimbal global information */
gimbal_yaw_t gim;
imu_t        imu;

char color_flag=0;//0为红色，1为蓝色
char auto_pid_flag = 1;
char manual_pid_flag = 0;

send_frame auto_tx_data = {0};

uint8_t send_flag=0;
/* gimbal pid parameter */

float yaw_angle_fdb = 0;
float pit_angle_fdb = 0; 

/* read from flash */
int32_t   pit_center_offset = 0;
int32_t   yaw_center_offset = 0;

/* for debug */
uint32_t gimbal_time_last;
int gimbal_time_ms;
/* 执行云台功能任务的函数 */
void gimbal_task(const void* argu)
{
  //云台和拨弹电机参数初始化
  gimbal_init_param();
  
  //从flash中读取云台中点位置
  read_gimbal_offset(&pit_center_offset, &yaw_center_offset);
  
  //云台控制任务循环
  uint32_t gimbal_wake_time = osKernelSysTick();
  while (1)
  {
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET);
    gimbal_time_ms = HAL_GetTick() - gimbal_time_last;
    gimbal_time_last = HAL_GetTick();
    
    //获取云台传感器信息
    get_gimbal_information();
    
    //根据控制命令切换云台状态
    get_gimbal_mode();
  
    switch (gim.ctrl_mode)
    {
      //云台初始化状态
      case GIMBAL_INIT:
      {
        gimbal_init_handle();
      }break;
      
      //云台无输入状态
      case GIMBAL_NO_ACTION:
      {
        gimbal_noaction_handle();
      }break;
      
      //云台闭环控制模式
      case GIMBAL_CLOSE_LOOP_ZGYRO:
      {
				if(manual_pid_flag == 0){
					pid_reset_manual();
					manual_pid_flag = 1;
					auto_pid_flag = 0;
				}
        gimbal_loop_handle();
      }break;
      case GIMBAL_AUTO:
			{
				if(auto_pid_flag == 0){
					pid_reset_auto();
					auto_pid_flag = 1;
					manual_pid_flag = 0;
				}
				gimbal_auto_control();
				
				
//					float frame[2]={yaw_relative_angle,pit_relative_angle};
//					//发送云台角度数据
//					HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_2);
//					write_uart(JETSON_UART,(uint8_t*)"\n",1);
//					write_uart(JETSON_UART,(uint8_t*)frame,sizeof(frame));
//					send_flag=0;
//					

			}break;
      default:
        break;
    }

    //云台控制，如果有模块离线，就切断云台电机输出
    if (gim.ctrl_mode != GIMBAL_RELAX                 //云台释放模式
        && !glb_err.err_list[REMOTE_CTRL_OFFLINE].err_exist //遥控器离线
        && !glb_err.err_list[GIMBAL_YAW_OFFLINE].err_exist  //yaw轴电机离线
        && !glb_err.err_list[GIMBAL_PIT_OFFLINE].err_exist) //pitch轴电机离线
    {
      gimbal_custom_control();
    }
    else
    {
      gim.ctrl_mode = GIMBAL_RELAX;
      pid_trigger.iout = 0;
      send_gimbal_moto_zero_current();
    }
		//发送反馈数据
		//frame_send data_send={0xaa,yaw_relative_angle,color_flag,pit_relative_angle,0,1,0,0xbb};
		//frame_send data_send={0xaa,yaw_angle_ref,0,pit_angle_ref,0,1,0,0xbb};
		auto_tx_data.head = 0xbbbb;
		auto_tx_data.pitchAngleGet = pit_relative_angle;
		auto_tx_data.yawAngleGet = yaw_relative_angle;
		auto_tx_data.shootStatusGet = shoot_status;
		
		write_uart(NUC_UART,(uint8_t*)&auto_tx_data,sizeof(auto_tx_data));
    //云台任务周期控制 5ms
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET);
				
    osDelayUntil(&gimbal_wake_time, GIMBAL_PERIOD);
  }
}

/* 以下为云台任务调用的内部函数，请勿改动 */

/**
  * @brief     get relative position angle to center
  * @param[in] raw_ecd: gimbal motor encoder raw angle
  * @param[in] center_offset: read gimbal_cali_data from chip flash
  * @retval    relative angle, unit is degree.
  */
static int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset)
{
  int16_t tmp = 0;
  if (center_offset >= 4096)
  {
    if (raw_ecd > center_offset - 4096)
      tmp = raw_ecd - center_offset;
    else
      tmp = raw_ecd + 8192 - center_offset;
  }
  else
  {
    if (raw_ecd > center_offset + 4096)
      tmp = raw_ecd - 8192 - center_offset;
    else
      tmp = raw_ecd - center_offset;
  }
  return tmp;
}

static action_mode_e remote_is_action(void)
{
  if ((abs(rc.ch1) >= 10)
   || (abs(rc.ch2) >= 10)
   || (abs(rc.ch3) >= 10)
   || (abs(rc.ch4) >= 10)
   || (abs(rc.mouse.x) >= 5)
   || (abs(rc.mouse.y) >= 5))
  {
    return IS_ACTION;
  }
  else
  {
    return NO_ACTION;
  }
}

/* gimbal relative position param */
float     pit_relative_angle;
float     yaw_relative_angle;  //unit: degree
void get_gimbal_information(void)
{
  //获取 imu 数据
  get_imu_data(&imu);
  
  //云台相对角度获取
  yaw_relative_angle = get_relative_pos(moto_yaw.ecd, yaw_center_offset)/22.75f;
  pit_relative_angle = get_relative_pos(moto_pit.ecd, pit_center_offset)/22.75f;
  
  //云台模式获取
  gim.ac_mode = remote_is_action();
  gim.last_mode = gim.ctrl_mode;
  
  //读取遥控器的控制命令
  pc_kb_hook();
}

uint8_t last_cali_cmd;
extern TaskHandle_t task1_t;
void read_gimbal_offset(int32_t *pit_offset, int32_t *yaw_offset)
{
  /* 配置摩擦轮电调，同时电调消除警报 */
//  start_pwm_output(PWM_IO1);
//  start_pwm_output(PWM_IO2);
  
  if (glb_cali_data.gimbal_cali_data.calied_flag == CALIED_FLAG)
  {
    *pit_offset = glb_cali_data.gimbal_cali_data.pit_offset;
    *yaw_offset = glb_cali_data.gimbal_cali_data.yaw_offset;
    
    //一起正常，唤醒底盘任务
    osThreadResume(task1_t);
  }
  else
  {
    //云台没有校准，进入校准处理
    while (1)
    {
//		int tmp=sprintf(print_buf,"%d %d\n",moto_pit.ecd,moto_yaw.ecd);
//		write_uart(BLUETOOTH_UART,(uint8_t*)print_buf,tmp);
		gimbal_cali_hook();
		send_chassis_moto_zero_current();
		send_gimbal_moto_zero_current();

      osDelay(10);
    }
  }
}

gimbal_back_e gimbal_back_step;

static void gimbal_back_param(void)
{
  gimbal_back_step = BACK_PIT_STEP;
  gim.ecd_offset_angle = yaw_relative_angle;
  
  ramp_init(&pit_ramp, BACK_CENTER_TIME/GIMBAL_PERIOD);
  ramp_init(&yaw_ramp, BACK_CENTER_TIME/GIMBAL_PERIOD);
  
  pid_pit_speed.max_output = 8000;
  
}


float chassis_angle_ac = 5;
void get_gimbal_mode(void)
{
  switch (rc.sw2)
  {
    case (RC_UP):
    {
      if (gim.ac_mode == NO_ACTION)
      {
        if (gim.ctrl_mode == GIMBAL_CLOSE_LOOP_ZGYRO)
        {
          //if (fabs(chassis.vw) <= chassis_angle_ac)
          if (fabs(yaw_relative_angle) <= chassis_angle_ac)
          {
            //begin no action handle
            gim.ctrl_mode = GIMBAL_NO_ACTION;
            
            gim.no_action_flag = 1;
            gim.no_action_time = HAL_GetTick();
          }
        }
      }
      else  //IS_ACTION mode
      {
        if (gim.ctrl_mode == GIMBAL_NO_ACTION||gim.ctrl_mode == GIMBAL_AUTO)
        {
          gim.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;
          gim.no_action_flag = 0;
          
          yaw_angle_ref = 0;
          gim.yaw_offset_angle = imu.angle_z;
        }

      }
      
      if (chassis.mode == CHASSIS_SPIN)
      {
        if (gim.ctrl_mode == GIMBAL_NO_ACTION)
        {
          gim.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;
          gim.no_action_flag = 0;
          
          yaw_angle_ref = 0;
          gim.yaw_offset_angle = imu.angle_z;
        }
      }

      if (gim.last_mode == GIMBAL_RELAX)
      {
        gim.ctrl_mode = GIMBAL_INIT;
        gimbal_back_param();
      }
    }
    break;
		case (RC_MI)://自瞄模式
		{
			//如果遥控无输入
//			if (gim.ac_mode == NO_ACTION)
//			{
				gim.ctrl_mode=GIMBAL_AUTO;
//			}
//			 else  //IS_ACTION mode
//      {
//          gim.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;
//          gim.no_action_flag = 0;
//          
//          yaw_angle_ref = 0;
//          gim.yaw_offset_angle = imu.angle_z;
//      }
		}break;
    default:
    {
      gim.ctrl_mode = GIMBAL_RELAX;
      gimbal_cali_hook();
    }
    break;
  }

}

void gimbal_loop_handle(void)
{
  //static float chassis_angle_tmp;
  //fdb for feedback
  pit_angle_fdb = pit_relative_angle;
	//pit_angle_fdb = gim.pit_offset_angle-imu.angle_y;
  yaw_angle_fdb = imu.angle_z - gim.yaw_offset_angle;
  
  //chassis_angle_tmp = yaw_angle_fdb - yaw_relative_angle;
  if (chassis.mode != CHASSIS_FIXED_ROUTE)
  {
		gimbal_yaw_control();
		gimbal_pitch_control();
//    //限制yaw轴的活动角度
//    if ((yaw_relative_angle >= YAW_ANGLE_MIN) && (yaw_relative_angle <= YAW_ANGLE_MAX))
//    {
//      VAL_LIMIT(yaw_angle_ref, chassis_angle_tmp + YAW_ANGLE_MIN, chassis_angle_tmp + YAW_ANGLE_MAX);
//    }
    //限制pitch轴的活动角度
    if ((pit_relative_angle >= PIT_ANGLE_MIN) && (pit_relative_angle <= PIT_ANGLE_MAX))
    {
      VAL_LIMIT(pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
    }
  }

}

//云台初始化
void gimbal_init_handle(void)
{
  pit_angle_fdb = pit_relative_angle;
  yaw_angle_fdb = yaw_relative_angle;
  
  /* gimbal pitch back center */
  pit_angle_ref = pit_relative_angle * (1 - ramp_calc(&pit_ramp));

  switch (gimbal_back_step)
  {
    case BACK_PIT_STEP:
    {
      /* keep yaw unmove this time */
      yaw_angle_ref = gim.ecd_offset_angle;
      
      if(fabs(pit_angle_fdb) <= 2.0f)
        gimbal_back_step = YAW_BACK_STEP;
    }break;
    
    case YAW_BACK_STEP:
    {
      /* yaw back center after pitch arrive */
      yaw_angle_ref = yaw_relative_angle * ( 1 - ramp_calc(&yaw_ramp));
      
      if (fabs(yaw_angle_fdb) <= 2.0f)
         gimbal_back_step = BACK_IS_OK;
    }break;
    
    case BACK_IS_OK:
    {
      /* yaw arrive and switch gimbal state */
      gim.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;
      
      gim.yaw_offset_angle = imu.angle_z;
			gim.pit_offset_angle = imu.angle_y;
      pit_angle_ref = 0;
      yaw_angle_ref = 0;
      
      pid_pit_speed.max_output = 10000;
    }break;
  }
}


void gimbal_noaction_handle(void)
{
  if (gim.no_action_flag == 1)
  {
    if ((HAL_GetTick() - gim.no_action_time) < 1500)
    {
      gimbal_loop_handle();
    }
    else
    {
      gim.no_action_flag = 2;
      yaw_angle_ref = 0;
    }
  }
  
  if (gim.no_action_flag == 2)
  {
    pit_angle_fdb = pit_relative_angle;
    yaw_angle_fdb = yaw_relative_angle;
  }
}

/**
  * @brief     initialize gimbal pid parameter, such as pitch/yaw/trigger motor, 
  *            imu temperature
  * @attention before gimbal control loop in gimbal_task() function
  */
void gimbal_init_param(void)
{
  //电调 bug
  osDelay(3000);
  
  /* 云台pitch轴电机PID参数初始化 */
  pid_init(&pid_pit, 3000, 600,
                  90.0, 0.9, 750.0); //
  pid_init(&pid_pit_speed, 8000,2000,
                  16, 0.01, 3);

  /* 云台yaw轴电机PID参数初始化 */
  pid_init(&pid_yaw, 4000, 50,
                  30, 1.5, 10.0); //
  pid_init(&pid_yaw_speed, 8000, 3000,
                  100, 0.01, 40.0);
	pid_init(&pid_yaw_auto,180,170,0.08,0.15,0.0);
	
	pid_init(&pid_pit_auto,25,23,0.1,0.05,0.0);
  /* 将云台的初始化状态设置为释放 */
  gim.ctrl_mode = GIMBAL_RELAX;
  
}


void gimbal_init_param_manual(void)
{
  //电调 bug
  osDelay(3000);
  
  /* 云台pitch轴电机PID参数初始化 */
  pid_init(&pid_pit, 3000, 600,
                  90.0, 0.9, 750.0); //
  pid_init(&pid_pit_speed, 8000,2000, 
                  16, 0.01, 3);

  /* 云台yaw轴电机PID参数初始化 */
  pid_init(&pid_yaw, 4000, 50,
                  30, 1.5, 10.0); //
  pid_init(&pid_yaw_speed, 8000, 3000,
                  100, 0.01, 40.0);
	pid_init(&pid_yaw_auto,180,170,0.08,0.15,0.0);
	
	pid_init(&pid_pit_auto,25,23,0.1,0.05,0.0);
  /* 将云台的初始化状态设置为释放 */
  gim.ctrl_mode = GIMBAL_RELAX;
  
}

void pid_reset_manual(void){
	  /* 云台pitch轴电机PID参数重置 */
  pid_reset(&pid_pit,
                  90.0, 0.9, 750.0); //
  pid_reset(&pid_pit_speed,
                  16, 0.01, 3);
	pid_reset(&pid_pit_auto,
									0.1,0.05,0.0);
	
  /* 云台yaw轴电机PID参数重置 */
  pid_reset(&pid_yaw,
                  30, 1.5, 10.0); //
  pid_reset(&pid_yaw_speed,
                  100, 0.01, 40.0);
	pid_reset(&pid_yaw_auto,
									0.08,0.15,0.0);
	

  /* 将云台的初始化状态设置为释放 */
}

void pid_reset_auto(void){
		  /* 云台pitch轴电机PID参数重置 */
  pid_reset(&pid_pit,
                  90.0, 0.9, 750.0); //
  pid_reset(&pid_pit_speed,
                  16, 0.01, 3);
	pid_reset(&pid_pit_auto,
									0.1,0.05,0.0);
	
  /* 云台yaw轴电机PID参数重置 */
  pid_reset(&pid_yaw,
                  30, 1.5, 10.0); //
  pid_reset(&pid_yaw_speed,
                  100, 0.01, 40.0);
	pid_reset(&pid_yaw_auto,
									0.08,0.15,0.0);
}

