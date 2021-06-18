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
 



#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "stm32f4xx_hal.h"
#include "rm_hal_lib.h"

/* 云台控制周期 (ms) */
#define GIMBAL_PERIOD 5
/* 云台回中初始化时间 (ms) */
#define BACK_CENTER_TIME 1000

/**
  * @brief     云台控制任务函数
  */
void gimbal_task(const void* argu);

/**
  * @brief     云台控制模式枚举
  */
typedef enum
{
  GIMBAL_INIT = 0,         //云台初始化
  GIMBAL_RELAX,            //云台断电
  GIMBAL_CLOSE_LOOP_ZGYRO, //云台跟随单轴角度
  GIMBAL_NO_ACTION,        //无手动信号输入状态
	GIMBAL_AUTO							 //云台自瞄模式
} gimbal_mode_e;

/**
  * @brief     云台控制信号输入状态枚举
  */
typedef enum
{
  NO_ACTION = 0,           //无控制信号输入
  IS_ACTION,               //有控制信号输入
} action_mode_e;

/**
  * @brief     云台回中状态枚举
  */
typedef enum
{
  BACK_PIT_STEP,           //云台 pitch 轴回中
  YAW_BACK_STEP,           //云台 yaw 轴回中
  BACK_IS_OK,              //云台回中完毕
} gimbal_back_e;

/**
  * @brief     云台控制数据结构体
  */
typedef struct
{
  gimbal_mode_e ctrl_mode; //云台当前控制模式
  gimbal_mode_e last_mode; //云台上次控制模式

  action_mode_e ac_mode;   //云台控制信号输入模式
  
  uint8_t  no_action_flag; //无控制信号标志
  uint32_t no_action_time; //无控制信号时间

  float ecd_offset_angle;  //云台初始编码器值
  float yaw_offset_angle;  //云台初始 yaw 轴角度
	float pit_offset_angle;  //云台初始 yaw 轴角度
} gimbal_yaw_t;

//自瞄发送结构体
typedef __packed struct
{
	  uint16_t head;					//帧头
    float pitchAngleGet;    //pitch??????
    float yawAngleGet;      //yaw??????
    uint8_t rotateDricetion;   //旋转方向
    float timeBais;         //预测时间偏置
    float compensateBais;   //弹道补偿偏置
		int shootStatusGet;
}send_frame;


//自瞄接收结构体
typedef __packed struct 
{
    uint16_t head;  				 //帧头
    float pitchAngleSet;    //pitch轴角度设定值
    float yawAngleSet;      //yaw轴角度设定值
    float targetAngle;      //目标装甲板角度
    uint8_t shootCommand;   //发射指令
}recv_frame;

extern gimbal_yaw_t gim;
extern imu_t        imu;

/* 云台 PID 控制相关数据 */
extern float yaw_angle_ref;
extern float pit_angle_ref; 
extern float yaw_angle_fdb;
extern float pit_angle_fdb;
extern float yaw_speed_ref;
extern float pit_speed_ref;
/* 云台相对角度 */
extern float yaw_relative_angle;
extern float pit_relative_angle;
extern char color_flag;


/**
  * @brief     读取云台中点的编码器偏移数据
  * @param     pit_offset: pitch 轴编码器数据指针
  * @param     yaw_offset: yaw 轴编码器数据指针
  */
static void read_gimbal_offset(int32_t *pit_offset, int32_t *yaw_offset);
/**
  * @brief     获取云台传感器信息
  */
static void get_gimbal_information(void);
/**
  * @brief     获取云台控制模式
  */
static void get_gimbal_mode(void);
/**
  * @brief     云台控制参数初始化
  */
static void gimbal_init_param(void);
/**
  * @brief     云台回中初始化模式处理函数
  */
static void gimbal_init_param_manual(void);
/**
  * @brief     云台回中手动初始化模式处理函数
  */
static void pid_reset_manual(void);
/**
  * @brief     云台切换手动瞄准模式pid重置函数
  */
static void pid_reset_auto(void);
/**
  * @brief     云台切换自动瞄准模式pid重置函数
  */
static void gimbal_init_handle(void);
/**
  * @brief     云台无控制信号输入模式处理函数
  */

static void gimbal_noaction_handle(void);
/**
  * @brief     云台跟随编码器闭环控制处理函数
  */
static void gimbal_loop_handle(void);

/**
  * @brief     云台自定义控制代码接口
  */
void gimbal_custom_control(void);
/**
  * @brief     云台自瞄控制代码接口
  */
void gimbal_auto_control(void);
/**
  * @brief     云台 yaw 轴位置闭环控制
  */
void gimbal_yaw_control(void);
/**
  * @brief     云台 pitch 轴位置闭环控制
  */
void gimbal_pitch_control(void);
/**
  * @brief     发射机构自定义控制代码接口
  */
void shoot_custom_control(void);
/**
  * @brief     开关摩擦轮控制函数
  */
void turn_on_off_friction_wheel(void);

#endif

