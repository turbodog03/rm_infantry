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
 



/************************************ 用户对发射机构自定义控制 *************************************/
#include "gimbal_task.h"
#include "can_device.h"
#include "pid.h"
#include "shoot_task.h"


/* 拨弹电机期望位置(单位是编码器数值encoder) */
int32_t trigger_moto_position_ref;
/* 拨弹电机期望转速(rpm) */
int16_t trigger_moto_speed_ref;
/* 拨弹电机电流 */
int16_t trigger_moto_current;
/* 摩擦轮电机电流 */
int16_t shoot_moto_current_left;
int16_t shoot_moto_current_right;

/* 发射机构控制相关数据 */
extern int16_t  trigger_moto_speed_ref;    //拨弹电机速度目标值
extern uint8_t  fric_wheel_run;            //摩擦轮开关
extern uint16_t fric_wheel_speed;          //摩擦轮速度
extern int32_t  trigger_moto_position_ref; //拨弹电机位置目标值


float speedInNumsPerSec;
uint32_t numsOfOneShot;
uint32_t delayTimeInMs;

/* 卡弹处理 */
uint32_t stall_count = 0;
uint32_t stall_inv_count = 0;
uint8_t  stall_f = 0;
void block_bullet_handle(void)
{
  if (pid_trigger_speed.out <= -5000)  //卡弹电流
  {
    if (stall_f == 0)
      stall_count ++;
  }
  else
    stall_count = 0;
  
  if (stall_count >= 600) //卡弹时间3s
  {
    stall_f = 1;
    stall_count = 0;
    
  }
  
  if (stall_f == 1)
  {
    stall_inv_count++;
    
    if (stall_inv_count >= 100)  //反转时间0.5s
    {
      stall_f = 0;
      stall_inv_count = 0;
    }
    else
      trigger_moto_speed_ref = 2000;
  }
}

/**
  * @brief          根据期望射频计算拨弹电机速度，可实现以任意间隔任意射频发射任意数量子弹
  * @param[1]       射频，单位：发/s
  * @param[2]       单次射击子弹数
  * @param[3]       两次射击间隔时间
  * @retval         拨弹电机期望速度 单位：RPM
  */
float ShootAndDelay(float speedInNumsPerSec, uint32_t numsOfOneShot, uint32_t delayTimeInMs)
{
    static uint32_t ticksInMs = 0, lastNumsOfOneShot = 0, lastDelayTimeInMs = 0, count = 0;
    static int32_t lastAngle = 0;
    static float speed = 0;
    if (count == 0 || lastNumsOfOneShot != numsOfOneShot || lastDelayTimeInMs != delayTimeInMs)
    {
        ticksInMs = HAL_GetTick() + delayTimeInMs + 1;
        lastAngle = moto_trigger.total_angle;
    }
    if (lastAngle - moto_trigger.total_angle > 8191 * TRIGGER_MOTOR_REDUCTION_RATIO / BULLETS_PER_ROUND * numsOfOneShot)
    {
        lastAngle = moto_trigger.total_angle;
        speed = 0;
        ticksInMs = HAL_GetTick();
    }

    if (HAL_GetTick() - ticksInMs > delayTimeInMs)
        speed = speedInNumsPerSec / BULLETS_PER_ROUND * TRIGGER_MOTOR_REDUCTION_RATIO * 60;

    count++;
    lastNumsOfOneShot = numsOfOneShot;
    lastDelayTimeInMs = delayTimeInMs;
    return speed;
}

/* 子弹的单发和连发处理 */
void shoot_custom_control(void)
{
  if (fric_wheel_run)
  {
		switch(shoot_state)
		{
			case single_shoot:
				if(shoot_cmd)
				{
				/* 如果是单发命令，拨轮旋转45度 */
					trigger_moto_position_ref = moto_trigger.total_ecd + DEGREE_45_TO_ENCODER;
					shoot_cmd=0;
				}
				/* 闭环计算拨弹电机期望转速 */
					trigger_moto_speed_ref = pid_calc(&pid_trigger, moto_trigger.total_ecd, trigger_moto_position_ref);
				goto emmm;
			case trible_shoot:
				if(shoot_cmd)
				{
				/* 如果是三发命令，拨轮旋转3*45度 */
					trigger_moto_position_ref = moto_trigger.total_ecd + 3*DEGREE_45_TO_ENCODER;
					shoot_cmd=0;
				}
				/* 闭环计算拨弹电机期望转速 */
					trigger_moto_speed_ref = pid_calc(&pid_trigger, moto_trigger.total_ecd, trigger_moto_position_ref);
				goto emmm;
			case continuous_shoot:
				speedInNumsPerSec=8.0f;
			  numsOfOneShot=8;
				delayTimeInMs=10;
				break;
			case dont_shoot:
				trigger_moto_speed_ref = 0;
				goto emmm;
		}
		trigger_moto_speed_ref=-ShootAndDelay(speedInNumsPerSec,numsOfOneShot,delayTimeInMs);
    block_bullet_handle();                                 //卡弹处理
    /* 闭环计算拨弹电机电流 */
		emmm:
    trigger_moto_current = pid_calc(&pid_trigger_speed, moto_trigger.speed_rpm, trigger_moto_speed_ref);
  }
  else
  {
    trigger_moto_current = 0;
  }
	/* 闭环计算摩擦轮电机电流 */
	shoot_moto_current_left = pid_calc(&pid_shoot_left, moto_shoot[0].speed_rpm, -fric_wheel_speed);
	shoot_moto_current_right = pid_calc(&pid_shoot_right, moto_shoot[1].speed_rpm, fric_wheel_speed);
	/* 发送拨弹电机、摩擦轮电机电流 */
	send_shoot_moto_current(shoot_moto_current_left,shoot_moto_current_right,trigger_moto_current);

}

/* 开关摩擦轮处理 */
void turn_on_off_friction_wheel(void)
{
  if (fric_wheel_run)
  {
    //打开摩擦轮、充能装置
		fric_wheel_speed=SHOT_FRIC_WHEEL_SPEED;
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5,GPIO_PIN_SET);
    //set_pwm_param(PWM_IO1, fric_wheel_speed);
    //set_pwm_param(PWM_IO2, fric_wheel_speed);
    //打开激光
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_3,GPIO_PIN_SET);
    write_led_io(LASER_IO, LED_ON);
  }
  else
  {
    //关闭摩擦轮、充能装置
		fric_wheel_speed=0;
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5,GPIO_PIN_RESET);
    //set_pwm_param(PWM_IO1, 1000);
    //set_pwm_param(PWM_IO2, 1000);
    //关闭激光
    write_led_io(LASER_IO, LED_OFF);
  }
}


