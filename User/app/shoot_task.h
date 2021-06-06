#ifndef __led_task_H__
#define __led_task_H__

#include "sys.h"
#include "cmsis_os.h"

/*************************发射速度设置*******************************/
#define SHOT_FRIC_WHEEL_SPEED    7000 //最大为2500
#define SHOT_SUCCESS_FRIC_WHEEL_SPEED  6800  //发射成功摩擦轮会减速
#define SHOT_ABLE_FRIC_WHEEL_SPEED  6000
#define SHOOT_PERIOD 10
/* 单发拨弹的编码器行程 */
#define DEGREE_60_TO_ENCODER  49146
#define DEGREE_45_TO_ENCODER -36860

#define TRIGGER_MOTOR_REDUCTION_RATIO 36
#define BULLETS_PER_ROUND 8

/*************************发射频率设置*******************************/
#define TRIGGER_MOTOR_SPEED      1500 //

void shoot_task(const void* argu);
void auto_shoot_control();
enum SHOOT_STATE
{
	single_shoot,
	trible_shoot,
	continuous_shoot,
	dont_shoot,
};

extern enum SHOOT_STATE shoot_state; 
extern uint8_t shoot_cmd;

#endif

