#ifndef __SERVO_TASK_H__
#define __SERVO_TASK_H__

#include "rm_hal_lib.h"
#include "can_device.h"
void servo_task(const void* argu);

extern float yaw_angle_ref;
extern float pit_angle_ref;
extern float yaw_speed_ref;
extern float pit_speed_ref;
extern float pit_angle_fdb; 
extern float yaw_angle_fdb; 
extern float yaw_relative_angle;
extern int16_t yaw_moto_current;
extern int16_t pit_moto_current;
extern imu_t  imu;

extern moto_measure_t moto_chassis[4];
extern int16_t chassis_moto_speed_ref[4];

extern char mmp_buf[];
#endif
