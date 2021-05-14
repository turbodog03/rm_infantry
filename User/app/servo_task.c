#include "servo_task.h"
#include "FreeRTOS.h"
#include "rm_hal_lib.h"
#include "can_device.h"
#include "math.h"
#include "cmsis_os.h"
#include "pid.h"
//int16_t mmp_current[4]={0};
//int step=0;
extern float amplitude;
void servo_task(const void* argu)
{
	uint32_t servo_wake_time = osKernelSysTick();
	//float i=0.0f;
	//pid_init(&pid_pit_speed, 8000,6000,50, 0.01, 0.2);
	osDelay(3000);
	while(1)
	{
//		write_led_io(LED_IO1,1);
//		pit_speed_ref=amplitude*sinf(i);
//		get_imu_data(&imu);
//		pit_moto_current = pid_calc(&pid_pit_speed, imu.gyro_y, pit_speed_ref);
		//int len=sprintf(mmp_buf,"pit:%f,%f,%f,%d\n",pit_angle_ref,pit_angle_fdb,pit_speed_ref,pit_moto_current);
		//float fdata[]={imu.gyro_y,pit_speed_ref,pit_angle_ref,pit_angle_fdb};
		//float fdata[]={imu.gyro_z,yaw_speed_ref,yaw_angle_ref,yaw_angle_fdb};
		float fdata[]={0,yaw_relative_angle,yaw_angle_ref,yaw_angle_fdb};
//		float fdata[]={imu.gyro_y,pit_speed_ref,pit_moto_current};
		uint8_t tail[]={0x00,0x00,0x80,0x7f};
		write_uart(BLUETOOTH_UART,(uint8_t*)fdata,sizeof(fdata));
		write_uart(BLUETOOTH_UART,(uint8_t*)tail,sizeof(tail));
		//send_gimbal_moto_current(0, pit_moto_current);
		//i+=0.314;
		//write_led_io(LED_IO1,0);
		osDelayUntil(&servo_wake_time, 10);
	}
}