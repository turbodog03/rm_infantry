#include "led_task.h"
#include "can_device.h"
void led_task(const void* argu)
{
	int16_t current[]={1000,1000,1000,1000};
	while(1)
	{
		send_chassis_moto_current(current);
	}
}
