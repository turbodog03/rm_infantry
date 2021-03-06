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
 


#include "startup.h"
#include "can_device.h"
#include "uart_device.h"
#include "calibrate.h"
#include "shoot_task.h"
#include "sys.h"
#include "stm32f4xx_hal_uart.h"
#include "referee_usart_task.h"


/**
  * @brief     运行在定义的任务函数执行前，可以用来初始化任务中用到的 IO 端口，
  *            配置、开启外界硬件设备，注册硬件设备的接收回调函数
  */
void init_setup(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin   = GPIO_PIN_4 | GPIO_PIN_12 | GPIO_PIN_5 | GPIO_PIN_6;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
  //关闭 LED 状态指示灯
  write_led_io(LED_G, LED_OFF);
  write_led_io(LED_R, LED_OFF);
	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_3,GPIO_PIN_RESET);
  
  //关闭所有 LED IO
  write_led_io(LED_IO1, LED_OFF);
  write_led_io(LED_IO2, LED_OFF);
  write_led_io(LED_IO3, LED_OFF);
  write_led_io(LED_IO4, LED_OFF);
  write_led_io(LED_IO5, LED_OFF);
  write_led_io(LED_IO6, LED_OFF);
  write_led_io(LED_IO7, LED_OFF);
  write_led_io(LED_IO8, LED_OFF);
  
  //读取全局校准数据
  read_cali_data();
  
  //初始化接收串口
	//遥控器
  uart_init(DBUS_UART, 100000, WORD_LEN_8B, STOP_BITS_1, PARITY_EVEN);
	//调试器
	uart_init(BLUETOOTH_UART, 115200, WORD_LEN_8B, STOP_BITS_1, PARITY_NONE);
	//MiniPC
	uart_init(NUC_UART, 115200, WORD_LEN_8B, STOP_BITS_1, PARITY_NONE);
	//uart_init(REFEREE_UART, 115200, WORD_LEN_8B, STOP_BITS_1, PARITY_NONE);
	//uart_init(USER_UART3, 115200, WORD_LEN_8B, STOP_BITS_1, PARITY_NONE);
  
	//注册遥控器接收数据回调函数
  uart_recv_callback_register(DBUS_UART, dbus_uart_callback);
	uart_recv_callback_register(NUC_UART, nuc_uart_callback);
	uart_recv_callback_register(BLUETOOTH_UART, bluetooth_uart_callback);
//	uart_recv_callback_register(REFEREE_UART,referee_uart_callback);
	
	
//	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
  //开启遥控器接收
  uart_receive_start(DBUS_UART, dbus_recv, DBUS_FRAME_SIZE);
	uart_receive_start(BLUETOOTH_UART, bluetooth_recv, BLUETOOTH_FRAME_SIZE);
	uart_receive_start(NUC_UART, nuc_recv, NUC_FRAME_SIZE);
	//uart_receive_start(REFEREE_UART,referee_recv,REFEREE_FRAME_SIZE);

  
  //初始化 CAN 设备
  can_device_init();
  //注册CAN1接收数据回调函数
  can_recv_callback_register(USER_CAN1, can1_recv_callback);
  //注册CAN2接收数据回调函数，摩擦轮和拨弹电机使用 CAN2
  can_recv_callback_register(USER_CAN2, can2_recv_callback); 
  //开启CAN接收数据中断
  can_receive_start();
	
	//设置弹仓盖pwm定时器
	set_pwm_group_param(1, 20000);
	
}







/**
* @brief     开启相关任务函数，不需要用户改动
  */
extern TaskHandle_t task1_t;
extern TaskHandle_t task2_t;
extern TaskHandle_t task3_t;
extern TaskHandle_t task4_t;
extern TaskHandle_t task5_t;

void sys_start_task(void)
{
#ifdef USER_TASK1
    osThreadDef(ostask1, USER_TASK1, osPriorityAboveNormal, 0, 128);
    task1_t = osThreadCreate(osThread(ostask1), NULL);
#endif

#ifdef USER_TASK2
    osThreadDef(ostask2, USER_TASK2, osPriorityHigh, 0, 128);
    task2_t = osThreadCreate(osThread(ostask2), NULL);
#endif

#ifdef USER_TASK3
    osThreadDef(ostask3, USER_TASK3, osPriorityAboveNormal, 0, 128);
    task3_t = osThreadCreate(osThread(ostask3), NULL);
#endif

#ifdef USER_TASK4
    osThreadDef(ostask4, USER_TASK4, osPriorityNormal, 0, 128);
    task4_t = osThreadCreate(osThread(ostask4), NULL);
#endif

#ifdef USER_TASK5
    osThreadDef(ostask5, USER_TASK5, osPriorityNormal, 0, 256);
    task5_t = osThreadCreate(osThread(ostask5), NULL);
#endif
}



