#include "shoot_task.h"
#include "can_device.h"
#include "rm_hal_lib.h"
#include "pid.h"
#include "uart_device.h"
#include "keyboard.h"
#include "detect_task.h"
#include "stdlib.h"
#include "uart_device.h"

extern void turn_on_off_friction_wheel(void);
void cap_control();
void auto_shoot_control();
extern int16_t trigger_moto_speed_ref;
extern int32_t  trigger_moto_position_ref; //�������λ��Ŀ��ֵ
extern char mmp_buf[20];
void shoot_custom_control(void);
int cap_open_flag = 1;
int cap_ok = 1;
int auto_shoot_cmd = 0;
int auto_shoot_ok = 0;

/* ���������ز��� */
enum SHOOT_STATE shoot_state; 
uint8_t   shoot_cmd = 0;
uint32_t  continue_shoot_time;
//uint8_t   continuous_shoot_cmd = 0;
uint8_t   fric_wheel_run = 0;
uint16_t  fric_wheel_speed = SHOT_FRIC_WHEEL_SPEED;
/* �ϴε�ң���������� */
uint8_t   last_left_key;
uint8_t   last_right_key;
uint8_t   last_sw1;
uint8_t   last_sw2;
int16_t   last_wheel_value;
//ң��������Ħ����
#define RC_FIRC_CTRL     ((last_sw1 != RC_UP) && (rc.sw1 == RC_UP))           //((last_wheel_value != -660) && (rc.wheel == -660))
//ң��������
#define RC_SINGLE_TRIG   ((last_sw1 != RC_MI) && (rc.sw1 == RC_MI))           // ((last_wheel_value != 660) && (rc.wheel == 660))
//ң��������
#define RC_CONTIN_TRIG   ((rc.sw1 == RC_MI) && (HAL_GetTick() - continue_shoot_time >= 1500))   //((rc.wheel == 660) && (HAL_GetTick() - continue_shoot_time >= 1000))
//ң�����˳�����ģʽ
#define EXIT_CONTIN_TRIG (last_sw1 != RC_MI)                                 // (rc.wheel <= 600)

void shoot_task(const void* argu)
{
	//��������ʼ��
	
	/* �������PID������ʼ�� */
  pid_init(&pid_trigger, 4000, 2000, 0.15f, 0, 0);
  pid_init(&pid_trigger_speed, 9000, 4000, 1.5, 0.05, 0);
	/* Ħ���ֵ��PID������ʼ�� */
	pid_init(&pid_shoot_left, 7000, 3000, 20.0f, 0.2, 0.0);
	pid_init(&pid_shoot_right, 7000, 3000,20.0f, 0.2, 0.0);
	uint32_t shoot_wake_time = osKernelSysTick();
	while(1)
	{
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
		/* ����Ħ���ֲ������� */
//		sprintf(mmp_buf,"%d %d\n",rc.sw1,last_sw1);
//		write_uart(BLUETOOTH_UART,(uint8_t*)mmp_buf,20);
//		if ( RC_FIRC_CTRL && rc.sw2 == RC_UP)
//				fric_wheel_run = !fric_wheel_run;
		
		if (rc.kb.bit.Q && rc.sw2 != RC_DN)
			fric_wheel_run = 1;
		
		if ((rc.kb.bit.Q && rc.kb.bit.SHIFT) || rc.sw2 == RC_DN)
			fric_wheel_run = 0;

		if (glb_err.err_list[REMOTE_CTRL_OFFLINE].err_exist)
			fric_wheel_run = 0;
		//���ص��ָ�
		if (rc.kb.bit.R){
			cap_open_flag = -1;
			cap_ok = 0;
		}
		if (rc.kb.bit.R &&rc.kb.bit.SHIFT){
			cap_open_flag = 1;
			cap_ok = 0;
		}
		
		/*�Զ����ʵ���ж��߼�*/
		void auto_shoot_control();
			
		
		/* ����Ħ����ʵ�ֺ��� */
		turn_on_off_friction_wheel();
		
		/* bullet single or continue trigger command control  */
		{
			if ( RC_SINGLE_TRIG                  //ң��������
				|| (rc.mouse.l && shoot_state==dont_shoot) || auto_shoot_cmd ) //��굥��
			{
				shoot_cmd=1;
				continue_shoot_time = HAL_GetTick();
				if(rc.kb.bit.SHIFT)
					shoot_state=trible_shoot;
				else
					shoot_state=single_shoot;
			}
			else if ( RC_CONTIN_TRIG             //ң��������
				|| rc.mouse.r ) //�������
			{
				shoot_state=continuous_shoot;
				trigger_moto_position_ref=moto_trigger.total_ecd;
			}
			else if(HAL_GetTick()-continue_shoot_time>500)
				shoot_state=dont_shoot;
			
//			if ( EXIT_CONTIN_TRIG               //�˳���������
//				|| ((km.rk_sta == KEY_RELEASE) && (last_right_key == KEY_PRESS_LONG)) )
//			{
//				trigger_moto_position_ref = moto_trigger.total_ecd;
//			}
			
			if (fric_wheel_run == 0)
			{
				shoot_state=dont_shoot;
			}
		}


		/* �����������ʵ�ֺ��� */
		shoot_custom_control();
		cap_control();
		
		if(rc.sw1&&(last_sw1!=rc.sw1))
		{
			last_sw1 = rc.sw1;
			if(rc.sw1==RC_UP)
				fric_wheel_run = !fric_wheel_run;
		}
		last_sw2 = rc.sw2;
		last_left_key    = km.lk_sta;
		last_right_key   = km.rk_sta;
		last_wheel_value = rc.wheel;
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
		osDelayUntil(&shoot_wake_time, SHOOT_PERIOD);
	}
}



void cap_control(){
	if(!cap_ok){
		if(cap_open_flag == 1){
			set_pwm_param(1, 1000);
		}
		else{
			set_pwm_param(1,2000);
		}
		start_pwm_output(1);
		cap_ok = 1;
	}
}

void auto_shoot_control(){
		if(!auto_shoot_ok){
			if(data_recv.shootCommand == 1){
				auto_shoot_cmd = 1;
				auto_shoot_ok = 1;
			}
		}
		if(data_recv.shootCommand ==0){
			auto_shoot_cmd = 0;
			auto_shoot_ok = 0;
		}
	}