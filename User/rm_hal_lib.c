#include "rm_hal_lib.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"

void stop_pwm_output(uint8_t pwm_id)
{
  switch ( pwm_id )
  {
    case 1:
      HAL_TIM_PWM_Stop(&htim4, 0);
      break;
    case 2:
      HAL_TIM_PWM_Stop(&htim4, 4);
      break;
    case 3:
      HAL_TIM_PWM_Stop(&htim4, 8);
      break;
    case 4:
      HAL_TIM_PWM_Stop(&htim4, 12);
      break;
    case 5:
      HAL_TIM_PWM_Stop(&htim5, 0);
      break;
    case 6:
      HAL_TIM_PWM_Stop(&htim5, 4);
      break;
    case 7:
      HAL_TIM_PWM_Stop(&htim5, 8);
      break;
    case 8:
      HAL_TIM_PWM_Stop(&htim5, 12);
      break;
    case 9:
      HAL_TIM_PWM_Stop(&htim2, 0);
      break;
    case 10:
      HAL_TIM_PWM_Stop(&htim2, 4);
      break;
    case 11:
      HAL_TIM_PWM_Stop(&htim2, 8);
      break;
    case 12:
      HAL_TIM_PWM_Stop(&htim2, 12);
      break;
    case 13:
      HAL_TIM_PWM_Stop(&htim8, 0);
      break;
    case 14:
      HAL_TIM_PWM_Stop(&htim8, 4);
      break;
    case 15:
      HAL_TIM_PWM_Stop(&htim8, 8);
      break;
    case 16:
      HAL_TIM_PWM_Stop(&htim8, 12);
      break;
    default:
      return;
  }
  return;
}