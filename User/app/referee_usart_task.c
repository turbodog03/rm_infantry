/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM裁判系统数据处理
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"

#include "bsp_usart.h"

#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"

#include "rm_hal_lib.h"

#include "RM_Cilent_UI.h"
#include "string.h"
#include "math.h"

#define AUTO			//显示雷达ui
//#define SHOOT					//显示发射准星ui  //ui待修正

#define PI acos(-1)
/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */

static void referee_unpack_fifo_data(void);

 
extern UART_HandleTypeDef huart6;

uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;


float power_buffer;//底盘功率缓冲值
float chassis_power; //底盘实时功率


/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void referee_usart_task(void const * argument)
{
	
    init_referee_struct_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);
		//雷达
		Graph_Data circle;
		Graph_Data line;
#ifdef SHOOT			
		Graph_Data line1;
		Graph_Data line2;
		Graph_Data line3;
		Graph_Data line4;
		Graph_Data line5;
#endif

		memset(&circle,0,sizeof(circle));
		memset(&line,0,sizeof(line));

	#ifdef SHOOT	
		memset(&line1,0,sizeof(line1));
		memset(&line2,0,sizeof(line2));
		memset(&line3,0,sizeof(line3));
		memset(&line4,0,sizeof(line4));
		memset(&line5,0,sizeof(line5));
#endif

		
		Circle_Draw(&circle,"001",UI_Graph_ADD,9,UI_Color_Black,8,960,540,300);
//		Line_Draw(&line,"002",UI_Graph_ADD,8,UI_Color_Black,8,960,540,1260,540);			
#ifdef SHOOT
		Line_Draw(&line1,"003",UI_Graph_ADD,7,UI_Color_Black,4,910,540,1010,540);
		Line_Draw(&line2,"004",UI_Graph_ADD,7,UI_Color_Black,4,930,500,990,500);
		Line_Draw(&line3,"005",UI_Graph_ADD,7,UI_Color_Black,4,950,460,970,460);
		Line_Draw(&line4,"006",UI_Graph_ADD,7,UI_Color_Black,4,960,540,960,440);
		Line_Draw(&line5,"007",UI_Graph_ADD,7,UI_Color_Black,4,955,420,950,420);
		//瞄准线
#endif
		
		double angle = 0;
		uint32_t referee_wake_time = osKernelSysTick();
			

    while(1)
    {					
				referee_unpack_fifo_data();
				get_chassis_power_and_buffer(&chassis_power,&power_buffer);
				float tmp[]={chassis_power,power_buffer};
				write_uart(BLUETOOTH_UART,(uint8_t*)&tmp,sizeof(uint8_t)*4*2);
				uint8_t tail[]={0x00,0x00,0x80,0x7f};
				write_uart(BLUETOOTH_UART,tail,sizeof(tail));
				static int  i =  0;
				i++;
#ifdef SHOOT				
				if(i ==10){
				UI_ReFresh(2,line1,line2);
				}
				
				if(i ==20){		
					UI_ReFresh(2,line3,line4);					
				}
				if(i == 30){
					UI_ReFresh(1,line5);
				}
#endif
#ifdef AUTO
				if(i == 15 ){
					Line_Draw(&line,"002",UI_Graph_ADD,8,UI_Color_Black,8,960,540,sin(angle*PI/180)*300+960,cos(angle*PI/180)*300+540);			
					UI_ReFresh(2,circle,line);
				}
					if(i == 30){
						if(angle == 360)
						{
							angle = 0;
						}
					angle++;						
//					
					Line_Draw(&line,"002",UI_Graph_Change,8,UI_Color_Black,8,960,540,sin(angle*PI/180)*300+960,cos(angle*PI/180)*300+540);			
//					//sin()、cos()的参数是弧度，角度转弧度
					UI_ReFresh(1,line);
					//更新线的位置
				}
#endif
				if(i == 40) i=0;
				//通过if调整发送ui数据的频率
				
				osDelayUntil(&referee_wake_time, 10);
    }
}


/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  while ( fifo_s_used(&referee_fifo) )
  {
    byte = fifo_s_get(&referee_fifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            referee_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}
void USART6_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);

        static uint16_t this_time_rx_len = 0;

        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[0], this_time_rx_len);
            //detect_hook(REFEREE_TOE);
        }
        else
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[1], this_time_rx_len);
            //detect_hook(REFEREE_TOE);
        }
    }
}


