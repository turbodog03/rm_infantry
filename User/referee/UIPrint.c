#include "math.h"
#include "UIPrint.h"
#include "String.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "crc8_crc16.h"
uint8_t Seq=1;
/**  
graphic_tpye�Ŀ�ȡֵ
0��ֱ�ߣ�
1�����Σ�
2����Բ��
3����Բ��
4��Բ����
5����������
6����������
7���ַ���
*/
/**
  * @brief          ɾ��ĳһͼ���UI
  * @param[in]      Delete_Operate: ͼ�β��� ����0���ղ��� 1��ɾ��ͼ�� 2��ɾ������
  * @param[in]      Layer: ͼ������ΧΪ0-9
  * @retval         none
  */
void DeleteUI(uint8_t Delete_Operate,uint8_t Layer){
	ext_client_custom_graphic_delete_t data;
	data.layer=Layer;
	data.operate_tpye=Delete_Operate;
}
/**
  * @brief          ���ָ���ṹ�����������Ϊ��һ��ֱ��
  * @param[in]      data: Ҫ�������Ľṹ��ĵ�ַ
  * @param[in]      graphname: ͼ�����ƣ���Ϊɾ�����޸ĵȲ����ͻ��˵�����
  * @param[in]      operate_tpye��ͼ�β��� ����0:�ղ��� 1:���� 2:�޸� 3:ɾ��
	* @param[in]      layer��ͼ������ΧΪ0-9
	* @param[in]      color����ɫ0��������ɫ��1����ɫ��2����ɫ��3����ɫ��4���Ϻ�ɫ��5����ɫ��6����ɫ��7����ɫ��8����ɫ��
	* @param[in]      width���������
	* @param[in]      start_x����� x ���� 
	* @param[in]      start_y����� y ����
	* @param[in]      end_x���յ� x����
	* @param[in]      end_y���յ� y����
  * @retval         none
  */
void DrawLine(graphic_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y){
	//memset(&data,0,sizeof(data));
	data->graphic_name[0]=graphname[0];
	data->graphic_name[1]=graphname[1];
	data->graphic_name[2]=graphname[2];
	data->operate_tpye=operate_tpye;
	data->graphic_tpye=0;
	data->layer=layer;
	data->color=color;
	data->width=width;
	data->start_x=start_x;
	data->start_y=start_y;
	data->end_x=end_x;
	data->end_y=end_y;
}
/**
  * @brief          ���ָ���ṹ�����������Ϊ��һ������
  * @param[in]      data: Ҫ�������Ľṹ��ĵ�ַ
  * @param[in]      graphname: ͼ�����ƣ���Ϊɾ�����޸ĵȲ����ͻ��˵�����
  * @param[in]      operate_tpye��ͼ�β��� ����0:�ղ��� 1:���� 2:�޸� 3:ɾ��
	* @param[in]      layer��ͼ������ΧΪ0-9
	* @param[in]      color����ɫ0��������ɫ��1����ɫ��2����ɫ��3����ɫ��4���Ϻ�ɫ��5����ɫ��6����ɫ��7����ɫ��8����ɫ��
	* @param[in]      width���������
	* @param[in]      start_x����� x ���� 
	* @param[in]      start_y����� y ����
	* @param[in]      end_x���ԽǶ��� x����
	* @param[in]      end_y���ԽǶ��� y����
  * @retval         none
  */
void DrawRectangle(graphic_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y){
	//memset(&data,0,sizeof(data));
	data->graphic_name[0]=graphname[0];
	data->graphic_name[1]=graphname[1];
	data->graphic_name[2]=graphname[2];
	data->operate_tpye=operate_tpye;
	data->graphic_tpye=1;
	data->layer=layer;
	data->color=color;
	data->width=width;
	data->start_x=start_x;
	data->start_y=start_y;
	data->end_x=end_x;
	data->end_y=end_y;
}
/**
  * @brief          ���ָ���ṹ�����������Ϊ��һ����Բ
  * @param[in]      data: Ҫ�������Ľṹ��ĵ�ַ
  * @param[in]      graphname: ͼ�����ƣ���Ϊɾ�����޸ĵȲ����ͻ��˵�����
  * @param[in]      operate_tpye��ͼ�β��� ����0:�ղ��� 1:���� 2:�޸� 3:ɾ��
	* @param[in]      layer��ͼ������ΧΪ0-9
	* @param[in]      color����ɫ0��������ɫ��1����ɫ��2����ɫ��3����ɫ��4���Ϻ�ɫ��5����ɫ��6����ɫ��7����ɫ��8����ɫ��
	* @param[in]      width���������
	* @param[in]      start_x��Բ�� x ���� 
	* @param[in]      start_y��Բ�� y ���� 
	* @param[in]      radius���뾶
  * @retval         none
  */
void DrawFullCircle(graphic_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t radius){
	//memset(&data,0,sizeof(data));
	data->graphic_name[0]=graphname[0];
	data->graphic_name[1]=graphname[1];
	data->graphic_name[2]=graphname[2];
	data->operate_tpye=operate_tpye;
	data->graphic_tpye=2;
	data->layer=layer;
	data->color=color;
	data->width=width;
	data->start_x=start_x;
	data->start_y=start_y;
	data->radius=radius;
}
/**
  * @brief          ���ָ���ṹ�����������Ϊ���ַ���
  * @param[in]      data: Ҫ�������Ľṹ��ĵ�ַ
  * @param[in]      graphname: ͼ�����ƣ���Ϊɾ�����޸ĵȲ����ͻ��˵�����
  * @param[in]      operate_tpye��ͼ�β��� ����0:�ղ��� 1:���� 2:�޸� 3:ɾ��
	* @param[in]      layer��ͼ������ΧΪ0-9
	* @param[in]      color����ɫ0��������ɫ��1����ɫ��2����ɫ��3����ɫ��4���Ϻ�ɫ��5����ɫ��6����ɫ��7����ɫ��8����ɫ��
	* @param[in]      start_angle�������С
	* @param[in]      end_angle���ַ����� 
	* @param[in]      width���������
	* @param[in]      start_x����� x ����
	* @param[in]      start_y����� y ����
	* @param[in]      chardata���ַ�����
  * @retval         none
  */
void DrawString(ext_client_custom_character_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t start_angle,uint32_t width,uint32_t start_x,uint32_t start_y,char chardata[30]){
	int i=0;
	while(chardata[i]!='\0'){
	data->chardata[i]=chardata[i];
	i++;
	}
	data->grapic_data_struct.graphic_name[0]=graphname[0];
	data->grapic_data_struct.graphic_name[1]=graphname[1];
	data->grapic_data_struct.graphic_name[2]=graphname[2];
	data->grapic_data_struct.operate_tpye=operate_tpye;
	data->grapic_data_struct.graphic_tpye=7;
	data->grapic_data_struct.layer=layer;
	data->grapic_data_struct.color=color;
	data->grapic_data_struct.start_angle=start_angle;
	data->grapic_data_struct.end_angle=i;
	data->grapic_data_struct.width=width;
	data->grapic_data_struct.start_x=start_x;
	data->grapic_data_struct.start_y=start_y;
}
void SendPredictData(graphic_data_struct_t data,uint32_t End_x,uint32_t End_y){
	graphic_data_struct_t imageData;
	imageData=data;
	imageData.end_x=End_x;
	imageData.end_y=End_y;
	uint16_t frametail=0xFFFF;
	//֡ͷ����
	Head head;
	head.SOF=UI_SOF;
	head.Data_Length=DrawOneGraph;
	head.Seq=Seq;
	head.CRC8=get_CRC8_check_sum((uint8_t*)&head,4,0xFF);
	head.CMD_ID=UI_CMD_Robo_Exchange;
//���ݴ���
	ext_student_interactive_header_data_t datahead;
	datahead.data_cmd_id=Client_draws_one_graph;
	datahead.sender_ID=ARobot_ID;
	datahead.receiver_ID=ACilent_ID;
//֡β����
   frametail=get_CRC16_check_sum((unsigned char *)&head,sizeof(head),frametail);
   frametail=get_CRC16_check_sum((unsigned char *)&datahead,sizeof(datahead),frametail);
   frametail=get_CRC16_check_sum((unsigned char *)&data,sizeof(data),frametail);             //CRC16У��   //CRC16У��ֵ���㣨���֣�
	HAL_UART_Transmit(&huart6,(unsigned char *)&head,sizeof(head),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart6,(unsigned char *)&datahead,sizeof(datahead),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart6,(unsigned char *)&imageData,sizeof(imageData),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart6,(unsigned char *)&frametail,sizeof(frametail),HAL_MAX_DELAY);
	Seq++;
}
void SendStringData(ext_client_custom_character_t data){
	ext_client_custom_character_t imageData;
	imageData=data;
	uint16_t frametail=0xFFFF;
	//֡ͷ����
	Head head;
	head.SOF=UI_SOF;
	head.Data_Length=DrawCharacter;
	head.Seq=Seq;
	head.CRC8=get_CRC8_check_sum((uint8_t*)&head,4,0xFF);
	head.CMD_ID=UI_CMD_Robo_Exchange;
//���ݴ���
	ext_student_interactive_header_data_t datahead;
	datahead.data_cmd_id=Client_draws_character;
	datahead.sender_ID=ARobot_ID;
	datahead.receiver_ID=ACilent_ID;
//֡β����
   frametail=get_CRC16_check_sum((unsigned char *)&head,sizeof(head),frametail);
   frametail=get_CRC16_check_sum((unsigned char *)&datahead,sizeof(datahead),frametail);
   frametail=get_CRC16_check_sum((unsigned char *)&data,sizeof(data),frametail);             //CRC16У��   //CRC16У��ֵ���㣨���֣�
	HAL_UART_Transmit(&huart6,(unsigned char *)&head,sizeof(head),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart6,(unsigned char *)&datahead,sizeof(datahead),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart6,(unsigned char *)&imageData,sizeof(imageData),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart6,(unsigned char *)&frametail,sizeof(frametail),HAL_MAX_DELAY);
	Seq++;
}
void SendData(graphic_data_struct_t data){
	graphic_data_struct_t imageData;
	imageData=data;
	uint16_t frametail=0xFFFF;
	//֡ͷ����
	Head head;
	head.SOF=UI_SOF;
	head.Data_Length=DrawOneGraph;
	head.Seq=Seq;
	head.CRC8=get_CRC8_check_sum((uint8_t*)&head,4,0xFF);
	head.CMD_ID=UI_CMD_Robo_Exchange;
//���ݴ���
	ext_student_interactive_header_data_t datahead;
	datahead.data_cmd_id=Client_draws_one_graph;
	datahead.sender_ID=ARobot_ID;
	datahead.receiver_ID=ACilent_ID;
//֡β����
   frametail=get_CRC16_check_sum((unsigned char *)&head,sizeof(head),frametail);
   frametail=get_CRC16_check_sum((unsigned char *)&datahead,sizeof(datahead),frametail);
   frametail=get_CRC16_check_sum((unsigned char *)&data,sizeof(data),frametail);             //CRC16У��   //CRC16У��ֵ���㣨���֣�
	HAL_UART_Transmit(&huart6,(unsigned char *)&head,sizeof(head),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart6,(unsigned char *)&datahead,sizeof(datahead),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart6,(unsigned char *)&imageData,sizeof(imageData),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart6,(unsigned char *)&frametail,sizeof(frametail),HAL_MAX_DELAY);
	Seq++;
}
void SendDeleteData(ext_client_custom_graphic_delete_t data){
	ext_client_custom_graphic_delete_t imageData;
	uint16_t frametail=0xFFFF;
	//֡ͷ����
	Head head;
	head.SOF=UI_SOF;
	head.Data_Length=DeleteGraph;
	head.Seq=Seq;
	head.CRC8=get_CRC8_check_sum((uint8_t*)&head,4,0xFF);
	head.CMD_ID=UI_CMD_Robo_Exchange;
//���ݴ���
	ext_student_interactive_header_data_t datahead;
	datahead.data_cmd_id=Client_delete_graphics_ID;
	datahead.sender_ID=ARobot_ID;
	datahead.receiver_ID=ACilent_ID;
//֡β����
	frametail=get_CRC16_check_sum((unsigned char *)&head,sizeof(head),frametail);
  frametail=get_CRC16_check_sum((unsigned char *)&datahead,sizeof(datahead),frametail);
  frametail=get_CRC16_check_sum((unsigned char *)&data,sizeof(data),frametail);             //CRC16У��   //CRC16У��ֵ���㣨���֣�
	HAL_UART_Transmit(&huart6,(unsigned char *)&head,sizeof(head),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart6,(unsigned char *)&datahead,sizeof(datahead),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart6,(unsigned char *)&imageData,sizeof(imageData),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart6,(unsigned char *)&frametail,sizeof(frametail),HAL_MAX_DELAY);
	Seq++;
}
