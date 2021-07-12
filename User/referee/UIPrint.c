#include "math.h"
#include "UIPrint.h"
#include "String.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "crc8_crc16.h"
uint8_t Seq=1;
/**  
graphic_tpye的可取值
0：直线；
1：矩形；
2：整圆；
3：椭圆；
4：圆弧；
5：浮点数；
6：整型数；
7：字符；
*/
/**
  * @brief          删除某一图层的UI
  * @param[in]      Delete_Operate: 图形操作 其中0：空操作 1：删除图层 2：删除所有
  * @param[in]      Layer: 图层数范围为0-9
  * @retval         none
  */
void DeleteUI(uint8_t Delete_Operate,uint8_t Layer){
	ext_client_custom_graphic_delete_t data;
	data.layer=Layer;
	data.operate_tpye=Delete_Operate;
}
/**
  * @brief          填充指定结构体变量的内容为画一条直线
  * @param[in]      data: 要被操作的结构体的地址
  * @param[in]      graphname: 图像名称，作为删除、修改等操作客户端的索引
  * @param[in]      operate_tpye：图形操作 其中0:空操作 1:增加 2:修改 3:删除
	* @param[in]      layer：图层数范围为0-9
	* @param[in]      color：颜色0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
	* @param[in]      width：线条宽度
	* @param[in]      start_x：起点 x 坐标 
	* @param[in]      start_y：起点 y 坐标
	* @param[in]      end_x：终点 x坐标
	* @param[in]      end_y：终点 y坐标
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
  * @brief          填充指定结构体变量的内容为画一个矩形
  * @param[in]      data: 要被操作的结构体的地址
  * @param[in]      graphname: 图像名称，作为删除、修改等操作客户端的索引
  * @param[in]      operate_tpye：图形操作 其中0:空操作 1:增加 2:修改 3:删除
	* @param[in]      layer：图层数范围为0-9
	* @param[in]      color：颜色0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
	* @param[in]      width：线条宽度
	* @param[in]      start_x：起点 x 坐标 
	* @param[in]      start_y：起点 y 坐标
	* @param[in]      end_x：对角顶点 x坐标
	* @param[in]      end_y：对角顶点 y坐标
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
  * @brief          填充指定结构体变量的内容为画一个整圆
  * @param[in]      data: 要被操作的结构体的地址
  * @param[in]      graphname: 图像名称，作为删除、修改等操作客户端的索引
  * @param[in]      operate_tpye：图形操作 其中0:空操作 1:增加 2:修改 3:删除
	* @param[in]      layer：图层数范围为0-9
	* @param[in]      color：颜色0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
	* @param[in]      width：线条宽度
	* @param[in]      start_x：圆心 x 坐标 
	* @param[in]      start_y：圆心 y 坐标 
	* @param[in]      radius：半径
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
  * @brief          填充指定结构体变量的内容为画字符串
  * @param[in]      data: 要被操作的结构体的地址
  * @param[in]      graphname: 图像名称，作为删除、修改等操作客户端的索引
  * @param[in]      operate_tpye：图形操作 其中0:空操作 1:增加 2:修改 3:删除
	* @param[in]      layer：图层数范围为0-9
	* @param[in]      color：颜色0：红蓝主色；1：黄色；2：绿色；3：橙色；4：紫红色；5：粉色；6：青色；7：黑色；8：白色；
	* @param[in]      start_angle：字体大小
	* @param[in]      end_angle：字符长度 
	* @param[in]      width：线条宽度
	* @param[in]      start_x：起点 x 坐标
	* @param[in]      start_y：起点 y 坐标
	* @param[in]      chardata：字符数据
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
	//帧头处理
	Head head;
	head.SOF=UI_SOF;
	head.Data_Length=DrawOneGraph;
	head.Seq=Seq;
	head.CRC8=get_CRC8_check_sum((uint8_t*)&head,4,0xFF);
	head.CMD_ID=UI_CMD_Robo_Exchange;
//数据处理
	ext_student_interactive_header_data_t datahead;
	datahead.data_cmd_id=Client_draws_one_graph;
	datahead.sender_ID=ARobot_ID;
	datahead.receiver_ID=ACilent_ID;
//帧尾处理
   frametail=get_CRC16_check_sum((unsigned char *)&head,sizeof(head),frametail);
   frametail=get_CRC16_check_sum((unsigned char *)&datahead,sizeof(datahead),frametail);
   frametail=get_CRC16_check_sum((unsigned char *)&data,sizeof(data),frametail);             //CRC16校验   //CRC16校验值计算（部分）
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
	//帧头处理
	Head head;
	head.SOF=UI_SOF;
	head.Data_Length=DrawCharacter;
	head.Seq=Seq;
	head.CRC8=get_CRC8_check_sum((uint8_t*)&head,4,0xFF);
	head.CMD_ID=UI_CMD_Robo_Exchange;
//数据处理
	ext_student_interactive_header_data_t datahead;
	datahead.data_cmd_id=Client_draws_character;
	datahead.sender_ID=ARobot_ID;
	datahead.receiver_ID=ACilent_ID;
//帧尾处理
   frametail=get_CRC16_check_sum((unsigned char *)&head,sizeof(head),frametail);
   frametail=get_CRC16_check_sum((unsigned char *)&datahead,sizeof(datahead),frametail);
   frametail=get_CRC16_check_sum((unsigned char *)&data,sizeof(data),frametail);             //CRC16校验   //CRC16校验值计算（部分）
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
	//帧头处理
	Head head;
	head.SOF=UI_SOF;
	head.Data_Length=DrawOneGraph;
	head.Seq=Seq;
	head.CRC8=get_CRC8_check_sum((uint8_t*)&head,4,0xFF);
	head.CMD_ID=UI_CMD_Robo_Exchange;
//数据处理
	ext_student_interactive_header_data_t datahead;
	datahead.data_cmd_id=Client_draws_one_graph;
	datahead.sender_ID=ARobot_ID;
	datahead.receiver_ID=ACilent_ID;
//帧尾处理
   frametail=get_CRC16_check_sum((unsigned char *)&head,sizeof(head),frametail);
   frametail=get_CRC16_check_sum((unsigned char *)&datahead,sizeof(datahead),frametail);
   frametail=get_CRC16_check_sum((unsigned char *)&data,sizeof(data),frametail);             //CRC16校验   //CRC16校验值计算（部分）
	HAL_UART_Transmit(&huart6,(unsigned char *)&head,sizeof(head),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart6,(unsigned char *)&datahead,sizeof(datahead),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart6,(unsigned char *)&imageData,sizeof(imageData),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart6,(unsigned char *)&frametail,sizeof(frametail),HAL_MAX_DELAY);
	Seq++;
}
void SendDeleteData(ext_client_custom_graphic_delete_t data){
	ext_client_custom_graphic_delete_t imageData;
	uint16_t frametail=0xFFFF;
	//帧头处理
	Head head;
	head.SOF=UI_SOF;
	head.Data_Length=DeleteGraph;
	head.Seq=Seq;
	head.CRC8=get_CRC8_check_sum((uint8_t*)&head,4,0xFF);
	head.CMD_ID=UI_CMD_Robo_Exchange;
//数据处理
	ext_student_interactive_header_data_t datahead;
	datahead.data_cmd_id=Client_delete_graphics_ID;
	datahead.sender_ID=ARobot_ID;
	datahead.receiver_ID=ACilent_ID;
//帧尾处理
	frametail=get_CRC16_check_sum((unsigned char *)&head,sizeof(head),frametail);
  frametail=get_CRC16_check_sum((unsigned char *)&datahead,sizeof(datahead),frametail);
  frametail=get_CRC16_check_sum((unsigned char *)&data,sizeof(data),frametail);             //CRC16校验   //CRC16校验值计算（部分）
	HAL_UART_Transmit(&huart6,(unsigned char *)&head,sizeof(head),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart6,(unsigned char *)&datahead,sizeof(datahead),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart6,(unsigned char *)&imageData,sizeof(imageData),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart6,(unsigned char *)&frametail,sizeof(frametail),HAL_MAX_DELAY);
	Seq++;
}
