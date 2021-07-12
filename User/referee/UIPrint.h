//本UI绘制库根据裁判协议V1.2(20210430)编写	请自行更改机器人角色设置Robot_ID和Cilent_ID(6,7行)
#include "stdint.h"
//frame_header(5-byte)+cmd_id(2-byte)+data(n-byte)+frame_tail(2-byte,CRC16,整包校验)
//其中frame_header为SOF(1-byte)+data_length(2-byte)+seq(1-byte)+CRC8(1-byte)   datalength非常重要，一定要确保正确

#define ARobot_ID UI_Data_RobotID_BStandard3
#define ACilent_ID UI_Data_CilentID_BStandard3
/****************************数据帧起始字节，为固定值********************/
#define UI_SOF 0xA5
/****************************cmd_id********************/
#define UI_CMD_Robo_Exchange 0x0301
/****************************数据帧帧头数据段内容ID*********************/
#define Client_delete_graphics_ID 0x0100
#define Client_draws_one_graph 0x0101
#define Client_draws_two_graph 0x0102
#define Client_draws_five_graph 0x0103
#define Client_draws_seven_graph 0x0104
#define Client_draws_character 0x0110
/****************************不同数据的datalength*********************/
#define DeleteGraph 8
#define DrawOneGraph 21
#define DrawTwoGraph 36
#define DrawFiveGraph 81
#define DrawSevenGraph 111
#define DrawCharacter 51
/****************************红方机器人ID********************/
#define UI_Data_RobotID_RHero 1         
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************蓝方机器人ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************红方操作手ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************蓝方操作手ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A
typedef  __packed struct
{
   uint8_t SOF;                    //起始字节,固定0xA5
   uint16_t Data_Length;           //帧数据长度
   uint8_t Seq;                    //包序号
   uint8_t CRC8;                   //CRC8校验值
   uint16_t CMD_ID;                //命令ID
} Head;
typedef __packed struct
{
 uint16_t data_cmd_id;             //数据段内容ID
 uint16_t sender_ID;
 uint16_t receiver_ID;
}ext_student_interactive_header_data_t;  //数据帧帧头
typedef __packed struct
{
uint8_t operate_tpye;
uint8_t layer;
} ext_client_custom_graphic_delete_t;  //删除操作数据帧

typedef __packed struct
{
uint8_t graphic_name[3];
uint32_t operate_tpye:3;
uint32_t graphic_tpye:3;
uint32_t layer:4;
uint32_t color:4;
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10;
uint32_t start_x:11;
uint32_t start_y:11;
uint32_t radius:10;
uint32_t end_x:11;
uint32_t end_y:11;
} graphic_data_struct_t;  //图形数据数据帧
typedef __packed struct
{
uint8_t graphic_name[3];
uint32_t operate_tpye:3;
uint32_t graphic_tpye:3;
uint32_t layer:4;
uint32_t color:4;
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10;
uint32_t start_x:11;
uint32_t start_y:11;
uint32_t data;
} int_data_struct_t;  //数学数据数据帧（整形、浮点型）

typedef __packed struct
{
 graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t; //一个图形
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;  //两个图形
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;  //5个图形
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;  //7个图形
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct;
uint8_t chardata[30];
} ext_client_custom_character_t;  //字符图形
void DeleteUI(uint8_t Delete_Operate,uint8_t Layer);
void DrawLine(graphic_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y);
void DrawRectangle(graphic_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y);
void DrawFullCircle(graphic_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t radius);
void DrawString(ext_client_custom_character_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t start_angle,uint32_t width,uint32_t start_x,uint32_t start_y,char chardata[30]);
void SendPredictData(graphic_data_struct_t data,uint32_t End_x,uint32_t End_y);
void SendStringData(ext_client_custom_character_t data);
void SendData(graphic_data_struct_t data);
void SendDeleteData(ext_client_custom_graphic_delete_t data);
