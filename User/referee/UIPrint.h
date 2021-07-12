//��UI���ƿ���ݲ���Э��V1.2(20210430)��д	�����и��Ļ����˽�ɫ����Robot_ID��Cilent_ID(6,7��)
#include "stdint.h"
//frame_header(5-byte)+cmd_id(2-byte)+data(n-byte)+frame_tail(2-byte,CRC16,����У��)
//����frame_headerΪSOF(1-byte)+data_length(2-byte)+seq(1-byte)+CRC8(1-byte)   datalength�ǳ���Ҫ��һ��Ҫȷ����ȷ

#define ARobot_ID UI_Data_RobotID_BStandard3
#define ACilent_ID UI_Data_CilentID_BStandard3
/****************************����֡��ʼ�ֽڣ�Ϊ�̶�ֵ********************/
#define UI_SOF 0xA5
/****************************cmd_id********************/
#define UI_CMD_Robo_Exchange 0x0301
/****************************����֡֡ͷ���ݶ�����ID*********************/
#define Client_delete_graphics_ID 0x0100
#define Client_draws_one_graph 0x0101
#define Client_draws_two_graph 0x0102
#define Client_draws_five_graph 0x0103
#define Client_draws_seven_graph 0x0104
#define Client_draws_character 0x0110
/****************************��ͬ���ݵ�datalength*********************/
#define DeleteGraph 8
#define DrawOneGraph 21
#define DrawTwoGraph 36
#define DrawFiveGraph 81
#define DrawSevenGraph 111
#define DrawCharacter 51
/****************************�췽������ID********************/
#define UI_Data_RobotID_RHero 1         
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************����������ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************�췽������ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************����������ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A
typedef  __packed struct
{
   uint8_t SOF;                    //��ʼ�ֽ�,�̶�0xA5
   uint16_t Data_Length;           //֡���ݳ���
   uint8_t Seq;                    //�����
   uint8_t CRC8;                   //CRC8У��ֵ
   uint16_t CMD_ID;                //����ID
} Head;
typedef __packed struct
{
 uint16_t data_cmd_id;             //���ݶ�����ID
 uint16_t sender_ID;
 uint16_t receiver_ID;
}ext_student_interactive_header_data_t;  //����֡֡ͷ
typedef __packed struct
{
uint8_t operate_tpye;
uint8_t layer;
} ext_client_custom_graphic_delete_t;  //ɾ����������֡

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
} graphic_data_struct_t;  //ͼ����������֡
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
} int_data_struct_t;  //��ѧ��������֡�����Ρ������ͣ�

typedef __packed struct
{
 graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t; //һ��ͼ��
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;  //����ͼ��
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;  //5��ͼ��
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;  //7��ͼ��
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct;
uint8_t chardata[30];
} ext_client_custom_character_t;  //�ַ�ͼ��
void DeleteUI(uint8_t Delete_Operate,uint8_t Layer);
void DrawLine(graphic_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y);
void DrawRectangle(graphic_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y);
void DrawFullCircle(graphic_data_struct_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t width,uint32_t start_x,uint32_t start_y,uint32_t radius);
void DrawString(ext_client_custom_character_t *data,char graphname[3],uint32_t operate_tpye,uint32_t layer,uint32_t color,uint32_t start_angle,uint32_t width,uint32_t start_x,uint32_t start_y,char chardata[30]);
void SendPredictData(graphic_data_struct_t data,uint32_t End_x,uint32_t End_y);
void SendStringData(ext_client_custom_character_t data);
void SendData(graphic_data_struct_t data);
void SendDeleteData(ext_client_custom_graphic_delete_t data);
