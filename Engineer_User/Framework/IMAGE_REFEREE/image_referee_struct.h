//
// Created by pump_li on 2023/9/16.
//
#ifndef CUSTOM_CONTROLLER_TRASFER_IMAGE_REFEREE_H
#define CUSTOM_CONTROLLER_TRASFER_IMAGE_REFEREE_H
#include "stm32f4xx.h"
#include "stdio.h"
#include "string.h"

#define PROTOCAL_FRAME_MAX_SIZE 128 // 整个交互数据的包总共长最大为 128 个字节
#define C5_Remotec_Offset 1024
#define FrameHeaderSOF 0xA5
#define C5_ImageSof 0x53
#define Image_Referee_RecDataBuffDepth 200 // 接收缓存区单个通道深度，
#define Image_Referee_TxDataBuffDepth 200	 // DMA发送缓存区

#pragma pack (1)
// extern uint8_t Send_Pack_buf[Image_Referee_TxDataBuffDepth+1];
typedef __packed struct
{
    uint8_t SOF;		 // 数据帧起始字节，固定值为0xA5;
    uint16_t DataLength; // 数据帧内Data长度;
    uint8_t Seq;		 // 包序号;
    uint8_t CRC8;		 // 帧头CRC8;
} frame_header_t;		 // 一个完整的通信帧里面的帧头详细定义

typedef enum
{
    SELF_CONTROL_INTERACTIVE_ID = 0x0302, // 自定义控制器交互数据
    KEY_AND_MOUSE_ID = 0x0304,			  // 键鼠信息
    Clear_Flag = 0x00,
} frame_cmd_id_e;



typedef __packed struct
{
    frame_header_t header;
    frame_cmd_id_e frame_cmd_id;
    uint8_t Data[PROTOCAL_FRAME_MAX_SIZE];
    uint16_t frame_tail;
} RefereeFullFrame_t;
// 一个完整的通信帧格式

#pragma pack ()

typedef __packed struct
{
    uint8_t header;
    uint8_t sof;
    uint8_t Data[PROTOCAL_FRAME_MAX_SIZE];
    uint16_t frame_tail;
} C5_Imgae_Frame_t;

typedef __packed struct
{
    float yaw_4310;
    float pitch_10010;
    float pitch_3508;
    float roll_3508;
    float pitch_4310;
    float roll_2006;
} self_control_t;

typedef __packed struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
} ext_robot_command_t;//图传链路客户端下发信息

#define self_controler_data_max_length 30 // 发送的内容数据段最大为 30
typedef __packed struct
{
    uint8_t Data[self_controler_data_max_length];
}self_control_data_t;//自定义控制器数据包结构体

typedef __packed struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t middle_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
} key_and_mouse_t;

typedef struct
{
    self_control_data_t self_control_all_data; // 自定义控制器命令交互数据
    key_and_mouse_t key_and_mouse;										   // 鼠标和键盘数据
} Image_receive_referee_t;
// 解包后获得的数据存在此结构体内

typedef struct
{
    uint32_t self_control_all_data; // 自定义控制器命令交互数据
    uint32_t key_and_mouse;						// 鼠标和键盘数据
} Image_Referee_UndateFlag_t;

#endif //CUSTOM_CONTROLLER_TRASFER_IMAGE_REFEREE_H
