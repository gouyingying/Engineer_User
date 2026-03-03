//
// Created by pump_li on 2023/10/9.
//

#ifndef CUSTOM_CONTROLLER_IMAGE_REFEREE_HPP
#define CUSTOM_CONTROLLER_IMAGE_REFEREE_HPP
#include <cstring>
#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "usartio.hpp"
#include "CRCCheck.h"
#include "remotec.hpp"
#ifdef __cplusplus
}
#endif

#define SEND_LEN 4

#define KEY_W           ((uint16_t)1 << 0)
#define KEY_S           ((uint16_t)1 << 1)
#define KEY_A           ((uint16_t)1 << 2)
#define KEY_D           ((uint16_t)1 << 3)
#define KEY_SHIFT       ((uint16_t)1 << 4)
#define KEY_CTRL        ((uint16_t)1 << 5)
#define KEY_Q           ((uint16_t)1 << 6)
#define KEY_E           ((uint16_t)1 << 7)
#define KEY_R           ((uint16_t)1 << 8)
#define KEY_F           ((uint16_t)1 << 9)
#define KEY_G           ((uint16_t)1 << 10)
#define KEY_Z           ((uint16_t)1 << 11)
#define KEY_X           ((uint16_t)1 << 12)
#define KEY_C           ((uint16_t)1 << 13)
#define KEY_V           ((uint16_t)1 << 14)
#define KEY_B           ((uint16_t)1 << 15)



typedef struct
{
    uint8_t  Head1;                 //帧头1
    uint8_t  Head2;                 //帧头2
    uint8_t  Len;                   //长度
    // uint8_t  Cmd;                   //命令
    uint8_t Data1[SEND_LEN];   //数据
    uint8_t Data2[SEND_LEN];   //数据
    uint8_t Data3[SEND_LEN];   //数据
    uint8_t Data4[SEND_LEN];   //数据
    uint8_t Data5[SEND_LEN];   //数据
    uint8_t Data6[SEND_LEN];   //数据
    uint8_t Cmd;              //命令


    // uint16_t CRC16;                 //CRC校验
}SEND_MESSAGE;

void image_referee_send_init();
void Image_Referee_KEY_AND_MOUSE_Slove();
bool Check_Key(uint16_t which_key);
void Image_Key_PortHandle();
Image_ctrl_t Get_Image_Ctrl();
void key_move_control();
void key_pump_control();
bool C5_portSetProtect();


class cImage_Referee: public cUSARTC{
public:
    cImage_Referee(UART_HandleTypeDef *local_huart, uint16_t buf_size, eUSART_type type) :
            cUSARTC(local_huart, buf_size, type){}
    void rxUserCALLBACK() override;
};

extern cImage_Referee Image_Referee;
void Self_Control_Transfer(const uint8_t *Temp_self_control_all_data,uint8_t size);
extern uint8_t it_flag;
extern int pump_s_flag;
extern int pump_flag;
#endif CUSTOM_CONTROLLER_IMAGE_REFEREE_HPP
