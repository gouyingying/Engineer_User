//
// Created by ZONE7 on 2024/10/16.
//

#ifndef SEND_HPP
#define SEND_HPP

#include "usart.h"
#include "self_control.hpp"
#define SEND_LEN 8

class send {

};


typedef struct
{
    uint8_t  Head1;                 //帧头1
    uint8_t  Head2;                 //帧头2
    uint8_t  Len;                   //长度
    uint8_t Data[SEND_LEN];   //数据


    // uint16_t CRC16;                 //CRC校验
}SEND_MESSAGE_CHASSIS;

void send_init();
void send_to_chassis(int16_t vx, int16_t vy, int16_t vz, int8_t car_mode);
extern SEND_MESSAGE_CHASSIS send_message;
#endif //SEND_HPP
