//
// Created by ZONE7 on 2024/10/16.
//

#ifndef RECEIVE_HPP
#define RECEIVE_HPP

#include <cstdint>
#include "main.h"
#include "gpio.h"
#include "dma.h"
#include "usart.h"
#include <cstring>
#include "usartio.hpp"
#define RECEIVE_RVSIZE 255

class receive :public cUSARTC
{
public:
    receive(UART_HandleTypeDef *uart,uint16_t size,eUSART_type type);
    void rxUserCALLBACK() override;
    void Gyro_Solve();
    float Get_Angle1();
    float Get_Angle2();
    float Get_Angle3();
    float Get_Angle4();
    float Get_Angle5();
    float Get_Angle6();

private:
    char receive_rx_buf_[RECEIVE_RVSIZE]{};
    char receive_buf_[RECEIVE_RVSIZE]{};
    float receive_buf1;
    float receive_buf2;
    float receive_buf3;
    float receive_buf4;
    float receive_buf5;
    float receive_buf6;

};

extern receive receive_from_self_control;

#endif //RECEIVE_HPP
