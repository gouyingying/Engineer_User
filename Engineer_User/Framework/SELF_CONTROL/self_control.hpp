//
// Created by ZONE7 on 2024/9/25.
//

#ifndef SELF_CONTROL_HPP
#define SELF_CONTROL_HPP

#include <cstdint>
#include "main.h"
#include "gpio.h"
#include "dma.h"
#include "usart.h"
#include <cstring>
#include "usartio.hpp"
#define SELF_CONTROL_RVSIZE 255

class Self_Control :public cUSARTC
{
public:
    Self_Control(UART_HandleTypeDef *uart,uint16_t size,eUSART_type type);
    void rxUserCALLBACK() override;
    void Gyro_Solve();
    float Get_Control_Angle();
    float Get_Control_Last_Angle();
    void Set_Control_Last_Angle();
private:
    char self_control_rx_buf_[SELF_CONTROL_RVSIZE]{};
    char self_control_buf_[SELF_CONTROL_RVSIZE]{};
    float angle;
    float last_angle;
};


extern Self_Control self_control1;
extern Self_Control self_control2;


#endif //SELF_CONTROL_HPP
