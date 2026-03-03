//
// Created by ZONE7 on 2024/9/27.
//

#ifndef UNI_MOTOR_HPP
#define UNI_MOTOR_HPP
#include "unitree.hpp"
#include "usart.h"
#include "usartio.hpp"
#include "crc_ccitt.hpp"



class uni_motor {
    public:
         uni_motor(UART_HandleTypeDef *uart);
        void Init();
        void SetPos(float postar);
        void Debug();
    MOTOR_recv Uni_Recv();
    private:
        MOTOR_send cmd = {0};
        MOTOR_recv data = {0};
        float pos_init = 0;
        bool uni_init = false;
        HAL_StatusTypeDef MotorFlag=HAL_ERROR;
};

extern uni_motor Uni_Motor;

#endif //UNI_MOTOR_HPP
