//
// Created by ZONE7 on 2024/9/27.
//

#include "uni_motor.hpp"
#include "usart.h"

uni_motor Uni_Motor(&huart6);

uni_motor::uni_motor(UART_HandleTypeDef *uart){}

void uni_motor::Init()
{
    // for (int i = 0; i < 10;) {
        cmd.id = 0;            //给电机控制指令结构体赋值
        cmd.mode = 1;
        cmd.T = 0.0;
        cmd.W = 0.0;
        cmd.Pos = 0 * 6.33;
        cmd.K_P = 0.09;
        cmd.K_W = 0.0;
        MotorFlag = SERVO_Send_recv(&cmd, &data);

    //     if (MotorFlag == HAL_OK)
    //         i++;
    //     HAL_Delay(10);
    // }
    pos_init = data.Pos;
}

void uni_motor::SetPos(float postar)
{
    // for (int i = 0; i < 10;) {
        cmd.id = 0;            //给电机控制指令结构体赋值
        cmd.mode = 1;
        cmd.T = 0.0;
        cmd.W = 0.0;
        cmd.Pos = postar * 6.33;
        cmd.K_P = 0.09;
        cmd.K_W = 0.0;
        MotorFlag = SERVO_Send_recv(&cmd, &data);

    //     if (MotorFlag == HAL_OK)
    //         i++;
    //     HAL_Delay(10);
    // }
    // pos_init = data.Pos;

}

MOTOR_recv uni_motor::Uni_Recv()
{
    return data;
}

void uni_motor::Debug()
{
    // usart_printf("%f,%f\r\n", data.Pos,cmd.Pos);
}