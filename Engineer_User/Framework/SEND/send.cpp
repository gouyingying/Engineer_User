//
// Created by ZONE7 on 2024/10/16.
//

#include "send.hpp"
#include "ledio.hpp"

SEND_MESSAGE_CHASSIS send_message;


void send_init() {
    send_message.Head1 = 0x04;
    send_message.Head2 = 0x02;
    send_message.Len = 8;
}

void send_to_chassis(int16_t vx, int16_t vy, int16_t vz, int8_t car_mode) {
    // usart_printf("%.2f, %.2f, %.2f，%.2f, %.2f, %.2f\r\n",self_control1.Get_Control_Angle(),self_control2.Get_Control_Angle(),self_control3.Get_Control_Angle(),self_control4.Get_Control_Angle(),self_control5.Get_Control_Angle(),self_control6.Get_Control_Angle());
    send_message.Data[0]= vx >> 8;
    send_message.Data[1]= vx & 0xff;
    send_message.Data[2]= vy >> 8;
    send_message.Data[3]= vy & 0xff;
    send_message.Data[4]= vz >> 8;
    send_message.Data[5]= vz & 0xff;
    send_message.Data[6]= car_mode;
    // send_message.Data[7]= 0x00;



    // usart_printf("send_buf1: %.2f, send_buf2: %.2f, send_buf3: %.2f, send_buf4: %.2f, send_buf5: %.2f, send_buf6: %.2f\r\n",send_buf1,send_buf2,send_buf3,send_buf4,send_buf5,send_buf6);


    // HAL_UART_Transmit(&huart4, (uint8_t*)&send_message, 28,0xffff);
    HAL_UART_Transmit_IT(&huart4, (uint8_t*)&send_message, 12);
    // HAL_UART_Transmit_DMA(&huart4, (uint8_t*)&send_message, 12);
}
//
// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//     switch (GPIO_Pin)
//     {
//         case KEY1_Pin:
//             send_message.Cmd = 0x55;
//             LED.ON();
//             break;
//         case KEY2_Pin:
//             send_message.Cmd = 0x00;
//             LED.OFF();
//             break;
//         default:
//             break;
//     }
// }