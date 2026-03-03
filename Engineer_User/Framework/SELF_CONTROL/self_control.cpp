//
// Created by ZONE7 on 2024/9/25.
//


#include "self_control.hpp"

#include "ledio.hpp"

// Self_Control self_control1(&huart4, 255,DMA_IDLE_IT);
// Self_Control self_control2(&huart4, 255,DMA_IDLE_IT);


void USART_RxCallback(uint8_t *rx_buf){};


Self_Control::Self_Control(UART_HandleTypeDef *uart, uint16_t size, eUSART_type type):cUSARTC(uart, size, type)
{}

void Self_Control::rxUserCALLBACK()
{
    memcpy(self_control_rx_buf_, rx_buf_, SELF_CONTROL_RVSIZE);
    //解包预处理
    //将串口收到的数据进行处理，新的数组以数字开头，便于之后字符转浮点数的运算
    memcpy(self_control_buf_, &self_control_rx_buf_[0], 9);
    Gyro_Solve();

    memset(self_control_buf_,0,9);
}
/*解包计算编码器角度值
 *
 */
void Self_Control::Gyro_Solve()
{


    angle = (float)(*(self_control_buf_ + 3) << 24 | *(self_control_buf_ + 4) << 16 | *(self_control_buf_ + 5) << 8 | *(self_control_buf_ + 6))/262144*360;
    angle = angle - 180;
    // usart_printf("%.2f\r\n",angle);
    // HAL_GPIO_WritePin(GPIOH,LED_B_Pin,GPIO_PIN_SET);

}

float Self_Control::Get_Control_Angle()
{
    return angle;
}

float Self_Control::Get_Control_Last_Angle()
{
    return last_angle;
}

void Self_Control::Set_Control_Last_Angle()
{
    last_angle = angle;
}



