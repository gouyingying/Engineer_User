//
// Created by ZONE7 on 2024/9/28.
//

#include "debugc.hpp"

DebugParam cDEBUGC::Debug_Param = {0};
cDEBUGC debug(&huart1, DEBUG_RVSIZE, DMA_IDLE_IT);
cDEBUGC::cDEBUGC(UART_HandleTypeDef *uart, uint16_t size, eUSART_type type):cUSARTC(uart, size, type)
{}

DebugParam& cDEBUGC::Param()
{
    return Debug_Param;
}
DebugParam &param = debug.Param();
void cDEBUGC::rxUserCALLBACK()
{
    // usart_printf("111111111\r\n");
    memcpy(debug_rx_buf_, rx_buf_, DEBUG_RVSIZE);
    //解包预处理
    //将串口收到的数据进行处理，新的数组以数字开头，便于之后字符转浮点数的运算
    memcpy(debug_buf_, &debug_rx_buf_[5], 10);
    uint8_t data_length = DEBUG_RVSIZE - __HAL_DMA_GET_COUNTER(which_uart_->hdmarx);   //计算接收到的数据长度
    //速度环pid参数设置
    switch (debug_rx_buf_[0])
    {
        case VEL_LOOP:
        {
            switch (debug_rx_buf_[3])
            {
                case VEL_KP:
                    Debug_Param.vel_kp = strtof(debug_buf_, &p_end_);
                    break;
                case VEL_KI:
                    Debug_Param.vel_ki = strtof(debug_buf_, &p_end_);
                    break;
                case VEL_KF:
                    Debug_Param.vel_kf = strtof(debug_buf_, &p_end_);
                    break;
                case VEL_RAMPSTEP:
                    Debug_Param.vel_rampstep = strtof(debug_buf_, &p_end_);
                    break;
                case VEL_TARVALUE:
                    Debug_Param.vel_tarvalue = strtof(debug_buf_, &p_end_);
                break;
            }
            break;
        }
        case POS_LOOP:
        {
            switch (debug_rx_buf_[3])
            {
                case POS_KP:
                    Debug_Param.pos_kp = strtof(debug_buf_, &p_end_);
                    break;
                case POS_KD:
                    Debug_Param.pos_kd = strtof(debug_buf_, &p_end_);
                    break;
                case POS_KF:
                    Debug_Param.pos_kf = strtof(debug_buf_, &p_end_);
                break;
                case POS_MAXSTEP:
                    Debug_Param.pos_maxstep = strtof(debug_buf_, &p_end_);
                    break;
                case POS_TARVALUE:
                    Debug_Param.pos_tarvalue = strtof(debug_buf_, &p_end_);
                    break;
            }
            break;
        }
    }
    memset(debug_rx_buf_, 0, data_length);                                            //清零接收缓冲区
    data_length = 0;

}
