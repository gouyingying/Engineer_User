//
// Created by ZONE7 on 2024/9/24.
//

#include "usartio.hpp"
// #include "remotec.hpp"
#include "debugc.hpp"
#include "Self_Control.hpp"
#include "image_referee.hpp"
#include "receive.hpp"
#include "remotec.hpp"
#include "witimu.hpp"

void USARTIO_Init()
{
    debug.open();
    wit.open();
    // self_control1.open();
    // self_control2.open();
    Image_Referee.open();
    // receive_from_self_control.open();
    remote.open();
}

/**
 * @brief DMA空闲中断回调函数
 * @param huart
 * @param Size
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    debug.rxCallBack(huart);
    wit.rxCallBack(huart);
    Image_Referee.rxCallBack(huart);
    // receive_from_self_control.rxCallBack(huart);
    // self_control1.rxCallBack(huart);
    // self_control2.rxCallBack(huart);
}

/**
 * @brief 中断回调函数
 * @param huart
 * @param Size
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    remote.rxCallBack(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
}

cUSARTC::cUSARTC(UART_HandleTypeDef *uart, uint16_t size, eUSART_type type): which_uart_(uart), bufsize_(size),
                                                                             type_(type)
{
}

void cUSARTC::open() //串口开启函数
{
    if (type_ == DMA_CPLT_IT) HAL_UART_Receive_DMA(which_uart_, rx_buf_, bufsize_);
    else if (type_ == DMA_IDLE_IT) HAL_UARTEx_ReceiveToIdle_DMA(which_uart_, rx_buf_, bufsize_);
}

void cUSARTC::close() //串口关闭函数
{
    HAL_UART_DMAStop(which_uart_);
}

void cUSARTC::rxCallBack(UART_HandleTypeDef *uart)
{
    if (uart == which_uart_)
    {
        close();
        rxUserCALLBACK();
        open();
    }
}

void cUSARTC::txCallBack(UART_HandleTypeDef *uart)
{
    if (uart == which_uart_)
    {
        txUserCALLBACK();
    }
}
