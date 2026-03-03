//
// Created by ZONE7 on 2024/9/24.
//
#include "main.h"
#include "gpio.h"
#include "dma.h"
#include "can.h"
#include "usart.h"
#include <cstring>
#include "usartio.hpp"

void BSP_Init(void);



int main()
{
    BSP_Init();
    User_Init();
    while(1)
    {

    }
}


void BSP_Init(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_DMA_Init();
    MX_UART4_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();


    USARTIO_Init();
}