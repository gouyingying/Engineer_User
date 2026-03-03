//
// Created by ZONE7 on 2024/9/24.
//

#ifndef USARTIO_HPP
#define USARTIO_HPP

#include "main.h"
#include "usart.h"
#include <cstring>

typedef enum
{
    DMA_IDLE_IT = 0,
    DMA_CPLT_IT = 1
}eUSART_type;

class cUSARTC{
public:
    cUSARTC(UART_HandleTypeDef *uart,uint16_t size,eUSART_type type);

    virtual void open();
    void close();
    void rxCallBack(UART_HandleTypeDef *uart);
    void txCallBack(UART_HandleTypeDef *uart);

    virtual void rxUserCALLBACK(void){};
    virtual void txUserCALLBACK(void){};
    UART_HandleTypeDef *which_uart_;
protected:
    eUSART_type type_;
    uint16_t bufsize_;
    uint8_t rx_buf_[255];
};

void USARTIO_Init(void);


#endif //USARTIO_HPP
