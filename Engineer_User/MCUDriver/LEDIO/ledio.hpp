//
// Created by ZONE7 on 2024/9/24.
//

#ifndef LEDIO_HPP
#define LEDIO_HPP

#include "main.h"

class LEDC {
public:
    LEDC(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState GPIO_State);
    void ON()const;
    void OFF()const;
    void Toggle()const;
protected:
    GPIO_TypeDef* GPIOx_;
    uint16_t GPIO_Pin_;
    GPIO_PinState GPIO_State_;
};

extern LEDC led;

#endif //LEDIO_HPP
