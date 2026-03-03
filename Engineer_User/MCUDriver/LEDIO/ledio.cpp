//
// Created by 24481 on 24-9-10.
//

#include "ledio.hpp"
 LEDC led(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);


LEDC::LEDC(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState GPIO_State):GPIOx_(GPIOx), GPIO_Pin_(GPIO_Pin), GPIO_State_(GPIO_State)
{}
void LEDC::ON()const
{
    HAL_GPIO_WritePin(GPIOx_, GPIO_Pin_, GPIO_PIN_SET);
}
void LEDC::OFF()const
{
    HAL_GPIO_WritePin(GPIOx_, GPIO_Pin_, GPIO_PIN_RESET);
}
void LEDC::Toggle()const
{
    HAL_GPIO_TogglePin(GPIOx_, GPIO_Pin_);
}
