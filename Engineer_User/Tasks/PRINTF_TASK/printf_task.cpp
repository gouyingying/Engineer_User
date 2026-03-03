//
// Created by ZONE7 on 2024/9/24.
//

#include "printf_task.hpp"
#include "cmsis_os.h"
#include "receive.hpp"
#include "ledio.hpp"
#include "send.hpp"
#include "gimbal.hpp"
#include "chassisc.hpp"
#include "witimu.hpp"

void printf_task(void const * argument)
{
    TickType_t current;

    send_init();
    /* USER CODE BEGIN GimbalTask */
    /* Infinite loop */
    for(;;)
    {
        current = xTaskGetTickCount();
     // if (portSetProtect())
     //     {
     //     chassis_com.protect();
     //     // led.ON();
     //          }
     //     else
     //     {
     //
     //         chassis_data_update();
     //
     //         led.Toggle();
     //     }
        vTaskDelayUntil(&current, 5 / portTICK_RATE_MS);
    }
    /* USER CODE END GimbalTask */
}
