//
// Created by ZONE7 on 2025/1/19.
//

#include "canio_basic.hpp"

// void CAN_Filter_Init(CAN_HandleTypeDef *hcan)
// {
//     CAN_FilterTypeDef can_filter;
//     if (hcan1.Instance == CAN1)
//     {
//         //DJImotor CAN1 FIFO0配置
//         can_filter.FilterBank = 0;
//         can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
//         can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
//         can_filter.FilterIdHigh = 0x0200 << 5;
//         can_filter.FilterIdLow = 0x0000;
//         can_filter.FilterMaskIdHigh = 0x07F8 << 5; //Can1的过滤器只接收canid为0x200~0x207
//         can_filter.FilterMaskIdLow = 0x0000;
//         can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//         can_filter.FilterActivation = CAN_FILTER_ENABLE;
//         HAL_CAN_ConfigFilter(hcan, &can_filter);
//     }
//     if (hcan2.Instance == CAN2)
//     {
//         //RefereeSystem CAN2 FIFO0配置
//         can_filter.FilterBank = 14;
//         can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
//         can_filter.FilterScale = CAN_FILTERSCALE_16BIT;
//         // 配置第一个16位过滤器 (0x200 - 0x207)
//         can_filter.FilterIdHigh = 0x200 << 5; // 第一个过滤器ID，左移5位
//         can_filter.FilterMaskIdHigh = 0x7F8 << 5; // 掩码：忽略低3位
//
//         // 配置第二个16位过滤器 (0x400 - 0x407)
//         can_filter.FilterIdLow = 0x400 << 5; // 第二个过滤器ID，左移5位
//         can_filter.FilterMaskIdLow = 0x7F8 << 5; // 掩码：忽略低3位
//         can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//         can_filter.FilterActivation = CAN_FILTER_ENABLE;
//         can_filter.SlaveStartFilterBank = 14;
//         HAL_CAN_ConfigFilter(hcan, &can_filter);
//
//         //DMmotor CAN2 FIFO1配置
//         can_filter.FilterBank = 15;
//         can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
//         can_filter.FilterScale = CAN_FILTERSCALE_16BIT;
//         can_filter.FilterIdHigh = 0x0307 << 5;
//         can_filter.FilterIdLow = 0x0308 << 5;
//         can_filter.FilterMaskIdHigh = 0x0306 << 5;
//         can_filter.FilterMaskIdLow = 0x0308 << 5;
//         can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
//         can_filter.FilterActivation = CAN_FILTER_ENABLE;
//         HAL_CAN_ConfigFilter(hcan, &can_filter);
//     }
//
//     HAL_CAN_Start(hcan);
//     HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
//     HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
// }
