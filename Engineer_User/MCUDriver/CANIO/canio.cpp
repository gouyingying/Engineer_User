//
// Created by ZONE7 on 2024/9/24.
//

#include "canio.hpp"

#include "cmsis_os.h"
#include "main.h"
#include "usart.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Struct_CAN_Manage_Object CAN1_Manage_Object = {0};
Struct_CAN_Manage_Object CAN2_Manage_Object = {0};

// CAN通信发送缓冲区
uint8_t CAN1_0x1ff_Tx_Data[8];
uint8_t CAN1_0x200_Tx_Data[8];
uint8_t CAN1_0x2ff_Tx_Data[8];
uint8_t CAN1_0x3fe_Tx_Data[8];
uint8_t CAN1_0x4fe_Tx_Data[8];

uint8_t CAN2_0x1ff_Tx_Data[8];
uint8_t CAN2_0x200_Tx_Data[8];
uint8_t CAN2_0x2ff_Tx_Data[8];
uint8_t CAN2_0x3fe_Tx_Data[8];
uint8_t CAN2_0x4fe_Tx_Data[8];

uint8_t CAN_Supercap_Tx_Data[8];

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化CAN总线
 *
 * @param hcan CAN编号
 * @param Callback_Function 处理回调函数
 */
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function)
{
    HAL_CAN_Start(hcan);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

    if (hcan->Instance == CAN1)
    {
        CAN1_Manage_Object.CAN_Handler = hcan;
        CAN1_Manage_Object.Callback_Function = Callback_Function;
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(1) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }
    else if (hcan->Instance == CAN2)
    {
        CAN2_Manage_Object.CAN_Handler = hcan;
        CAN2_Manage_Object.Callback_Function = Callback_Function;
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(14) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(15) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }
}

/**
 * @brief 配置CAN的过滤器
 *
 * @param hcan CAN编号
 * @param Object_Para 编号 | FIFOx | ID类型 | 帧类型
 * @param ID ID
 * @param Mask_ID 屏蔽位(0x3ff, 0x1fffffff)
 */
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
    CAN_FilterTypeDef can_filter_init_structure;

    //检测传参是否正确
    assert_param(hcan != NULL);

    if ((Object_Para & 0x02))
    {
        //数据帧
        //掩码后ID的高16bit
        can_filter_init_structure.FilterIdHigh = ID << 3 << 16;
        //掩码后ID的低16bit
        can_filter_init_structure.FilterIdLow = ID << 3 | ((Object_Para & 0x03) << 1);
        // ID掩码值高16bit
        can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 3 << 16;
        // ID掩码值低16bit
        can_filter_init_structure.FilterMaskIdLow = Mask_ID << 3 | ((Object_Para & 0x03) << 1);
    }
    else
    {
        //其他帧
        //掩码后ID的高16bit
        can_filter_init_structure.FilterIdHigh = ID << 5;
        //掩码后ID的低16bit
        can_filter_init_structure.FilterIdLow = ((Object_Para & 0x03) << 1);
        // ID掩码值高16bit
        can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 5;
        // ID掩码值低16bit
        can_filter_init_structure.FilterMaskIdLow = ((Object_Para & 0x03) << 1);
    }
    //滤波器序号, 0-27, 共28个滤波器, 前14个在CAN1, 后14个在CAN2
    can_filter_init_structure.FilterBank = Object_Para >> 3;
    //滤波器绑定FIFO0
    can_filter_init_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;
    //使能滤波器
    can_filter_init_structure.FilterActivation = ENABLE;
    //滤波器模式，设置ID掩码模式
    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    // 32位滤波
    can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
    //从机模式选择开始单元
    can_filter_init_structure.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);
}

/**
 * @brief 发送数据帧
 *
 * @param hcan CAN编号
 * @param ID ID
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    //检测传参是否正确
    assert_param(hcan != NULL);

    tx_header.StdId = ID;
    tx_header.ExtId = 0;
    tx_header.IDE = 0;
    tx_header.RTR = 0;
    tx_header.DLC = Length;

    return (HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox));
}

/**
 * @brief CAN的TIM定时器中断发送回调函数
 *
 */
void TIM_CAN_PeriodElapsedCallback()
{
    static int mod10 = 0;

    mod10++;

    // CAN1电机
    CAN_Send_Data(&hcan1, 0x1ff, CAN1_0x1ff_Tx_Data, 8);
    osDelay(1);
    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
    osDelay(1);
    CAN_Send_Data(&hcan1, 0x2ff, CAN1_0x2ff_Tx_Data, 8);
    osDelay(1);
    CAN_Send_Data(&hcan1, 0x3fe, CAN1_0x3fe_Tx_Data, 8);
    osDelay(1);

    // CAN2电机
    CAN_Send_Data(&hcan2, 0x1ff, CAN2_0x1ff_Tx_Data, 8);
    osDelay(1);
    CAN_Send_Data(&hcan2, 0x200, CAN2_0x200_Tx_Data, 8);
    osDelay(1);
    CAN_Send_Data(&hcan2, 0x2ff, CAN2_0x2ff_Tx_Data, 8);
    osDelay(1);

}

/**
 * @brief HAL库CAN接收FIFO0中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    //选择回调函数
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CAN1_Manage_Object.Rx_Buffer.Header, CAN1_Manage_Object.Rx_Buffer.Data);
        CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CAN2_Manage_Object.Rx_Buffer.Header, CAN2_Manage_Object.Rx_Buffer.Data);
        CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
    }
}

/**
 * @brief HAL库CAN接收FIFO1中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    //选择回调函数
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &CAN1_Manage_Object.Rx_Buffer.Header, CAN1_Manage_Object.Rx_Buffer.Data);
        CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &CAN2_Manage_Object.Rx_Buffer.Header, CAN2_Manage_Object.Rx_Buffer.Data);
        CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
    }
}


// // CAN底盘发送函数
// /**
//   *@breif   CAN发送底盘速度信息给云台
//   *@param
//   *@retval  None
//   */
// void CAN_ChasisSendSpd(int16_t vx, int16_t vy, int16_t vz, int8_t car_mode, int8_t is_aimbot)
// {
//     static CAN_TxHeaderTypeDef CANx_tx_message; //定义一个CAN发送报文头
//     static uint8_t CANx_send_data[8]; //定义一个数据数组，用于存放发送的数据
//     uint32_t send_mail_box; //定义一个变量用于存储发送邮箱编号
//     CANx_tx_message.StdId = 0x401; //标识符，形参数据存入发送的数据包
//     CANx_tx_message.IDE = CAN_ID_STD; //标识符选择位，STD-标准帧
//     CANx_tx_message.RTR = CAN_RTR_DATA; //定义帧类型
//     CANx_tx_message.DLC = 0x08; //数据帧长度为8位
//     CANx_send_data[0] = vx >> 8; //依次将要发送的数据移入数据数组，下同
//     CANx_send_data[1] = vx & 0xFF;
//     CANx_send_data[2] = vy >> 8;
//     CANx_send_data[3] = vy & 0xFF;
//     CANx_send_data[4] = vz >> 8;
//     CANx_send_data[5] = vz & 0xFF;
//     CANx_send_data[6] = car_mode;
//     CANx_send_data[7] = is_aimbot;
//     HAL_CAN_AddTxMessage(&hcan2,
//                          &CANx_tx_message, //hal库can发送函数：该函数用于向发送邮箱
//                          CANx_send_data, &send_mail_box); //添加发送报文，并激活发送请求
// }
//
// /**
//   *@breif   CAN发送云台的信息给底盘
//   *@param
//   *@retval  None
//   */
// void CAN_ChasisSendMsg(int16_t yaw, int16_t pitch, int8_t servo_status, int8_t power_status, int8_t rammer_status,
//                        int8_t redraw_status)
// {
//     CAN_TxHeaderTypeDef tx_msg;
//     uint32_t send_mail_box = 2;
//     uint8_t send_data[8];
//     tx_msg.StdId = 0x402;
//     tx_msg.IDE = CAN_ID_STD;
//     tx_msg.RTR = CAN_RTR_DATA;
//     tx_msg.DLC = 0x08;
//     send_data[0] = (yaw >> 8);
//     send_data[1] = yaw & 0xff;
//     send_data[2] = (pitch >> 8);
//     send_data[3] = pitch & 0xff;
//     send_data[4] = servo_status;
//     send_data[5] = power_status;
//     send_data[6] = rammer_status;
//     send_data[7] = redraw_status;
//     HAL_CAN_AddTxMessage(&hcan2, &tx_msg, send_data, &send_mail_box);
// }