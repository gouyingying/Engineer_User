//
// Created by 24481 on 24-10-8.
//

#include "chassisc.hpp"
#include "send.hpp"
#include "remotec.hpp"
#include "DM4310.hpp"
#include "end_act.hpp"
//
cChassisC chassis_com;

// SEND_CHASSIS_MESSAGE send_message;
float angle1, angle2, angle3, angle4, angle5, angle6;
int pump_arm,pump_save;

void cChassisC::Init()
{
    chassis_yaw_.Init(10.0f, 0.1f, 0.0f, 0.0f, 15.0f * PI, 300.0f);

}

/**
  *@breif   车体模式切换：小陀螺、随动、自瞄
  *@param   none
  *@retval  当前的车体模式
  */
void cChassisC::SetCarMode()
{
    static float portion = TIMpiece;
    static float tar_pos = 0;
    static float _tar_pos = 0;

    if (portSetProtect())
    {
        CarMode_ = PROTECT;
    }
    else
    {
        switch (CtrlMode)
        {
        case eRC:
            switch (remote.Get_Remote().rc.s[0]) //右侧拨杆
            {
        case 1: //上拨
            CarMode_ = FREE;
            break;
        case 3: //拨到中间
            CarMode_ = FOLLOW;
            break;
        case 2: //下拨

            break;
        default:
            break;
            }
            break;
        case eKey:

            // if (rc_ctrl.key.F.Is_Click_Once)chassis_com.CarMode_ = cChassisC::SPIN;
                // else chassis_com.CarMode_ = cChassisC::FOLLOW;
                    break;
        default:
            break;
        }
        // vz= static_cast<float>(-remote.Get_Remote().rc.ch[0]) / 6.6f * 2.0f;
    }
    //

    switch (CarMode_)
    {
        case FOLLOW:
            {

                // ChassisYawTarget = (DM_yaw.Get_Control_Angle()-2.23) * 24.0f;       // ///优弧劣弧处理
                if (ChassisYawTarget - ((DM_yaw.Get_Now_Angle() + DM_yaw_offset) * 24.0f) > 180)
                    ChassisYawTarget -= 360; //加减2π
                if (((DM_yaw.Get_Now_Angle()-2.23) * 24.0f) - ChassisYawTarget > 180)
                    ChassisYawTarget += 360;

                if (TuoluoDiredtion - ((DM_yaw.Get_Now_Angle()+ DM_yaw_offset) * 24.0f) > 360)
                    TuoluoDiredtion -= 360;
                if (((DM_yaw.Get_Now_Angle()-2.23) * 24.0f) - TuoluoDiredtion > 360)
                    TuoluoDiredtion += 360;

                //
                chassis_yaw_.Set_Target(ChassisYawTarget);
                chassis_yaw_.Set_Now((DM_yaw.Get_Now_Angle() + DM_yaw_offset) * 24.0f);
                chassis_yaw_.TIM_Adjust_PeriodElapsedCallback();
                vz = - chassis_yaw_.Get_Out();
                if(pump_save_flag == true)
                {
                    vz = 0;
                }
                // if(vz<50 && vz>-50)
                // {
                //     vz = 0;
                // }
                // usart_printf("%.2f,%.2f,%.2f,%.2f\r\n",ChassisYawTarget,((DM_yaw.Get_Now_Angle()-2.23) * 24.0f),((DM_yaw.Get_Control_Angle()-2.23) * 24.0f),chassis_com.vz);
            }
            break;

        case FREE:
            {
                switch (CtrlMode)
                {
                    case eRC:
                        vz= static_cast<float>(-remote.Get_Remote().rc.ch[0]) / 6.6f * 2.0f;
                        break;
                    case image:
                        vz=static_cast<float>(-k_vz) / 6.6f * 2.0f;;
                        break;
                }
            }
            break;

        case PROTECT:
            vz = 0.0f;
            break;
        default:
            break;

    Last_CarMode_ = CarMode_;
    }
}

void cChassisC::protect()
{
    CarMode_ = PROTECT;
    // send_to_chassis(0, 0, 0, CarMode_);
    CAN_ChasisSendSpd(0, 0, 0, CarMode_, 0);
}

void CAN_ChasisSendSpd(int16_t vx, int16_t vy, int16_t vz, int8_t car_mode, int8_t is_aimbot)
{
    static CAN_TxHeaderTypeDef CANx_tx_message; //定义一个CAN发送报文头
    static uint8_t CANx_send_data[8]; //定义一个数据数组，用于存放发送的数据
    uint32_t send_mail_box; //定义一个变量用于存储发送邮箱编号
    CANx_tx_message.StdId = 0x401; //标识符，形参数据存入发送的数据包
    CANx_tx_message.IDE = CAN_ID_STD; //标识符选择位，STD-标准帧
    CANx_tx_message.RTR = CAN_RTR_DATA; //定义帧类型
    CANx_tx_message.DLC = 0x08; //数据帧长度为8位
    CANx_send_data[0] = vx >> 8; //依次将要发送的数据移入数据数组，下同
    CANx_send_data[1] = vx & 0xFF;
    CANx_send_data[2] = vy >> 8;
    CANx_send_data[3] = vy & 0xFF;
    CANx_send_data[4] = vz >> 8;
    CANx_send_data[5] = vz & 0xFF;
    CANx_send_data[6] = car_mode;
    CANx_send_data[7] = is_aimbot;
    HAL_CAN_AddTxMessage(&hcan2,
                         &CANx_tx_message, //hal库can发送函数：该函数用于向发送邮箱
                         CANx_send_data, &send_mail_box); //添加发送报文，并激活发送请求

}

/**
  *@breif   CAN发送云台的信息给底盘
  *@param
  *@retval  None
  */
void CAN_ChasisSendMsg(int16_t yaw, int16_t pitch, int8_t pump_arm, int8_t pump_save, int8_t rammer_status,
                       int8_t redraw_status)
{
    CAN_TxHeaderTypeDef tx_msg;
    uint32_t send_mail_box = 2;
    uint8_t send_data[8];
    tx_msg.StdId = 0x402;
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = 0x08;
    send_data[0] = (yaw >> 8);
    send_data[1] = yaw & 0xff;
    send_data[2] = (pitch >> 8);
    send_data[3] = pitch & 0xff;
    send_data[4] = pump_arm;
    send_data[5] = pump_save;
    send_data[6] = rammer_status;
    send_data[7] = redraw_status;
    HAL_CAN_AddTxMessage(&hcan2, &tx_msg, send_data, &send_mail_box);
}

void cChassisC::ComLoop()
{



}

void cChassisC::rx_callback(uint8_t *Rx_Data)
{
    // memcpy(&receive_buf1,&Image_referee_recv_mesg.self_control_all_data.Data[0],4);
    // memcpy(&receive_buf2,&Image_referee_recv_mesg.self_control_all_data.Data[4],4);
    // memcpy(&receive_buf3,&Image_referee_recv_mesg.self_control_all_data.Data[8],4);
    // memcpy(&receive_buf4,&Image_referee_recv_mesg.self_control_all_data.Data[12],4);
    // memcpy(&receive_buf5,&Image_referee_recv_mesg.self_control_all_data.Data[16],4);
    // memcpy(&receive_buf6,&Image_referee_recv_mesg.self_control_all_data.Data[20],4);
}


void cChassisC::getCtrlData()
{
    flag++;
    if (flag == 5)
    {
        flag = 0;
    }
    vx = portSetVx();
    vy = portSetVy();
    SetCarMode(); //得到vz和CarMode的值
    if(image_ctrl.key.G.Now_State)chassis_com.redraw_status = 1;
    else chassis_com.redraw_status = 0;

    // yaw_offset = gimbal.yaw.getAngle() - TuoluoDiredtion;
}
void cChassisC::canSend()
{
    if (flag == 1)
    {
        CAN_ChasisSendSpd(vx, vy, vz, CarMode_, 0); //0表示自瞄关,1表示自瞄开
        // send_to_chassis(vx, vy, vz, CarMode_);
    }
    if (flag == 3)
        CAN_ChasisSendMsg(-yaw_offset, 0, pump_arm, pump_save, 0, chassis_com.redraw_status);
}

void angle_1_2_rx_callback(uint8_t *Rx_Data)
{
    memcpy(&angle1,&Rx_Data[0],4);
    memcpy(&angle2,&Rx_Data[4],4);
}

void angle_3_4_rx_callback(uint8_t *Rx_Data)
{
    memcpy(&angle3,&Rx_Data[0],4);
    memcpy(&angle4,&Rx_Data[4],4);
}

void angle_5_6_rx_callback(uint8_t *Rx_Data)
{
    memcpy(&angle5,&Rx_Data[0],4);
    memcpy(&angle6,&Rx_Data[4],4);
}

// void send_chassis_init() {
//     send_message.Head1 = 0x04;
//     send_message.Head2 = 0x02;
//     send_message.Len = 8;
// }