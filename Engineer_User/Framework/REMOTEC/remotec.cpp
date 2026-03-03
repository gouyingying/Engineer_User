//
// Created by ZONE7 on 2024/9/28.
//

#include "remotec.hpp"

#include "usart.h"
#include "motorc.hpp"
Image_ctrl_t image_ctrl;
self_control_t self_ctrl;
C5_Online_detect_t C5_GetNewData;
Online_detect_t RC_GetNewData;
eCtrlMode CtrlMode = eRC;
float k_vx = 0.0f;
float k_vy = 0.0f;
float k_vz = 0.0f;
float K_angle = 0.0f;
float K_speed = 1.0f;

void SbusToRc(const uint8_t *sbus_buff, sRemoteInfo *rc_ctrl)
{
    if (sbus_buff == nullptr || rc_ctrl == nullptr)
    {
        return;
    }
    RC_GetNewData.live_time = 100;
    rc_ctrl->rc.ch[0] = ((int16_t)sbus_buff[0] | ((int16_t)sbus_buff[1] << 8)) & 0x07FF;        //!< Channel 0  right horizontal
    rc_ctrl->rc.ch[1] = (((int16_t)sbus_buff[1] >> 3) | ((int16_t)sbus_buff[2] << 5)) & 0x07FF; //!< Channel 1  right vertical
    rc_ctrl->rc.ch[2] = (((int16_t)sbus_buff[2] >> 6) | ((int16_t)sbus_buff[3] << 2) |
                         ((int16_t)sbus_buff[4] << 10)) & 0x07FF;                                //!< Channel 2  left horizontal
    rc_ctrl->rc.ch[3] = (((int16_t)sbus_buff[4] >> 1) | ((int16_t)sbus_buff[5] << 7)) & 0x07FF; //!< Channel 3  left vertical
    rc_ctrl->rc.s[0] = ((sbus_buff[5] >> 4) & 0x0003);                                          //!< Switch right
    rc_ctrl->rc.s[1] = ((sbus_buff[5] >> 4) & 0x000C) >> 2;                                     //!< Switch left
    rc_ctrl->mouse.x = ((int16_t)sbus_buff[6]) | ((int16_t)sbus_buff[7] << 8);                  //!< Mouse X axis
    rc_ctrl->mouse.y = ((int16_t)sbus_buff[8]) | ((int16_t)sbus_buff[9] << 8);                  //!< Mouse Y axis
    rc_ctrl->mouse.z = ((int16_t)sbus_buff[10]) | ((int16_t)sbus_buff[11] << 8);                //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buff[12];                                                     //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buff[13];                                                     //!< Mouse Right Is Press ?
    rc_ctrl->key.v = ((int16_t)sbus_buff[14]) | ((int16_t)sbus_buff[15] << 8);                  //!< KeyBoard value
    rc_ctrl->rc.ch[4] = ((int16_t)sbus_buff[16]) | ((int16_t)sbus_buff[17] << 8);               //NULL
    // usart_printf("%d, %d, %d, %d, %d, %d\r\n", rc_ctrl->rc.ch[0], rc_ctrl->rc.ch[1], rc_ctrl->rc.ch[2], rc_ctrl->rc.ch[3], rc_ctrl->rc.s[0], rc_ctrl->rc.s[1]);
    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
    // usart_printf("%d, %d, %d, %d, %d, %d\r\n", rc_ctrl->rc.ch[0], rc_ctrl->rc.ch[1], rc_ctrl->rc.ch[2], rc_ctrl->rc.ch[3], rc_ctrl->rc.s[0], rc_ctrl->rc.s[1]);
}

cREMOTEC remote{&huart2,RC_FRAME_LENGTH,DMA_CPLT_IT};

void cREMOTEC::rxUserCALLBACK()
{
    SbusToRc(rx_buf_,&rc_ctrl);
    ///ch[2]为x方向速度、ch[3]为y方向速度、ch[0]为z轴转速度
    // chassis.rcUpdateVel(rc_ctrl.rc.ch[2]/6.6f*2.0f,  rc_ctrl.rc.ch[3]/6.6f*2.0f,   -rc_ctrl.rc.ch[0]/6.6f*2.0f,   rc_ctrl.rc.s[0]);
    // usart_printf("%d, %d, %d,%d, %d, %d,%.2f\r\n", rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[1], rc_ctrl.rc.ch[2], rc_ctrl.rc.ch[3], rc_ctrl.rc.s[0], rc_ctrl.rc.s[1],remote.Get_Remote().rc.ch[0]/660.0f*200.f);
}

sRemoteInfo cREMOTEC::Get_Remote()
{
    return rc_ctrl;
}

/**********************************RC_Ctrl**********************************************/

/**
 * 保护检测 如果遥控器指定保护或遥控器断连
 * @return Car.is_protect true
 */
bool portSetProtect()
{
    bool is_protect;
    RC_GetNewData.live_time--;
    if (RC_GetNewData.live_time <= 0)
        is_protect = true;
    else is_protect = false;
    return is_protect;
}

/**
  *@breif   设置Yaw轴移动值
  *@param   目标值
  *@retval  Yaw轴移动值yaw_speed
  */

float portSetVx(void)
{
    static float target_vx = 0;

    switch (CtrlMode)
    {
    case eRC:
        target_vx = static_cast<float>(remote.Get_Remote().rc.ch[2]) * 200.0f / 660.0f;
        break;
    case eKey:
        {

        }
        break;
    case image:
        target_vx = k_vx * 200.0f * K_speed;
        break;
    default:
        target_vx = 0.0f;
        break;
    }
    return target_vx;
}

float portSetVy(void)
{
    static float target_vy = 0;
    switch (CtrlMode)
    {
    case eRC:
        target_vy = static_cast<float>(remote.Get_Remote().rc.ch[3]) * 200.0f / 660.0f;
        break;
    case eKey:

        break;
    case image:
        target_vy = k_vy * 200.0f * K_speed;
        break;
    default:
        target_vy = 0.0f;
        break;
    }
    return target_vy;
}

float portSetAngle(void)
{
    static float portion = TIMpiece;
    static float tar_pos = 0;
    static float _tar_pos = 0;
    if(portSetProtect())
    {
        tar_pos = 0;
        _tar_pos = 0;
    }
    switch (CtrlMode)
    {
        case eRC:
            _tar_pos += static_cast<float>(remote.Get_Remote().rc.ch[1]) * portion * 200.0f / 660.0f;
            // LIMIT_VALUE(_tar_pos, -60, 20);
            tar_pos=_tar_pos;
            break;
        case eKey:
            break;
        case image:
            _tar_pos += static_cast<float>(image_ctrl.mouse.mouse_y) * portion * 200.0f / 660.0f;
            // LIMIT_VALUE(_tar_pos, -60, 20);
            tar_pos=_tar_pos;
            break;
        default:
            break;
    }
    return tar_pos;
}

float portSetAngle_Z(void)
{
    static float portion = TIMpiece;
    static float tar_pos = 0;
    static float _tar_pos = 0;
    if(portSetProtect())
    {
        tar_pos = 0;
        _tar_pos = 0;
    }
    switch (CtrlMode)
    {
    case eRC:
        _tar_pos = static_cast<float>(remote.Get_Remote().rc.ch[0])  * 200.0f / 660.0f;
        LIMIT_VALUE(_tar_pos, -600, 360);
        tar_pos=_tar_pos;
        break;
    case eKey:
        break;
    case image:
        _tar_pos = static_cast<float>(image_ctrl.mouse.mouse_x) * portion * 200.0f / 660.0f;
        tar_pos=_tar_pos;
        break;
    default:
        break;
    }

    return tar_pos;
}

float Limit_Angle(float tar_get_,float min,float max)
{
    tar_get_ = (((tar_get_)<(min)) ? (min) : (((tar_get_)>(max)) ? (max) : (tar_get_)));
    return tar_get_;
}

/**
* 遥控器 获取拨杆控制信息
*/
bool flag=false;
void RC_Levers()
{
    //左拨杆
    if (remote.Get_Remote().rc.s[1] == 2)CtrlMode = image; //左拨杆下
    else CtrlMode = eRC;
    //右拨杆
    switch (CtrlMode)
    {
        case eRC:
            switch (remote.Get_Remote().rc.s[0]) //右侧拨杆
            {
                case 1: //上拨

                    break;
                case 3: //拨到中间
                    break;
                case 2: //下拨

                    break;
                default:
                    break;
            }
            break;
        case eKey:


            break;
        default:
            break;
    }

    switch (CtrlMode)
    {
        case eRC:
            if (remote.Get_Remote().rc.s[1] == 1) //左拨杆上拨
            {

            }
            if (remote.Get_Remote().rc.s[1] == 3) //左拨杆置中

            break;

        case eKey:
        {

        }
            break;
        default:
            break;
    }
}

self_control_t Get_Self_Ctrl()
{
    return self_ctrl;
}