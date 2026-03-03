//
// Created by ZONE7 on 2025/5/2.
//
#include "yaw_ctrl.hpp"
#include "chassisc.hpp"
#include "DM4310.hpp"
#include "end_act.hpp"

void yaw_control(void)
{
    switch (chassis_com.CarMode_)
    {
    case chassis_com.FOLLOW:
        {
            DM_yaw.Set_Control_Angle(-portSetAngle_Z() / 200.0f  - 2.23f );
        }
        break;

    case chassis_com.FREE:
        {
            DM_yaw.Set_Control_Angle(Limit_Angle(((Get_Self_Ctrl().yaw_4310 -32)/90.0f * 2.0f * PI),-6.00,5.33));
            // Limit_Angle(((Get_Self_Ctrl().yaw_4310 - 64)/180.0f * 2.0f * PI),-6.00,5.33);
            // switch (CtrlMode)
            // {
            // case eRC:
            //
            //     break;
            // case image:
            //
            //     break;
            // }
        }
        break;
    }

}
