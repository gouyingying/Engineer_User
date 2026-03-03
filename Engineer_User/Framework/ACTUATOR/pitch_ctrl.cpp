//
// Created by ZONE7 on 2025/5/7.
//

#include "pitch_ctrl.hpp"
#include "DM10010.hpp"
#include "remotec.hpp"

void pitch_control()
{
    if(DM_Pitch.Get_Now_Torque()<0.32f && DM_Pitch.Get_Now_Torque()>-0.32f)
    {
        DM_Pitch.Set_Control_Angle(Limit_Angle(self_ctrl.pitch_10010/ 180.0f * 2.0f * PI,-1.09,3.75));
        DM_Pitch.Set_Control_Omega(PI);
    }
    else
    {
        DM_Pitch.Set_Control_Omega(0);
    }
}