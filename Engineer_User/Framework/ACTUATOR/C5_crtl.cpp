//
// Created by ZONE7 on 2025/5/2.
//

#include "C5_crtl.hpp"

void C5_Loop()
{
    if(image_ctrl.rc.Pause_Key.Is_Click_Once)
    {
        if(image_ctrl.rc.Trigger_Key.Is_Click_Hold)
        {
            pump_save = 1;
        }
        else
        {
            pump_save = 0;
        }
    }
}