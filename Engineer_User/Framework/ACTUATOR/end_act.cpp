//
// Created by ZONE7 on 2025/5/2.
//

#include "end_act.hpp"

bool pump_save_flag = false;
int pump_save_state = 0;
bool pump_get_flag = false;
int pump_get_state = 0;
bool gold_miner_flag = false;
bool gold_miner_mid_flag = false;
bool silver_miner_flag = false;
float yaw_val;

end_act end_pitch;

void end_act::Init()
{
    end_pitch_pid.Init(10.0f, 0.1f, 0.0f, 0.0f, 15.0f * PI, 300.0f);
}

void end_act::SetEndMode()
{
    if(image_ctrl.rc.Middle_Key == 0)
    {
        End_Pitch_Mode_ = GOLD;
    }
    else if(image_ctrl.rc.Middle_Key == 2 )
    {
        End_Pitch_Mode_ = SILVER;
    }
    else
    {
        End_Pitch_Mode_ = FREE;
    }
    // usart_printf("%d\r\n",End_Pitch_Mode_);
    CrtlLoop();
}

void end_act::CrtlLoop()
{
    static float gold_tag_val;
    static float silver_tag_val;
    switch (End_Pitch_Mode_)
    {
    case FREE:
        // DM_4310_pitch.Set_Control_Angle(self_ctrl.pitch_4310/360.0f * 2.0
            End_Pitch_Control();
        break;
    case GOLD:
        // End_Pitch_Target = -static_cast<float>(wit.WIT_IMU_Angle(imu_pitch)) - pitch_offset_g;
            // DM_4310_pitch.Set_Control_Angle( End_Pitch_Target /360.0f * 2.0f * PI);
                // usart_printf("%.2f,%.2f,%.2f,%.2f\r\n",wit.WIT_IMU_Angle(imu_pitch),End_Pitch_Target,DM_4310_pitch.Get_Control_Angle(),DM_4310_pitch.Get_Now_Angle());
                    if(wit.WIT_IMU_Angle(imu_pitch)-pitch_offset_g > 4.0f)
                    {
                        gold_tag_val += 0.01;
                    }
                    else if(wit.WIT_IMU_Angle(imu_pitch)-pitch_offset_g > 0.05f)
                    {
                        gold_tag_val += 0.0001;
                    }
                    else if(wit.WIT_IMU_Angle(imu_pitch)-pitch_offset_g < -4.0f)
                    {
                        gold_tag_val -= 0.01;
                    }
                    else if(wit.WIT_IMU_Angle(imu_pitch)-pitch_offset_g < -0.05f)
                    {
                        gold_tag_val -= 0.0001;
                    }
                    DM_4310_pitch.Set_Control_Angle(LIMIT_VALUE(gold_tag_val,-1.5,1.2));
                    break;
        case SILVER:
            // End_Pitch_Target += static_cast<float>(wit.WIT_IMU_Angle(imu_pitch)) - pitch_offset_s;
            // DM_4310_pitch.Set_Control_Angle( End_Pitch_Target /360.0f * 2.0f * PI);
                    if(wit.WIT_IMU_Angle(imu_pitch)-pitch_offset_s > 4.0f)
                    {
                        silver_tag_val += 0.01;
                    }
                    else if(wit.WIT_IMU_Angle(imu_pitch)-pitch_offset_s > 0.05f)
                    {
                        silver_tag_val += 0.0001;
                    }
                    else if(wit.WIT_IMU_Angle(imu_pitch)-pitch_offset_s < -4.0f)
                    {
                        silver_tag_val -= 0.01;
                    }
                    else if(wit.WIT_IMU_Angle(imu_pitch)-pitch_offset_s < -0.05f)
                    {
                        silver_tag_val -= 0.0001;
                    }
                    DM_4310_pitch.Set_Control_Angle(LIMIT_VALUE(silver_tag_val,-1.5,1.2));
                    break;
    }
}

void Pump_Save()
{
    static float yaw_val = DM_yaw_offset;
    static float roll_val = -62.0 + roll_3508.pos_init;
    static int save_times;
    bool yaw_set;
    if (image_ctrl.key.V.Is_Click_Once == 1)//换按键
    {
        pump_save_flag = true;
        pitch_3508.Set_Target_Angle(110.0f + pitch_3508.pos_init);
        pump_save_state = 1;
        if (pitch_3508.Get_Now_Angle() >= (100.0f + pitch_3508.pos_init) && pump_save_state==1)
        {
            roll_3508.Set_Target_Angle(-62.0f + roll_3508.pos_init);
            pump_save_state =2;
            if (roll_3508.Get_Now_Angle() <= (-58.0f+ roll_3508.pos_init) && pump_save_state==2)
            {
                DM_4310_pitch.Set_Control_Angle(-1.4f);
                pump_save_state =3;
                if (DM_4310_pitch.Get_Now_Angle() < -1.3 && pump_save_state ==3)
                {
                    pitch_3508.Set_Target_Angle(150.0f + pitch_3508.pos_init);
                    pump_save_state =4;
                    if (pitch_3508.Get_Now_Angle()>(140.0f + pitch_3508.pos_init)   && pump_save_state == 4)
                    {
                        if(DM_yaw.Get_Now_Angle()<3.45 && yaw_set == false)
                        yaw_val += 0.03;
                        DM_yaw.Set_Control_Angle(yaw_val);
                        pump_save_state =5;
                        if (DM_yaw.Get_Now_Angle() > 3.45f &&pump_save_state ==5)
                        {
                            // yaw_val = 3.48;
                            yaw_set = true;

                                yaw_val += image_ctrl.mouse.mouse_z /2000.f;
                                if(image_ctrl.mouse.middle_button_down.Is_Click_Once == 1)
                                {
                                    save_times++;
                                    DM_4310_pitch.Set_Control_Angle(-1.5f);
                                    DM_Pitch.Set_Control_Angle(0.89f);
                                    pump_arm = 1;
                                    pump_save_state =6;
                                    if (DM_Pitch.Get_Control_Angle() > 0.85&&pump_save_state ==6)
                                    {
                                    pump_save = 0;
                                    // usart_printf("%d\r\n",save_times);
                                    if ((pump_save == 0 && pump_save_state ==7)|| save_times >100)
                                    {
                                        pump_save_state =7;
                                        pitch_3508.Set_Target_Angle(250.0f+ pitch_3508.pos_init);
                                        DM_Pitch.Set_Control_Angle(1.0f);
                                        usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", DM_yaw.Get_Now_Angle(), DM_Pitch.Get_Now_Angle(),pitch_3508.Get_Now_Angle(),roll_3508.Get_Now_Angle(),DM_4310_pitch.Get_Now_Angle(),roll_2006.Get_Now_Angle());

                                        if(pitch_3508.Get_Target_Angle()>(200.0f + pitch_3508.pos_init) && DM_Pitch.Get_Control_Angle()>0.97  && pump_save_state ==7)
                                        {
                                            pump_save_state = 8;
                                        }
                                        }
                                    }

                                }
                        }
                    }
                }
            }

        }
    }
    else if(image_ctrl.key.V.Is_Click_Once == 0  && pump_save_flag ==true)
    {

        DM_yaw.Set_Control_Angle(yaw_val);
        yaw_val -= 0.03;
        if(DM_yaw.Get_Now_Angle() <= DM_yaw_offset )
        {
            yaw_val = DM_yaw_offset;
            roll_val += 1;
            roll_3508.Set_Target_Angle(roll_val+ roll_3508.pos_init);
            if(roll_3508.Get_Now_Angle()>-3.0f + roll_3508.pos_init)
            {
                roll_val = 0;
                pitch_3508.Set_Target_Angle(10.0f + pitch_3508.pos_init);
                DM_Pitch.Set_Control_Angle(0.0f);
                DM_4310_pitch.Set_Control_Angle(0.0f);
                if(DM_Pitch.Get_Now_Angle()<0.03)
                {
                    save_times = 0;
                    image_ctrl.mouse.middle_button_down.Is_Click_Once = 0;
                    pump_save_flag = false;
                    yaw_val = -2.23;
                    roll_val = -62.0;
                }
            }

        }
    }
}

void Pump_Get()
{
    static float yaw_val = DM_yaw_offset;
    static float roll_val = -62.0 + roll_3508.pos_init;
    static int save_times;
    bool yaw_set;
    if (image_ctrl.key.B.Is_Click_Once == 1)//换按键
    {
        pump_get_flag = true;
        pitch_3508.Set_Target_Angle(110.0f + pitch_3508.pos_init);
        pump_get_state = 1;
        if (pitch_3508.Get_Now_Angle() >= (100.0f + pitch_3508.pos_init) && pump_get_state==1)
        {
            roll_3508.Set_Target_Angle(-62.0f + roll_3508.pos_init);
            pump_get_state =2;
            if (roll_3508.Get_Now_Angle() <= (-58.0f+ roll_3508.pos_init) && pump_get_state==2)
            {
                DM_4310_pitch.Set_Control_Angle(-1.4f);
                pump_get_state =3;
                if (DM_4310_pitch.Get_Now_Angle() < -1.3 && pump_get_state ==3)
                {
                    pitch_3508.Set_Target_Angle(150.0f + pitch_3508.pos_init);
                    pump_get_state =4;
                    if (pitch_3508.Get_Now_Angle()>(140.0f + pitch_3508.pos_init)   && pump_get_state == 4)
                    {
                         if(DM_yaw.Get_Now_Angle()<3.45 && yaw_set == false)
                        yaw_val += 0.03;
                        DM_yaw.Set_Control_Angle(yaw_val);
                        pump_save_state =5;
                        if (DM_yaw.Get_Now_Angle() > 3.45f &&pump_save_state ==5)
                        {
                            // yaw_val = 3.48;
                            yaw_set = true;

                                yaw_val += image_ctrl.mouse.mouse_z /2000.f;
                                if(image_ctrl.mouse.middle_button_down.Is_Click_Once == 1)
                                {
                                    save_times++;
                                    DM_4310_pitch.Set_Control_Angle(-1.5f);
                                    DM_Pitch.Set_Control_Angle(0.89f);
                                    pump_save = 1;
                                    pump_save_state =6;
                                    if (DM_Pitch.Get_Control_Angle() > 0.85&&pump_save_state ==6)
                                    {
                                    pump_arm = 0;
                                    // usart_printf("%d\r\n",save_times);
                                    if ((pump_save == 0 && pump_save_state ==7)|| save_times >100)
                                    {
                                        pump_save_state =7;
                                        pitch_3508.Set_Target_Angle(250.0f+ pitch_3508.pos_init);
                                        DM_Pitch.Set_Control_Angle(1.0f);
                                        usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", DM_yaw.Get_Now_Angle(), DM_Pitch.Get_Now_Angle(),pitch_3508.Get_Now_Angle(),roll_3508.Get_Now_Angle(),DM_4310_pitch.Get_Now_Angle(),roll_2006.Get_Now_Angle());

                                        if(pitch_3508.Get_Target_Angle()>(200.0f + pitch_3508.pos_init) && DM_Pitch.Get_Control_Angle()>0.97  && pump_save_state ==7)
                                        {
                                            pump_save_state = 8;
                                        }
                                        }
                                    }

                                }
                        }
                    }
                }
            }

        }
    }
    else if(image_ctrl.key.B.Is_Click_Once == 0  && pump_get_flag ==true)
    {
        DM_yaw.Set_Control_Angle(yaw_val);
        yaw_val -= 0.03;
        if(DM_yaw.Get_Now_Angle() <= DM_yaw_offset )
        {
            yaw_val = DM_yaw_offset;
            roll_val += 1;
            roll_3508.Set_Target_Angle(roll_val+ roll_3508.pos_init);
            if(roll_3508.Get_Now_Angle()>-3.0f + roll_3508.pos_init)
            {
                roll_val = 0;
                pitch_3508.Set_Target_Angle(10.0f + pitch_3508.pos_init);
                DM_Pitch.Set_Control_Angle(0.0f);
                DM_4310_pitch.Set_Control_Angle(0.0f);
                if(DM_Pitch.Get_Now_Angle()<0.03)
                {
                    save_times = 0;
                    pump_get_flag = false;
                    yaw_val = -2.23;
                    roll_val = -62.0;
                }
            }

        }
    }
}

void Gold_Miner()
{
    static float pitch_10010_val = 2.32f;
    static float pitch_3508_val = 170.64f;
    static float pitch_4310_val = -0.70f;
    static float roll_val = roll_3508.pos_init;
    static int gold_times;
    static int gold_miner_states;
    if(image_ctrl.mouse.left_button_down.Is_Click_Once == 1)
    {
        gold_miner_flag = true;
        DM_Pitch.Set_Control_Angle(LIMIT_VALUE(pitch_10010_val,2.12,3));
        DM_4310_pitch.Set_Control_Angle(LIMIT_VALUE(pitch_4310_val,-1.5,-0.68));
        pitch_3508.Set_Target_Angle(pitch_3508_val + pitch_3508.pos_init);
        roll_3508.Set_Target_Angle(roll_val);
        pump_save = 1;
        gold_times++;
        usart_printf("%.2f,%.2f,%.2f,%d\r\n",DM_Pitch.Get_Now_Angle(),pitch_3508.Get_Now_Angle(),DM_4310_pitch.Get_Now_Angle(),gold_miner_states);
        // usart_printf("%.2f\r\n",wit.WIT_IMU_Angle(imu_yaw));
        if(DM_Pitch.Get_Now_Angle() > 2.31f && pitch_3508.Get_Now_Angle() > 160.0f + pitch_3508.pos_init && gold_miner_states == 0)
        {
            gold_miner_states = 1;

        }
        if(gold_miner_states == 1 && gold_times > 500 && image_ctrl.mouse.middle_button_down.Is_Click_Once == 1)
        {
            if(DM_4310_pitch.Get_Now_Angle()<-0.68)
            {
                pitch_4310_val += 0.01;
            }
            if(DM_Pitch.Get_Now_Angle()>2.12f)
            {
                pitch_10010_val -= 0.01;
            }
        }
    }
    else if(image_ctrl.mouse.left_button_down.Is_Click_Once == 0 && gold_miner_flag == true)
    {
        DM_Pitch.Set_Control_Angle(0.00);
        if(DM_Pitch.Get_Now_Angle()<0.02f)
        {
            pitch_3508_val = pitch_3508_val-3.0f;
            pitch_3508.Set_Target_Angle(pitch_3508_val + pitch_3508.pos_init);
        }
        if(pitch_3508.Get_Target_Angle() < 20+ pitch_3508.pos_init)
        {
            pitch_10010_val = 2.32f;
            pitch_3508_val = 170.64f;
            pitch_4310_val = -0.70f;
            roll_val = roll_3508.pos_init;
            gold_miner_flag = false;
            gold_times = 0;
            // pump_save = 0;
        }
    }

}

void Gold_Miner_Mid()
{
    static float pitch_10010_val_mid = 2.88f;
    static float pitch_3508_val_mid = 245.31f;
    static float pitch_4310_val_mid = -0.71f;
    static float roll_val_mid = roll_3508.pos_init;
    static int gold_mid_times;
    static int gold_miner_mid_states;
    if(image_ctrl.mouse.right_button_down.Is_Click_Once == 1)
    {
        gold_miner_mid_flag = true;
        DM_Pitch.Set_Control_Angle(LIMIT_VALUE(pitch_10010_val_mid,2.6,3.3));
        DM_4310_pitch.Set_Control_Angle(LIMIT_VALUE(pitch_4310_val_mid,-1.5,-0.60));
        pitch_3508.Set_Target_Angle(pitch_3508_val_mid + pitch_3508.pos_init);
        roll_3508.Set_Target_Angle(roll_val_mid);
        pump_save = 1;
        gold_mid_times++;
        usart_printf("%.2f,%.2f,%.2f,%d\r\n",DM_Pitch.Get_Now_Angle(),pitch_3508.Get_Now_Angle(),DM_4310_pitch.Get_Now_Angle(),gold_miner_mid_states);
        // usart_printf("%.2f\r\n",wit.WIT_IMU_Angle(imu_yaw));
        if(DM_Pitch.Get_Now_Angle() > 2.86f && pitch_3508.Get_Now_Angle() > 238.0f + pitch_3508.pos_init && gold_miner_mid_states == 0)
        {
            gold_miner_mid_states = 1;

        }
        if(gold_miner_mid_states == 1 && gold_mid_times > 500 && image_ctrl.mouse.middle_button_down.Is_Click_Once == 1)
        {
            if(DM_4310_pitch.Get_Now_Angle()<-0.62)
            {
                pitch_4310_val_mid += 0.01;
            }
            if(DM_Pitch.Get_Now_Angle()>2.78f)
            {
                pitch_10010_val_mid -= 0.01;
            }
        }
    }
    else if(image_ctrl.mouse.right_button_down.Is_Click_Once == 0 && gold_miner_mid_flag == true)
    {
        DM_Pitch.Set_Control_Angle(pitch_10010_val_mid);
        pitch_10010_val_mid -=0.01;
        if(DM_Pitch.Get_Now_Angle()<0.02f)
        {
            pitch_3508_val_mid = pitch_3508_val_mid-3.0f;
            pitch_3508.Set_Target_Angle(pitch_3508_val_mid + pitch_3508.pos_init);
        }
        if(pitch_3508.Get_Target_Angle() < 20+ pitch_3508.pos_init)
        {
            pitch_10010_val_mid = 2.88f;
            pitch_3508_val_mid = 245.31f;
            pitch_4310_val_mid = -0.71f;
            roll_val_mid = roll_3508.pos_init;
            image_ctrl.mouse.middle_button_down.Is_Click_Once = 0;
            gold_miner_mid_flag = false;
            gold_mid_times = 0;
            // pump_save = 0;
        }
    }

}

void Silver_Miner()
{
    static float pitch_10010_val = 2.31f;
    static float pitch_3508_val = 92.0f;
    static float pitch_4310_val = -0.30f;
    static float roll_val = -62.0 + roll_3508.pos_init;
    static int silver_times;
    static int silver_miner_states;
    if (image_ctrl.key.F.Is_Click_Once == 1)//换按键
    {
        silver_miner_flag = true;
        DM_Pitch.Set_Control_Angle(pitch_10010_val);
        DM_4310_pitch.Set_Control_Angle(LIMIT_VALUE(pitch_4310_val,-0.8,0.9));
        pitch_3508.Set_Target_Angle(pitch_3508_val + pitch_3508.pos_init);
        roll_3508.Set_Target_Angle(roll_val);
        pump_save = 1;
        usart_printf("%.2f,%.2f,%.2f,%d\r\n",pitch_3508.Get_Target_Angle(),pitch_3508.Get_Now_Angle(),DM_4310_pitch.Get_Now_Angle(),silver_miner_states);
        if(DM_Pitch.Get_Now_Angle() > 2.3f && pitch_3508.Get_Now_Angle() > 82.0f + pitch_3508.pos_init && silver_miner_states == 0)
        {
            silver_miner_states = 1;
        }
        if (silver_miner_states == 1)
        {
            if(DM_Pitch.Get_Now_Angle() > 1.60f)
            pitch_10010_val -= 0.03;
            if(DM_4310_pitch.Get_Now_Angle() > -0.57f)
            pitch_4310_val -= 0.01;
            if(DM_4310_pitch.Get_Now_Angle() < -0.56f && DM_Pitch.Get_Now_Angle() < 1.61f && silver_miner_states == 1)
            {
                silver_miner_states = 2;
            }
        }
        if(silver_miner_states == 2)
        {
            // if(DM_Pitch.Get_Now_Angle() > 1.0f)
            //     pitch_10010_val -= 0.02;
            if(pitch_3508.Get_Now_Angle() < 110.0f + pitch_3508.pos_init )
                pitch_3508_val = pitch_3508_val + 2.0f;
            if(DM_4310_pitch.Get_Now_Angle() > -0.75f)
                pitch_4310_val -= 0.05;
            // if(pitch_3508.Get_Now_Angle() > (128.0f + pitch_3508.pos_init) && DM_4310_pitch.Get_Now_Angle() >0.79f  && silver_miner_states == 2);
            // silver_miner_states = 3;
        }
    }
    else if(image_ctrl.key.F.Is_Click_Once == 0 && silver_miner_flag == true)
    {
        roll_val += 1;
        roll_3508.Set_Target_Angle(roll_val+ roll_3508.pos_init);
        if(roll_3508.Get_Now_Angle()>-3.0f + roll_3508.pos_init)
        {
            roll_val = 0;
            pitch_3508.Set_Target_Angle(30.0f + pitch_3508.pos_init);
            DM_Pitch.Set_Control_Angle(0.2f);
            DM_4310_pitch.Set_Control_Angle(-0.2f);
            if(DM_Pitch.Get_Now_Angle()<0.22)
            {
                silver_times = 0;
                silver_miner_states = 0;
                silver_miner_flag = false;
                pitch_10010_val = 2.31f;
                pitch_3508_val = 92.0f;
                pitch_4310_val = -0.30f;
                roll_val = -62.0 + roll_3508.pos_init;
            }
        }
    }
}

void End_Roll_Control()
{
    static float target_angle_;
    // roll_2006.Set_Target_Angle(Get_Self_Ctrl().roll_2006);
    if(image_ctrl.key.Z.Is_Click_Hold)
    {
        target_angle_ += 0.5f;
    }
    else if(image_ctrl.key.C.Is_Click_Hold)
    {
        target_angle_ -= 0.5f;
    }
    roll_2006.Set_Target_Angle(target_angle_);
    // target_angle_ = self_ctrl.roll_2006 / 180.0f * 116.0f;
    roll_2006.Set_Target_Angle(target_angle_);
}

void End_Pitch_Control()
{
    static float target_angle_;
    // roll_2006.Set_Target_Angle(Get_Self_Ctrl().roll_2006);
    // if(image_ctrl.key.Q.Is_Click_Hold)
    // {
    //     target_angle_ -= 0.01f;
    // }
    // else if(image_ctrl.key.X.Is_Click_Hold)
    // {
    //     target_angle_ += 0.01f;
    // }
    target_angle_ = self_ctrl.pitch_4310 /360.0f * 2.0f * PI;
    DM_4310_pitch.Set_Control_Angle(LIMIT_VALUE(target_angle_,-1.5,1.0));

}