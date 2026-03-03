//
// Created by ZONE7 on 2024/9/27.
//

#include "gimbal.hpp"
#include "usart.h"
#include "canio.hpp"
#include "pidc.hpp"
#include "motorc.hpp"
#include "self_control.hpp"
#include "ledio.hpp"
#include "unitree.hpp"
#include "uni_motor.hpp"
#include "receive.hpp"
#include "debugc.hpp"
#include "witimu.hpp"
#include "remotec.hpp"
#include "DM4310.hpp"
#include "DM10010.hpp"
#include "chassisc.hpp"
#include "yaw_ctrl.hpp"
#include "pitch_ctrl.hpp"
#include "end_act.hpp"

float pos_init;
float pitch_init;
bool ARM_init;
bool DM_init = false;
bool DM_Pos_init = false;
bool Image_init = false;
bool Init = false;
uint8_t init_staus = 0;

// gimbal gimbal;
//
// void gimbal::
 void self_check()
{
    static int16_t val_motor1 = 0, val_motor2 = 0, val_motor3 = 0, val_motor4 = 0, val_tuchuan1 = 0, val_tuchuan2 = 0;
    // static float val_uni = 0;
    if (pitch_3508.Get_Output_Max() > 4000 || pitch_3508.Get_Output_Max() < -4000) {//反馈电流超过5000
        pitch_3508.init = true;//电机初始化完成
         pitch_3508.pos_init = pitch_3508.Get_Now_Angle();//将电机初始位置设置为此时的位置
        pitch_3508.Set_Target_Angle(pitch_3508.pos_init + 20);
    } else if (pitch_3508.init == false)
    {
        //未完成初始化
        val_motor1 -= 3;//向一个方向转动
        pitch_3508.Set_Target_Angle(val_motor1);
    }
    // usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f\r\n",pitch_3508.init,pitch_3508.pos_init,val_motor1,pitch_3508.Get_Now_Angle(),pitch_3508.Get_Now_Torque());
}

void Gimbal_Init()
{
    //初始化各个模块
     PID_Init();
     chassis_com.Init();
     end_pitch.Init();
     pitch_init = DM_Pitch.Get_Now_Angle();
}

void PID_Init()
 {
     pitch_3508.PID_Angle.Init(11.0f, 2.01f, 0.0f, 0.0f, 15.0f * PI, 150.0f * PI);
     pitch_3508.PID_Omega.Init(35.0f, 0.10f, 0.0f, 0.0f, 2500.0f,  13000.0f);
     pitch_3508.Init(&hcan1, CAN_Motor_ID_0x201, Control_Method_ANGLE, 1.0f);//电机初始化，完成接收数组与对应通道的绑定

     roll_3508.PID_Angle.Init(15.0f, 0.1f, 0.0f, 0.0f, 15.0f * PI, 150.0f * PI);
     roll_3508.PID_Omega.Init(30.0f, 0.2f, 0.0f, 0.0f, 2500.0f,  13000.0f);
     roll_3508.Init(&hcan1, CAN_Motor_ID_0x202 , Control_Method_ANGLE, 1.0f);//电机初始化，完成接收数组与对应通道的绑定


     roll_2006.PID_Angle.Init(10.0f, 0.1f, 0.0f, 0.0f, 15.0f * PI, 150.0f * PI);
     roll_2006.PID_Omega.Init(20.0f, 0.0f, 0.0f, 0.0f, 2500.0f,  9000.0f);
     roll_2006.Init(&hcan1, CAN_Motor_ID_0x203, Control_Method_ANGLE, 1.0f);//电机初始化，完成接收数组与对应通道的绑定


     image_ptich.PID_Angle.Init(20.0f, 0.0f, 0.0f, 0.0f, 15.0f * PI, 150.0f * PI);
     image_ptich.PID_Omega.Init(20.0f, 0.05f, 0.0f, 0.0f, 2500.0f,  9000.0f);
     image_ptich.Init(&hcan2, CAN_Motor_ID_0x205, Control_Method_ANGLE, 1.0f);//电机初始化，完成接收数组与对应通道的绑定
 }

void motor_data_update()
 {

 }

void chassis_data_update()
 {
     RC_Levers(); //获取摇杆数据，一般得到标志位的值
     chassis_com.getCtrlData();
     chassis_com.canSend();

 }

void Gimbal_Loop()
 {

     // pump_arm = 1;
     // pump_save = 1;
     if (portSetProtect())
     {
         // DM_yaw.Set_Control_Angle(portSetAngle()-125/360.0f * 2.0f * PI);
         // DM_4310_pitch.Set_Control_Angle((portSetAngle()-50.0f)/360.0f * 2.0f * PI);
         // DM_Pitch.Set_Control_Angle((portSetAngle()-125.0f)/360.0f * 2.0f * PI);

         usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", DM_yaw.Get_Now_Angle(), DM_Pitch.Get_Now_Angle(),pitch_3508.Get_Now_Angle(),roll_3508.Get_Now_Angle(),DM_4310_pitch.Get_Now_Angle(),roll_2006.Get_Now_Angle());
         // usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%. 2f\r\n", DM_4310_pitch.Get_Now_Angle(), DM_4310_pitch.Get_Control_Angle(),DM_Pitch.Get_Now_Angle(),DM_Pitch.Get_Control_Angle(),DM_yaw.Get_Now_Angle(),DM_yaw.Get_Control_Angle());
         // usart_printf("%.2f,%.2f,%.2f\r\n",wit.WIT_IMU_Angle(imu_yaw),wit.WIT_IMU_Angle(imu_pitch),wit.WIT_IMU_Angle(imu_roll));
         // usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",receive_from_self_control.Get_Angle1(),receive_from_self_control.Get_Angle2(),receive_from_self_control.Get_Angle3(),receive_from_self_control.Get_Angle4(),receive_from_self_control.Get_Angle5(),receive_from_self_control.Get_Angle6());
         // usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",roll_2006.Get_Now_Angle(),pitch_3508_2.Get_Now_Angle(),roll_3508.Get_Now_Angle(),pitch_3508.Get_Now_Angle(),DM_Pitch.Get_Now_Angle(),DM_yaw.Get_Now_Angle());
         // usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f\r\n", pitch_3508.Get_Target_Omega(),pitch_3508.Get_Now_Omega(),pitch_3508.Get_Target_Angle(),pitch_3508.Get_Now_Angle(),pitch_3508.Get_Now_Torque());
         // usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f\r\n", roll_3508.Get_Target_Omega(),roll_3508.Get_Now_Omega(),roll_3508.Get_Target_Angle(),roll_3508.Get_Now_Angle(),roll_3508.Get_Now_Torque());
         // usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f\r\n", roll_3508_1.Get_Target_Omega(),roll_3508_1.Get_Now_Omega(),roll_3508_1.Get_Target_Angle(),roll_3508_1.Get_Now_Angle(),roll_3508_1.Get_Now_Torque());
         // usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f\r\n", roll_2006.Get_Target_Omega(),roll_2006.Get_Now_Omega(),roll_2006.Get_Target_Angle(),roll_2006.Get_Now_Angle(),roll_2006.Get_Now_Torque());

         protect();
         led.ON();
     }
     else
     {

         PosInit();

         if (pump_save_flag == false
             && pump_get_flag == false
             && silver_miner_flag ==false
             && gold_miner_flag == false
             && gold_miner_mid_flag == false
             && ARM_init == true
             )
         {
             End_Roll_Control();
             roll_3508.Set_Target_Angle(Limit_Angle(((Get_Self_Ctrl().roll_3508)/ 80.0f * 30.0f + roll_3508.pos_init),-60+ roll_3508.pos_init,60+ roll_3508.pos_init));
             pitch_3508.Set_Target_Angle( Limit_Angle(((Get_Self_Ctrl().pitch_3508) / 110.0f * 308.0f + pitch_3508.pos_init ),(0+ pitch_3508.pos_init),390+ pitch_3508.pos_init)-0.4f);
             yaw_control();
             end_pitch.SetEndMode();
             // DM_4310_pitch.Set_Control_Angle(Limit_Angle(Get_Self_Ctrl().pitch_4310/360.0f * 2.0f * PI,-1.5,1.2));
             // End_Pitch_Control();
             // DM_Pitch.Set_Control_Angle(Limit_Angle(portSetAngle()/ 180.0f * 2.0f * PI,-1.11,3.77));
             pitch_control();
         }
         if(Image_init == true)
         {
             image_ptich.Set_Target_Angle(Limit_Angle((portSetAngle() + image_ptich.pos_init),-60+ image_ptich.pos_init,20+ image_ptich.pos_init));

         }
         Pump_Save();
         Pump_Get();
         Gold_Miner();
         Gold_Miner_Mid();
         Silver_Miner();
         chassis_data_update();

         // usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", DM_yaw.Get_Now_Angle(), DM_Pitch.Get_Now_Angle(),pitch_3508.Get_Now_Angle(),roll_3508.Get_Now_Angle(),DM_4310_pitch.Get_Now_Angle(),roll_2006.Get_Now_Angle());
         // usart_printf("%.2f,%.2f\r\n",pitch_3508.pos_init,roll_3508.pos_init);
         // pitch_3508.Set_Target_Angle(80.0f);Get_Self_Ctrl
         // usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", DM_yaw.Get_Now_Angle(), DM_Pitch.Get_Now_Angle(),pitch_3508.Get_Now_Angle(),roll_3508.Get_Now_Angle(),DM_4310_pitch.Get_Now_Angle(),roll_2006.Get_Now_Angle());

         // Limit_Angle(Get_Self_Ctrl().roll_3508/ 80.0f * 30.0f,-30,60);

         // Limit_Angle((-Get_Self_Ctrl().pitch_3508 / 110.0f * 308.0f),0,390);

         // //

         // // roll_2006.Set_Target_Angle(receive_from_self_control.Get_Angle1()/ 180.0f * 116.0f);

         // // // //
         // DM_yaw.Set_Control_Angle(( receive_from_self_control.Get_Angle6() - 58.24f ) / 180.f * 2.5f * PI);
         // DM_yaw.Set_Control_Angle((Get_Self_Ctrl().yaw_4310 - 64)/180.0f * 2.0f * PI);
         // DM_yaw.Set_Control_Angle(-portSetAngle_Z() / 200.0f  - 2.23f );

         //-6.00,/-2.23,/3.38./5.33,


         // usart_printf("%.2f,%.2f，%.2f,%.2f\r\n", DM_4310_pitch.Get_Now_Angle(), DM_4310_pitch.Get_Control_Angle(),DM_Pitch.Get_Now_Angle(),DM_Pitch.Get_Control_Angle());
         // Limit_Angle(-Get_Self_Ctrl().pitch_10010 / 180.0f * 2.0f * PI,-1.11,3.77);

         DM_Pitch.TIM_Send_PeriodElapsedCallback();
         // usart_printf("%.2f,%.2f,%.2f,%.2f\r\n",DM_Pitch.Get_Control_Angle(),DM_Pitch.Get_Now_Angle(),DM_Pitch.Get_Now_MOS_Temperature(),DM_Pitch.Get_Now_Torque());

         DM_yaw.Set_Control_Omega(4*PI);
         DM_yaw.TIM_Send_PeriodElapsedCallback();
         // Limit_Angle(Get_Self_Ctrl().pitch_4310/360.0f * 2.0f * PI,-1.5,1.2);
         //up-1.5,down1.2
         static uint32_t Counter_KeepAlive = 0;
         if (Counter_KeepAlive++ > 100)
         {
             Counter_KeepAlive = 0;

             DM_yaw.TIM_Alive_PeriodElapsedCallback();
             DM_Pitch.TIM_Alive_PeriodElapsedCallback();
             DM_4310_pitch.TIM_Alive_PeriodElapsedCallback();

         }
         // usart_printf("%.2f,%.2f,%.2f,%.2f\r\n",DM_4310_pitch.Get_Control_Angle(),DM_4310_pitch.Get_Now_Angle(),DM_4310_pitch.Get_Now_MOS_Temperature(),DM_4310_pitch.Get_Now_Rotor_Temperature());
         //
         // // usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",receive_from_self_control.Get_Angle1(),receive_from_self_control.Get_Angle2(),receive_from_self_control.Get_Angle3(),receive_from_self_control.Get_Angle4(),receive_from_self_control.Get_Angle5(),receive_from_self_control.Get_Angle6());
         // usart_printf("%.2f,%.2f,%.2f\r\n", DM_Pitch.Get_Now_Angle(), DM_Pitch.Get_Control_Angle(),DM_Pitch.Get_Now_Torque());
         //
         // 保持存活

         if (DM_init == true)
         {


             DM_4310_pitch.Set_Control_Omega(PI);
             DM_4310_pitch.TIM_Send_PeriodElapsedCallback();



             pitch_3508.TIM_PID_PeriodElapsedCallback();
             roll_3508.TIM_PID_PeriodElapsedCallback();
             image_ptich.TIM_PID_PeriodElapsedCallback();
             roll_2006.TIM_PID_PeriodElapsedCallback();
             image_ptich.TIM_PID_PeriodElapsedCallback();
         }



         // usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",receive_from_self_control.Get_Angle1(),receive_from_self_control.Get_Angle2(),receive_from_self_control.Get_Angle3(),receive_from_self_control.Get_Angle4(),receive_from_self_control.Get_Angle5(),receive_from_self_control.Get_Angle6());


         // roll_3508.PID_Angle.Init(param.pos_kp, param.pos_kf, param.pos_kd, 0.0f, 15.0f * PI, 300.0f * PI);
         // roll_3508.PID_Omega.Init(param.vel_kp, param.vel_ki, 0.0f, 0.0f, 2500.0f,  13000.0f);
         // pitch_3508.Set_Target_Angle(portSetAngle());

         //
         // roll_2006.PID_Angle.Init(param.pos_kp, 0.0f, param.pos_kd, 0.0f, 15.0f * PI, 150.0f * PI);
         // roll_2006.PID_Omega.Init(param.vel_kp, param.vel_ki, 0.0f, 0.0f, 2500.0f,  16384.0f);
         // roll_2006.Set_Target_Angle(portSetAngle());

         // usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f\r\n", pitch_3508.Get_Target_Omega(),pitch_3508.Get_Now_Omega(),pitch_3508.Get_Target_Angle(),pitch_3508.Get_Now_Angle(),pitch_3508.Get_Now_Torque(),roll_2006.pos_init);
         // usart_printf("%.2f\r\n", portSetAngle());
         // usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f\r\n", roll_2006.Get_Target_Omega(),roll_2006.Get_Now_Omega(),roll_2006.Get_Target_Angle(),roll_2006.Get_Now_Angle(),roll_2006.Get_Now_Torque(),roll_2006.pos_init);

         // pitch_3508_2.TIM_PID_PeriodElapsedCallback();

         // usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f\r\n", roll_3508.Get_Target_Omega(),roll_3508.Get_Now_Omega(),roll_3508.Get_Target_Angle(),roll_3508.Get_Now_Angle(),roll_3508.Get_Now_Torque());

         // usart_printf("%f,%f,%d,%f\r\n",roll_3508.Get_Now_Torque(),roll_3508.pos_init,init_staus,roll_3508.Get_Now_Angle());
         // motor.Set_Target_Torque(50);
         // motor.Set_Target_Angle(param.pos_tarvalue);

         // motor.Set_Target_Angle(-receive_from_self_control.Get_Angle1());
         // usart_printf("%.2f,%.2f\r\n", roll.Get_Target_Omega(),roll.Get_Now_Omega());

         // usart_printf("%d,%d,%d,%d\r\n",init_staus,image_ctrl.rc.User_Key_L.Is_Click_Once,image_ctrl.rc.User_Key_L.Now_State,image_ctrl.rc.User_Key_L.Last_State);

     }

 }

void DM_Init()
 {
     if(DM_Pos_init == false)
     {
         DM_yaw.Set_Control_Angle(DM_yaw_offset);
         DM_Pitch.Set_Control_Omega(PI);
         DM_Pitch.Set_Control_Angle(0.00f);
     }
     if(DM_Pitch.Get_Now_Angle()>=-0.02 && DM_Pitch.Get_Now_Angle()<=0.02)
     {
         DM_Pos_init = true;
     }
 }

void ARM_POS_RESET()
 {
     if(image_ctrl.rc.User_Key_L.Is_Click_Once == 1&& image_ctrl.rc.User_Key_L.Now_State == 1 && image_ctrl.rc.User_Key_L.Last_State ==0)
     {
         DM_init = false;
         ARM_init = false;
         pitch_3508.init = false;
         roll_2006.init = false;
         init_staus = 0;
     }
 }

void PosInit()
 {
     static float val_uni = pos_init;
     static int Init_delay;
     static int16_t val_yawl;
     static float pitch_temp;
     // ARM_POS_RESET();
     //第一级Pitch初始化完成后再进行机械臂初始化
     DM_Init();
     if(DM_Pos_init == true)
         Init_delay++;
     if( Init_delay > 100 && (pitch_3508.Get_Now_Torque() < 3000 && pitch_3508.Get_Now_Torque() > -3000))
         DM_init = true;
     if(DM_init == true)
     {
         // usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f\r\n", pitch_3508.Get_Target_Omega(),pitch_3508.Get_Now_Omega(),pitch_3508.Get_Target_Angle(),pitch_3508.Get_Now_Angle(),pitch_3508.Get_Now_Torque(),roll_2006.pos_init);
         ARM_Pos_Init();//机械臂初始化
         Image_Init();
         if(init_staus == 2)
         {
            ARM_init = true;
         }

     }
 }


void ARM_Pos_Init()
    {
        static float val_motor[2]={pitch_3508.Get_Now_Angle(),roll_3508.Get_Now_Angle()};
        static int pitch_init_times;
        static int pitch_check_times;
        static int roll_init_times;
        /************************************ pitch1 ***********************************/
        if(init_staus == 0)
        {
            if ((pitch_3508.Get_Now_Torque() > 4000 || pitch_3508.Get_Now_Torque() < -4000) && pitch_3508.init == false)
            {//反馈电流超过5000
                pitch_check_times++;
                if(pitch_check_times > 100)
                {
                    pitch_3508.init = true;//电机初始化完成
    //            Engineer.Init = true;
                    pitch_3508.pos_init = pitch_3508.Get_Now_Angle() + 15.0f;//将电机初始位置设置为此时的位置
                    pitch_3508.Set_Target_Angle(pitch_3508.pos_init +PITCH_3508_MidPOS);
                    pitch_3508.action = false;//从初始位置到居中位置的动作
                }
                // usart_printf("%.2f,%.2f,%.2f,%.2f,%d\r\n",pitch_3508.Get_Target_Angle(),pitch_3508.Get_Now_Angle(),pitch_3508.Get_Now_Torque(),pitch_3508.pos_init,val_motor[0]);
            }
            else if (pitch_3508.init == false)
            {//未完成初始化
                val_motor[0] -= 3 / 19.0f;//向一个方向转动
                pitch_3508.Set_Target_Angle(val_motor[0]);
            }
            else if(pitch_3508.init == true)
            {
                pitch_init_times++;
                if(pitch_init_times > 50)
                {
                    init_staus = 1;
                    val_motor[0] = 0;
                    pitch_init_times = 0;
                }
            }
        }

        /************************************ Roll1***********************************/
        if(init_staus == 1)
        {
            if ((roll_3508.Get_Now_Torque() > 4000 || roll_3508.Get_Now_Torque() < -4000) && roll_3508.init == false)
            {//反馈电流超过5000
                roll_3508.init = true;//电机初始化完成
                //            Engineer.Init = true;
                roll_3508.pos_init = roll_3508.Get_Now_Angle()-30.0f;//将电机初始位置设置为此时的位置
                roll_3508.Set_Target_Angle(roll_3508.pos_init + ROLL_3508_MidPOS);
                roll_3508.action = false;//从初始位置到居中位置的动作
                // usart_printf("%.2f,%.2f,%.2f,%d,%d\r\n",roll_3508.Get_Target_Angle(),roll_3508.Get_Now_Angle(),roll_3508.Get_Now_Torque(),roll_3508.pos_init,val_motor[1]);
            }
            else if (roll_3508.init == false)
            {//未完成初始化
                val_motor[1] += 3 / 19.0f;//向一个方向转动
                roll_3508.Set_Target_Angle(val_motor[1]);
            }
            else if(roll_3508.init == true)
            {
                roll_init_times++;
                if(roll_init_times > 50)
                {
                    init_staus = 2;
                    val_motor[1] = 0;
                    roll_init_times = 0;
                }
            }
        }
    }
void Image_Init()
 {
     static float val_motor;
     static int image_init_times;
     // usart_printf("image_init\r\n");
     if ((image_ptich.Get_Now_Torque() > 5000 || image_ptich.Get_Now_Torque() < -5000) && image_ptich.init == false)
     {//反馈电流超过5000
         image_ptich.init = true;//电机初始化完成
         //            Engineer.Init = true;
         image_ptich.pos_init = image_ptich.Get_Now_Angle()- PITCH_Image_MidPOS;//将电机初始位置设置为此时的位置
         image_ptich.Set_Target_Angle(image_ptich.pos_init );
         image_ptich.action = false;//从初始位置到居中位置的动作
         // usart_printf("%.2f,%.2f,%.2f,%.2f,%d\r\n",pitch_3508.Get_Target_Angle(),pitch_3508.Get_Now_Angle(),pitch_3508.Get_Now_Torque(),pitch_3508.pos_init,val_motor[0]);
     }
     else if (image_ptich.init == false)
     {//未完成初始化
         val_motor += 3 / 36.0f;//向一个方向转动
         image_ptich.Set_Target_Angle(val_motor);
     }
     else if(image_ptich.init == true)
     {
         image_init_times++;
         if(image_init_times > 50)
         {
             Image_init = true;
             val_motor = 0;
             image_init_times = 0;
         }
     }
 }
    void ARM_POS_Check()
    {
        if (pitch_3508.action == false) {
            if ((pitch_3508.Get_Now_Angle() - pitch_3508.Get_Target_Angle() > -1.0f) && (pitch_3508.Get_Now_Angle() - pitch_3508.Get_Target_Angle() < 1.0f)) {
                pitch_3508.action = true;
            }
        }
        if (roll_3508.action == false) {
            if ((roll_3508.Get_Now_Angle() - roll_3508.Get_Target_Angle() > -1.0f) && (roll_3508.Get_Now_Angle() - roll_3508.Get_Target_Angle() < 1.0f)) {
                roll_3508.action = true;
            }
        }
    }



void protect()
 {
     pitch_3508.Protect();
     roll_3508.Protect();
     roll_2006.Protect();
     DM_yaw.CAN_Send_Exit();
     DM_Pitch.CAN_Send_Exit();
     DM_4310_pitch.CAN_Send_Exit();
     image_ptich.Protect();
     // init_staus = 0;
     portSetAngle();

     chassis_com.protect();
 }