//
// Created by ZONE7 on 2024/9/24.
//

#include "gimbal_task.hpp"
#include "cmsis_os.h"
#include "usart.h"
#include "canio.hpp"
#include "chassisc.hpp"
#include  "gimbal.hpp"

#include "pidc.hpp"
#include "motorc.hpp"
#include "self_control.hpp"
#include "ledio.hpp"
#include "unitree.hpp"
#include "uni_motor.hpp"
#include "DM4310.hpp"
#include "DM10010.hpp"
#include "image_referee.hpp"
#include "remotec.hpp"
#include "receive.hpp"
#include "debugc.hpp"
#include "canio_basic.hpp"

// uni_motor pitch_motor(&huart6);
float _tar_pos = 0.0f;
float _tar_pos1 = 0.0f;


uint32_t Counter = 0;
void CAN_Motor_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer);
void CAN_Motor_Call_Back_Can2(Struct_CAN_Rx_Buffer *Rx_Buffer);

void gimbal_task(void const * argument)
{
    TickType_t current;

    CAN_Init(&hcan1, CAN_Motor_Call_Back);//CAN初始化
    CAN_Init(&hcan2, CAN_Motor_Call_Back_Can2);//CAN初始化
    // CAN_Filter_Init(&hcan2);//CAN滤波器初始化
    Gimbal_Init();
    // Uni_Motor.Init();
    DM_yaw.Init(&hcan1,0x00,0x01,Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA);
    DM_Pitch.Init(&hcan1,0x02,0x03,Motor_DM_10010_Control_Method_NORMAL_ANGLE_OMEGA);
    DM_4310_pitch.Init(&hcan1,0x04,0x05,Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA);
    /* USER CODE BEGIN GimbalTask */
    /* Infinite loop */
    for(;;)
    {
        current = xTaskGetTickCount();
        // usart_printf("%.2f,%.2f\r\n",portSetVy(),portSetVx());
        // usart_printf("%d,%d\r\n",Get_Image_Ctrl().key.W.Is_Click_Hold,Get_Image_Ctrl().key.W.Is_Click_Once);
       // if(C5_portSetProtect())
       // {
       //     usart_printf("%d,%d,%d,%d,%d,%d,%d,%d\r\n",image_ctrl.rc.Pause_Key.Is_Click_Hold,image_ctrl.rc.Pause_Key.Is_Click_Once,image_ctrl.rc.Trigger_Key.Is_Click_Hold,image_ctrl.rc.Trigger_Key.Is_Click_Once,
       //                                              image_ctrl.rc.User_Key_L.Is_Click_Hold,image_ctrl.rc.User_Key_L.Is_Click_Once,image_ctrl.rc.User_Key_R.Is_Click_Hold,image_ctrl.rc.User_Key_R.Is_Click_Once);
       // }
         // usart_printf("%.2f,%.2f,%.2f\r\n",self_control1.Get_Control_Angle(),self_control2.Get_Control_Angle(),self_control3.Get_Control_Angle());
        // usart_printf("%.2f,%.2f,%.2f\r\n",self_control1.Get_Control_Last_Angle(),self_control2.Get_Control_Last_Angle(),self_control3.Get_Control_Last_Angle());
        /*
         *debug调参
         */
        // roll1.PID_Angle.Init(param.pos_kp, 0.0, param.pos_kd, param.pos_kf, 15.0f * PI, 15.0f * PI);
        // roll.PID_Angle.Init(param.pos_kp, 0.0, param.pos_kd, param.pos_kf, 15.0f * PI, 15.0f * PI);
        // roll1.PID_Omega.Init(param.vel_kp, param.vel_ki, 0.0f, param.vel_kf, 2500.0f,  1000.0f);


        // roll.Set_Target_Angle(self_control1.Get_Control_Angle()/19.0);


        // roll1.Set_Target_Angle(param.pos_tarvalue);
        // roll1.Set_Target_Omega(param.vel_tarvalue);
        // usart_printf("%.2f,%.2f,%.2f\r\n",param.vel_tarvalue,roll1.Get_Now_Omega(),param.vel_kp);
        // usart_printf("%.2f,%.2f\r\n",roll1.Get_Target_Angle(),roll1.Get_Now_Angle());
        // pitch_motor.SetPos(self_control1.Get_Control_Angle()/180.0*3.14);

        // if(((self_control2.Get_Control_Angle() - self_control2.Get_Control_Last_Angle() )< 180 ) && ((self_control2.Get_Control_Angle() - self_control2.Get_Control_Last_Angle()) > -180))
        // {
        //     self_control2.Set_Control_Last_Angle();
        // motor.Set_Target_Angle(self_control2.Get_Control_Angle()/PI);
        // motor1.Set_Target_Angle(self_control2.Get_Control_Angle()/PI);
        // }
        // usart_printf("%.2f\r\n",receive_from_self_control.Get_Angle1());


        // if (pump_flag == 1)
        // {
        //     HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET);
        //      HAL_GPIO_WritePin(GPIOA, PUMP_Pin, GPIO_PIN_SET);
        // }
        // else if (pump_flag == 0)
        // {
        //
        //      HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_RESET);
        //      HAL_GPIO_WritePin(GPIOA, PUMP_Pin, GPIO_PIN_RESET);
        // }
        // usart_printf("%d\r\n",pump_flag);
        // usart_printf("%.2f,%.2f,%.2f，%.2f,%.2f,%.2f\r\n",receive_from_self_control.Get_Angle1(),receive_from_self_control.Get_Angle2(),receive_from_self_control.Get_Angle3(),receive_from_self_control.Get_Angle4(),receive_from_self_control.Get_Angle5(),receive_from_self_control.Get_Angle6());

        Gimbal_Loop();

        // HAL_Delay( 3000);


        // usart_printf("%.2f,%.2f\r\n",motor.Get_Target_Angle(),roll.Get_Target_Angle());
        // motor.Set_Target_Angle(self_control2.Get_Control_Angle()/5);
        // motor1.Set_Target_Angle(self_control1.Get_Control_Angle()/5);


        // _tar_pos += (float) remote.Get_Remote().rc.ch[0] * 600.f * 0.002/ 660.0f / 19 ;//遥控器指令，后续按轴封装
        // motor.Set_Target_Angle(_tar_pos);
        // _tar_pos1 += (float) remote.Get_Remote().rc.ch[1] * 600.f * 0.002/ 660.0f / 19 ;
        // motor1.Set_Target_Angle(_tar_pos1);

        ;

        // motor_data_update();

        //输出数据到电机

        //通信设备回调数据
        TIM_CAN_PeriodElapsedCallback();

        vTaskDelayUntil(&current, 5 / portTICK_RATE_MS);
    }
    /* USER CODE END GimbalTask */
}

/**
 * @brief CAN报文回调函数
 *
 * @param Rx_Buffer CAN接收的信息结构体
 */
void CAN_Motor_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
{

    switch (Rx_Buffer->Header.StdId)
    {
    case (0x201):
        {
            pitch_3508.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
    case (0x202):
        {
            roll_3508.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
    case (0x203):
        {
            roll_2006.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
    case (0x204):
        {

        }
        break;
    case (0x205):
        {

        }
        break;
    case (0x00):
        {
            DM_yaw.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
   case (0x02):
       {
           DM_Pitch.CAN_RxCpltCallback(Rx_Buffer->Data);
       }
       break;
   case (0x04):
       {
           DM_4310_pitch.CAN_RxCpltCallback(Rx_Buffer->Data);
       }
        break;
    }
}

void CAN_Motor_Call_Back_Can2(Struct_CAN_Rx_Buffer *Rx_Buffer)
{

    switch (Rx_Buffer->Header.StdId)
    {
    case (0x205):
        {
            image_ptich.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;

    }
}