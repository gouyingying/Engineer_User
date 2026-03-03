//
// Created by ZONE7 on 2024/9/27.
//

#ifndef GIMBAL_HPP
#define GIMBAL_HPP

#define PITCH_3508_MidPOS 100
#define ROLL_3508_MidPOS 0
#define PITCH_Image_MidPOS 30
#define ROLL_2006_MidPOS 180
#define DM_yaw_offset -2.23f
// class gimbal {
//     public:
    void self_check();
    void Gimbal_Loop();
    void PID_Init();
    void Gimbal_Init();
    void motor_data_update();
    void PosInit();
    void ARM_Pos_Init();
    void ARM_POS_Check();
    void Image_Init();
    void protect();
    void chassis_data_update();
    // private:

// };

// extern gimbal gimbal;

#endif //GIMBAL_HPP
