//
// Created by ZONE7 on 2024/11/16.
//

#ifndef WITIMU_HPP
#define WITIMU_HPP

#define IMU_HEAD 0X55               //端口状态
#define IMU_ANGLE 0X53              //角度
#define IMU_QUATERNION 0X59         //四元数
#define IMU_SPEED 0X52              //角速度
#include "usartio.hpp"
#define WITIMU_RVSIZE 255


// #define WITIMU_RVSIZE 33

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} GetQuaternion; // 0-8191 对应0-360°
typedef enum
{
    imu_yaw,
    imu_pitch,
    imu_roll,
} IMU_data;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} GetNaiveAngle; // 0-8191 对应0-360°

class witimu :public cUSARTC
{
public:
    witimu(UART_HandleTypeDef *uart,uint16_t size,eUSART_type type);
    void rxUserCALLBACK() override;
    float IMU_AngleIncreLoop(float angle_now);
    void IMU_Data_Handler(void);
    float WIT_IMU_Angle(IMU_data Witch_angle);
    float WIT_IMU_Speed(IMU_data which);
    void IMU_Clean_Rotate();
private:
    char wit_rx_buf_[WITIMU_RVSIZE]{};
    char wit_buf_[WITIMU_RVSIZE]{};
     float angle[3] = {0};
     float speed[3] = {0};//PID用
    float Yaw_Angle;
    float Roll_Angle;
    float Pitch_Angle;
    float Yaw_Speed;
    float Roll_Speed;
    float Pitch_Speed;
    float last_angle;
    int32_t rotate_times;
};

extern witimu wit;



#endif //WITIMU_HPP
