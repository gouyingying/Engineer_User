//
// Created by ZONE7 on 2025/2/17.
//

#ifndef DM10010_HPP
#define DM10010_HPP

#include "pidc.hpp"
#include "canio.hpp"
#include "algorithm.hpp"
// #pragma pack (1)
/**
 * @brief 达妙电机状态
 *
 */
enum Enum_Motor_DM_10010_Status
{
    Motor_DM_10010_Status_DISABLE = 0,
    Motor_DM_10010_Status_ENABLE,
};

/**
 * @brief 达妙电机控制状态, 传统模式有效
 *
 */
enum Enum_Motor_DM_10010_Control_Status_Normal
{
    Motor_DM_10010_Control_Status_DISABLE = 0x0,
    Motor_DM_10010_Control_Status_ENABLE,
    Motor_DM_10010_Control_Status_OVERVOLTAGE = 0x8,
    Motor_DM_10010_Control_Status_UNDERVOLTAGE,
    Motor_DM_10010_Control_Status_OVERCURRENT,
    Motor_DM_10010_Control_Status_MOS_OVERTEMPERATURE,
    Motor_DM_10010_Control_Status_ROTOR_OVERTEMPERATURE,
    Motor_DM_10010_Control_Status_LOSE_CONNECTION,
    Motor_DM_10010Control_Status_MOS_OVERLOAD,
};

/**
 * @brief 达妙电机控制方式
 *
 */
enum Enum_Motor_DM_10010_Control_Method
{
    Motor_DM_10010_Control_Method_NORMAL_MIT = 0,
    Motor_DM_10010_Control_Method_NORMAL_ANGLE_OMEGA,
    Motor_DM_10010_Control_Method_NORMAL_OMEGA,
    Motor_DM_10010_Control_Method_NORMAL_FORCE_POSITION,
};

/**
 * @brief 达妙电机传统模式源数据
 *
 */
struct Struct_Motor_DM10010_CAN_Rx_Data_Normal
{
    uint8_t CAN_ID : 4;
    uint8_t Control_Status_Enum : 4;
    uint16_t Angle_Reverse;
    uint8_t Omega_11_4;
    uint8_t Omega_3_0_Torque_11_8;
    uint8_t Torque_7_0;
    uint8_t MOS_Temperature;
    uint8_t Rotor_Temperature;
} __attribute__((packed));

/**
 * @brief 达妙电机常规源数据, MIT控制报文
 *
 */
struct Struct_Motor_DM10010_CAN_Tx_Data_Normal_MIT
{
    uint16_t Control_Angle_Reverse;
    uint8_t Control_Omega_11_4;
    uint8_t Control_Omega_3_0_K_P_11_8;
    uint8_t K_P_7_0;
    uint8_t K_D_11_4;
    uint8_t K_D_3_0_Control_Torque_11_8;
    uint8_t Control_Torque_7_0;
} __attribute__((packed));

/**
 * @brief 达妙电机常规源数据, 位置速度控制报文
 *
 */
struct Struct_Motor_DM10010_CAN_Tx_Data_Normal_Angle_Omega
{
    float Control_Angle;
    float Control_Omega;
} __attribute__((packed));

/**
 * @brief 达妙电机常规源数据, 速度控制报文
 *
 */
struct Struct_Motor_DM10010_CAN_Tx_Data_Normal_Omega
{
    float Control_Omega;
} __attribute__((packed));

/**
 * @brief 达妙电机经过处理的数据, 传统模式有效
 *
 */
struct Struct_Motor_DM_10010_Rx_Data_Normal
{
    Enum_Motor_DM_10010_Control_Status_Normal Control_Status;
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    float Now_MOS_Temperature;
    float Now_Rotor_Temperature;
    uint32_t Pre_Encoder;
    int32_t Total_Encoder;
    int32_t Total_Round;
};



class Class_Motor_DM_10010_Normal {

    public:

    void Init(CAN_HandleTypeDef *hcan, uint8_t __CAN_Rx_ID, uint8_t __CAN_Tx_ID, Enum_Motor_DM_10010_Control_Method __Motor_DM_Control_Method = Motor_DM_10010_Control_Method_NORMAL_MIT, float __Angle_Max = 12.5f, float __Omega_Max = 25.0f, float __Torque_Max = 10.0f, float __Current_Max = 10.261194f);

    inline float Get_Angle_Max();

    inline float Get_Omega_Max();

    inline float Get_Torque_Max();

    inline float Get_Current_Max();

    inline Enum_Motor_DM_10010_Status Get_Status();

    inline Enum_Motor_DM_10010_Control_Status_Normal Get_Control_Status();

    inline float Get_Now_Angle();

    inline float Get_Now_Omega();

    inline float Get_Now_Torque();

    inline float Get_Now_MOS_Temperature();

    inline float Get_Now_Rotor_Temperature();

    inline Enum_Motor_DM_10010_Control_Method Get_Control_Method();

    inline float Get_Control_Angle();

    inline float Get_Control_Omega();

    inline float Get_Control_Torque();

    inline float Get_Control_Current();

    inline float Get_K_P();

    inline float Get_K_D();

    inline void Set_Control_Angle(float __Control_Angle);

    inline void Set_Control_Omega(float __Control_Omega);

    inline void Set_Control_Torque(float __Control_Torque);

    inline void Set_Control_Current(float __Control_Current);

    inline void Set_K_P(float __K_P);

    inline void Set_K_D(float __K_D);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);

    void CAN_Send_Clear_Error();

    void CAN_Send_Enter();

    void CAN_Send_Exit();

    void CAN_Send_Save_Zero();

    void TIM_Alive_PeriodElapsedCallback();

    void TIM_Send_PeriodElapsedCallback();

    void Protect();

protected:
    // 初始化相关变量

    Enum_Motor_DM_10010_Status motor_state_ = Motor_DM_10010_Status_DISABLE;

    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致
    uint16_t CAN_Rx_ID;
    // 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量
    uint16_t CAN_Tx_ID;
    // 最大位置, 与上位机控制幅值PMAX保持一致
    float Angle_Max;
    // 最大速度, 与上位机控制幅值VMAX保持一致
    float Omega_Max;
    // 最大扭矩, 与上位机控制幅值TMAX保持一致
    float Torque_Max;
    // 最大电流, 与上位机串口中上电打印电流保持一致
    float Current_Max;

    // 常量

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t Flag = 0;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;

    // 发送缓冲区
    uint8_t Tx_Data[8];

    // 读变量

    // 电机状态
    Enum_Motor_DM_10010_Status Motor_DM_Status = Motor_DM_10010_Status_DISABLE;
    // 电机对外接口信息
    Struct_Motor_DM_10010_Rx_Data_Normal Rx_Data;

    // 写变量

    // 读写变量

    // 电机控制方式
    Enum_Motor_DM_10010_Control_Method Motor_DM_Control_Method = Motor_DM_10010_Control_Method_NORMAL_MIT;

    // 角度, rad, 目标角度
    float Control_Angle = 0.0f;
    // 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
    float Control_Omega = 0.0f;
    // 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
    float Control_Torque = 0.0f;
    // 电流, A, EMIT模式是限幅, 其余模式无效
    float Control_Current = 0.0f;
    // K_P, 0~500, MIT模式有效
    float K_P = 0.0f;
    // K_D, 0~5, MIT模式有效
    float K_D = 0.0f;

    // 内部函数

    void Data_Process();

    void Output();
};

/**
 * @brief 获取角度最大值
 *
 * @return float 角度最大值
 */
inline float Class_Motor_DM_10010_Normal::Get_Angle_Max()
{
    return (Angle_Max);
}

/**
 * @brief 获取角速度最大值
 *
 * @return float 角速度最大值
 */
inline float Class_Motor_DM_10010_Normal::Get_Omega_Max()
{
    return (Omega_Max);
}

/**
 * @brief 获取扭矩最大值
 *
 * @return float 扭矩最大值
 */
inline float Class_Motor_DM_10010_Normal::Get_Torque_Max()
{
    return (Torque_Max);
}

/**
 * @brief 获取电流最大值
 *
 * @return float 电流最大值
 */
inline float Class_Motor_DM_10010_Normal::Get_Current_Max()
{
    return (Current_Max);
}

/**
 * @brief 获取电机状态
 *
 * @return Enum_Motor_DM_Status 电机状态
 */
inline Enum_Motor_DM_10010_Status Class_Motor_DM_10010_Normal::Get_Status()
{
    return (Motor_DM_Status);
}

/**
 * @brief 获取电机控制状态
 *
 * @return Enum_Motor_DM_Control_Status_Normal 电机控制状态
 */
inline Enum_Motor_DM_10010_Control_Status_Normal Class_Motor_DM_10010_Normal::Get_Control_Status()
{
    return (Rx_Data.Control_Status);
}

/**
 * @brief 获取当前角度
 *
 * @return float 当前角度
 */
inline float Class_Motor_DM_10010_Normal::Get_Now_Angle()
{
    return (Rx_Data.Now_Angle);
}

/**
 * @brief 获取当前角速度
 *
 * @return float 当前角速度
 */
inline float Class_Motor_DM_10010_Normal::Get_Now_Omega()
{
    return (Rx_Data.Now_Omega);
}

/**
 * @brief 获取当前扭矩
 *
 * @return float 当前扭矩
 */
inline float Class_Motor_DM_10010_Normal::Get_Now_Torque()
{
    return (Rx_Data.Now_Torque);
}

/**
 * @brief 获取当前MOS温度
 *
 * @return float 当前MOS温度
 */
inline float Class_Motor_DM_10010_Normal::Get_Now_MOS_Temperature()
{
    return (Rx_Data.Now_MOS_Temperature);
}

/**
 * @brief 获取当前转子温度
 *
 * @return float 当前转子温度
 */
inline float Class_Motor_DM_10010_Normal::Get_Now_Rotor_Temperature()
{
    return (Rx_Data.Now_Rotor_Temperature);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_Motor_DM_Control_Method 电机控制方式
 */
inline Enum_Motor_DM_10010_Control_Method Class_Motor_DM_10010_Normal::Get_Control_Method()
{
    return (Motor_DM_Control_Method);
}

/**
 * @brief 获取角度, rad, 目标角度
 *
 * @return float 角度, rad, 目标角度
 */
inline float Class_Motor_DM_10010_Normal::Get_Control_Angle()
{
    return (Control_Angle);
}

/**
 * @brief 获取角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 *
 * @return float 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 */
inline float Class_Motor_DM_10010_Normal::Get_Control_Omega()
{
    return (Control_Omega);
}

/**
 * @brief 获取扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 *
 * @return float 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 */
inline float Class_Motor_DM_10010_Normal::Get_Control_Torque()
{
    return (Control_Torque);
}

/**
 * @brief 获取电流, A, EMIT模式是限幅, 其余模式无效
 *
 * @return float 电流, A, EMIT模式是限幅, 其余模式无效
 */
inline float Class_Motor_DM_10010_Normal::Get_Control_Current()
{
    return (Control_Current);
}

/**
 * @brief 获取K_P, 0~500, MIT模式有效
 *
 * @return float K_P, 0~500, MIT模式有效
 */
inline float Class_Motor_DM_10010_Normal::Get_K_P()
{
    return (K_P);
}

/**
 * @brief 获取K_D, 0~5, MIT模式有效
 *
 * @return float K_D, 0~5, MIT模式有效
 */
inline float Class_Motor_DM_10010_Normal::Get_K_D()
{
    return (K_D);
}

/**
 * @brief 设定角度, rad, 目标角度
 *
 * @param __Control_Angle 角度, rad, 目标角度
 */
inline void Class_Motor_DM_10010_Normal::Set_Control_Angle(float __Control_Angle)
{
    Control_Angle = __Control_Angle;
}

/**
 * @brief 设定角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 *
 * @param __Control_Omega 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 */
inline void Class_Motor_DM_10010_Normal::Set_Control_Omega(float __Control_Omega)
{
    Control_Omega = __Control_Omega;
}

/**
 * @brief 设定扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 *
 * @param __Control_Torque 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 */
inline void Class_Motor_DM_10010_Normal::Set_Control_Torque(float __Control_Torque)
{
    Control_Torque = __Control_Torque;
}

/**
 * @brief 设定电流, A, EMIT模式是限幅, 其余模式无效
 *
 * @param __Control_Current 电流, A, EMIT模式是限幅, 其余模式无效
 */
inline void Class_Motor_DM_10010_Normal::Set_Control_Current(float __Control_Current)
{
    Control_Current = __Control_Current;
}

/**
 * @brief 设定K_P, 0~500, MIT模式有效
 *
 * @param __K_P K_P, 0~500, MIT模式有效
 */
inline void Class_Motor_DM_10010_Normal::Set_K_P(float __K_P)
{
    K_P = __K_P;
}

/**
 * @brief 设定K_D, 0~5, MIT模式有效
 *
 * @param __K_D K_D, 0~5, MIT模式有效
 */
inline void Class_Motor_DM_10010_Normal::Set_K_D(float __K_D)
{
    K_D = __K_D;
}


extern Class_Motor_DM_10010_Normal DM_Pitch;

#endif //DM10010_HPP
