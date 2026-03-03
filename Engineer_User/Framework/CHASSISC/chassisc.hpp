//
// Created by 24481 on 24-10-8.
//

#ifndef CHASSISC_HPP
#define CHASSISC_HPP

#include "main.h"
#include "remotec.hpp"
#include "gimbal.hpp"
#include "pidc.hpp"
#include "can.h"
#define SEND_LEN 8
class cChassisC {
public:
    pid chassis_yaw_;
    typedef enum
    {
        FREE = 1,   //云台自由模式，底盘不动
        SPIN = 2,   //匀速小陀螺模式
        FOLLOW = 3, //随动模式
        TWIST = 4,  //变速小陀螺模式
        PROTECT = 5,//保护模式
        WARNING = 6 //警告模式
    } eMODE_type;
    // cChassisC();
    typedef struct
    {
        uint8_t  Head1;                 //帧头1
        uint8_t  Head2;                 //帧头2
        uint8_t  Len;                   //长度
        uint8_t Data[SEND_LEN];   //数据
        uint8_t Cmd;              //命令
        // uint16_t CRC16;                 //CRC校验
    }SEND_CHASSIS_MESSAGE;


    class send {

    };




    void send_chassis_init();
    void SetCarMode();
    void ComLoop();
    void Init();
    void protect();
    void getCtrlData();
    void canSend();
    void rx_callback(uint8_t *Rx_Data);
public:
    float vx=0;
    float vy=0;
    float vz=0;
    eMODE_type CarMode_;
    uint8_t Last_CarMode_=0;
    uint8_t flag = 0;
    float ChassisYawTarget=-107.0f;
    float TuoluoDiredtion = -107.0f;
    float yaw_offset=0.0f;
    int8_t redraw_status=0;
};
void CAN_ChasisSendSpd(int16_t vx, int16_t vy, int16_t vz, int8_t car_mode, int8_t is_aimbot);
void CAN_ChasisSendMsg(int16_t yaw, int16_t pitch, int8_t servo_status, int8_t power_status, int8_t rammer_status,
                       int8_t redraw_status);
void angle_1_2_rx_callback(uint8_t *Rx_Data);
void angle_3_4_rx_callback(uint8_t *Rx_Data);
void angle_5_6_rx_callback(uint8_t *Rx_Data);

extern cChassisC chassis_com;
extern int pump_arm,pump_save;

#endif //CHASSISC_HPP
