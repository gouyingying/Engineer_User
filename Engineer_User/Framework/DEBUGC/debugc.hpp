//
// Created by ZONE7 on 2024/9/28.
//

#ifndef DEBUGC_HPP
#define DEBUGC_HPP
#include "main.h"
#include "usartio.hpp"
#include <cstdlib>
#include <cstring>

#define DEBUG_RVSIZE 255
typedef struct
{
    float vel_kp;
    float vel_ki;
    float vel_kf;
    float vel_rampstep;
    float vel_tarvalue;

    float pos_kp;
    float pos_kd;
    float pos_kf;
    float pos_maxstep;
    float pos_tarvalue;
}DebugParam;


#define MAOHAO  0x3A

#define VEL_LOOP 0x73
#define VEL_KP 0x70            //s_kp
#define VEL_KI 0x69            //s_ki
#define VEL_KF 0x66            //s_kf
#define VEL_RAMPSTEP 0x73       //s_ts
#define VEL_TARVALUE 0x76      //s_tv

#define POS_LOOP 0x70
#define POS_KP 0x70            //p_kp
#define POS_KD 0x64            //p_kd
#define POS_KF 0x66            //p_kf
#define POS_MAXSTEP 0x73       //p_ms
#define POS_TARVALUE 0x76      //p_tv

class cDEBUGC:public cUSARTC
{
public:
    cDEBUGC(UART_HandleTypeDef *uart,uint16_t size,eUSART_type type);
    void rxUserCALLBACK() override;
    DebugParam& Param();
private:
    static DebugParam Debug_Param;
    char debug_rx_buf_[DEBUG_RVSIZE]{};
    char debug_buf_[DEBUG_RVSIZE]{};
    char *p_end_{};
};

extern cDEBUGC debug;
extern DebugParam &param;
#endif //DEBUGC_HPP
