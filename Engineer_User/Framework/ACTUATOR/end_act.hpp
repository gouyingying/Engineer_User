//
// Created by ZONE7 on 2025/5/2.
//
#include "witimu.hpp"
#include "pidc.hpp"
#include "image_referee.hpp"
#include "DM10010.hpp"
#include "DM4310.hpp"
#include "motorc.hpp"
#include "chassisc.hpp"

#ifndef END_ACT_HPP
#define END_ACT_HPP

#define pitch_offset_g 0.0f
#define pitch_offset_s -90.0f

class end_act {

public:
    pid end_pitch_pid;
    typedef enum
    {
        FREE = 1,   //自由模式，自定义控制器控制
        GOLD = 2,
        SILVER =3,
    } End_eMODE_type;

    void SetEndMode();
    void CrtlLoop();
    void Init();

    End_eMODE_type End_Pitch_Mode_;
    float End_Pitch_Target;
};

void End_Roll_Control();
void End_Pitch_Control();
void Pump_Save();
void Pump_Get();
void Gold_Miner();
void Gold_Miner_Mid();
void Silver_Miner();

extern end_act end_pitch;
extern bool pump_save_flag;
extern bool pump_get_flag;
extern bool gold_miner_flag;
extern bool gold_miner_mid_flag;
extern bool silver_miner_flag;
#endif //END_ACT_HPP
