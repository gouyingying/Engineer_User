//
// Created by ZONE7 on 2025/1/19.
//

#ifndef CANIO_BASIC_HPP
#define CANIO_BASIC_HPP

#include "can.h"

#define CAN_CHASSIS_ALL_ID  0x200
#define CAN_3508_M1_ID  0x201
#define CAN_3508_M2_ID  0x202
#define CAN_3508_M3_ID  0x203
#define CAN_3508_M4_ID  0x204


#define CAN_RECEIVE_VEL_ID 0x401
#define CAN_RECEIVE_AGL_ID 0x402

#define CAN_SEND_SCAP_ID 0x02E
#define CAN_SET_SDCAP_ID 0x02F
#define CAN_RECEIVE_SCAP_ID 0x030

#define CAN_REFEREE_REC1_ID 0x405
#define CAN_REFEREE_REC2_ID 0x407
#define CAN_REFEREE_REC3_ID 0x406

typedef struct
{
    bool online;
    int32_t times;
}eOnlineState;

void CAN_Filter_Init(CAN_HandleTypeDef *hcan);

class canio_basic {

};



#endif //CANIO_BASIC_HPP
