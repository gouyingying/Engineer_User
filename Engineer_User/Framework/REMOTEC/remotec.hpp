//
// Created by ZONE7 on 2024/9/28.
//

#ifndef REMOTEC_HPP
#define REMOTEC_HPP



#include <cstdint>
#include <cctype>

#include "image_referee_struct.h"
#include "usartio.hpp"


#define TIMpiece 0.005
#define LIMIT_VALUE(x,min,max)  x = (((x)<(min)) ? (min) : (((x)>(max)) ? (max) : (x)))
static float mouse_sense = TIMpiece * 2.5;  //鼠标灵敏度
#define SBUS_RX_BUF_SIZE 255

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */

typedef struct
{
    struct
    {
        int16_t ch[5];
        char s[2];
    } __attribute__((packed)) rc;
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } __attribute__((packed)) mouse;
     struct
    {
        uint16_t v;
    } __attribute__((packed)) key;

} __attribute__((packed)) sRemoteInfo;

class cREMOTEC:public cUSARTC
{
public:
    cREMOTEC(UART_HandleTypeDef *uart,uint16_t size,eUSART_type type):
            cUSARTC(uart,size,type){};
    void rxUserCALLBACK() override;
    sRemoteInfo Get_Remote();
private:
    sRemoteInfo rc_ctrl{};//遥控器控制变量
};

typedef struct {
    int64_t live_time;
} Online_detect_t;   //检测遥控器是否断连

typedef struct {
    int64_t live_time;
} C5_Online_detect_t;   //检测遥控器是否断连

typedef enum
{
    eRC,
    eKey,
    image
} eCtrlMode;//控制方式

typedef enum
{
    Clik_Once = 0,
    Clik_Hold = 1,
} click_mode_e;


typedef __packed struct {
    uint8_t Last_State: 1;
    uint8_t Now_State: 1;
    uint8_t Is_Click_Once: 1;       //“：1”声明后该变量只有0和1
    bool Is_Click_Hold;
    click_mode_e click_mode;
} Key_and_Mouse_t_click;

typedef __packed struct
{
    __packed struct {
        int16_t mouse_x;//鼠标x轴，左负右正
        int16_t mouse_y;//鼠标y轴，上负下正
        int16_t mouse_z;//鼠标滚轮，前正后负
        Key_and_Mouse_t_click left_button_down;//左键
        Key_and_Mouse_t_click middle_button_down;
        Key_and_Mouse_t_click right_button_down;//右键
    } mouse;
    __packed struct{
        int16_t ch[5];
        uint8_t s[5];
        char Middle_Key;
        Key_and_Mouse_t_click Pause_Key;
        Key_and_Mouse_t_click User_Key_L;
        Key_and_Mouse_t_click User_Key_R;
        int16_t Rammc_Key;
        Key_and_Mouse_t_click Trigger_Key;
    } rc;
    __packed struct {
        uint16_t keyboard_value;
        Key_and_Mouse_t_click W;
        Key_and_Mouse_t_click S;
        Key_and_Mouse_t_click A;
        Key_and_Mouse_t_click D;
        Key_and_Mouse_t_click SHIFT;

        Key_and_Mouse_t_click CONTRL;
        Key_and_Mouse_t_click Q;
        Key_and_Mouse_t_click E;
        Key_and_Mouse_t_click R;
        Key_and_Mouse_t_click F;
        Key_and_Mouse_t_click G;
        Key_and_Mouse_t_click Z;
        Key_and_Mouse_t_click X;
        Key_and_Mouse_t_click C;
        Key_and_Mouse_t_click V;
        Key_and_Mouse_t_click B;
    } key;
    key_and_mouse_t Key_Mouse_raw;
} Image_ctrl_t;

float portSetVx(void);
float portSetVy(void);
float portSetAngle(void);
float portSetAngle_Z(void);
void RC_Levers();
bool portSetProtect();
float Limit_Angle(float tar_get_,float min,float max);

self_control_t Get_Self_Ctrl();

extern cREMOTEC remote;
extern eCtrlMode CtrlMode;
extern float k_vx;
extern float k_vy;
extern float k_vz;
extern float K_angle;
extern float K_speed;
extern Image_ctrl_t image_ctrl;
extern C5_Online_detect_t C5_GetNewData;
extern self_control_t self_ctrl;
#endif //REMOTEC_HPP
