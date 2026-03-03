//
// Created by pump_li on 2023/9/16.
//
#include "image_referee.hpp"
#include "image_referee_struct.h"
#include "canio.hpp"

#include "send.hpp"
#include "chassisc.hpp"
int pump_s_flag = 0;
int  pump_flag = 0;
int flag_send = 0;

// key_and_mouse_t image_referee_data;
SEND_MESSAGE receive_message;


void image_referee_send_init() {
    receive_message.Head1 = 0x04;
    receive_message.Head2 = 0x01;
    receive_message.Len = 6;
}
uint8_t SelfData_buf[10]{};
int16_t Angle11,Angle12,Angle13,Angle14,Angle15;
cImage_Referee Image_Referee(&huart6, Image_Referee_RecDataBuffDepth, DMA_IDLE_IT);

Image_receive_referee_t Image_referee_recv_mesg;//解包获得的最终数据存在这儿
Image_Referee_UndateFlag_t	Image_Referee_UpdateFlag = {0,0};

static uint32_t DataRecLength = 0;//因为这个变量每次进中断都要用，所以声明为static，不需要每次进来都创建变量，节省一个机器周期

void SelfControler_SetDMASend(uint8_t *Addr, uint16_t HowMany)
{
    HAL_UART_Transmit_DMA(&huart1, Addr, HowMany);
    //使能DMA发送
    __HAL_DMA_ENABLE(huart1.hdmatx);
//    Referee_TxDataBuff.NowDMASendIng = 1;
}
/**
 * @brief 自定义数据发送
 * @param Temp_self_control_all_data
 */
void Self_Control_Transfer(const uint8_t *Temp_self_control_all_data,uint8_t size)
{
    uint8_t temp_data[3]={};
    //自定义控制器，最长一次发30个字节，
    static RefereeFullFrame_t *Send_Pack = NULL;
    static uint8_t SendSEQ = 0;//包序号

    // Send_Pack = (RefereeFullFrame_t *) Send_Pack_buf;
    ++SendSEQ;
    Send_Pack->header.SOF = FrameHeaderSOF;//数据帧起始字节
    Send_Pack->header.DataLength = size+9;//包头总数据长度
    Send_Pack->header.Seq = SendSEQ;//包序号
    Send_Pack->header.CRC8 = GetCRC8CheckSum(&(Send_Pack->header.SOF), 4, CRC8_INIT_NUM);//CRC8校验
    Send_Pack->frame_cmd_id = SELF_CONTROL_INTERACTIVE_ID;//自定义控制器交互ID

    memcpy(Send_Pack->Data,Temp_self_control_all_data,size);
    //CRC16整包校验
    *((uint16_t *) (Send_Pack->Data + size)) = GetCRC16CheckSum(&(Send_Pack->header.SOF),7 + size, CRC16_INIT_NUM);

    // SelfControler_SetDMASend(Send_Pack_buf,Send_Pack->header.DataLength);
}
/**
 * @brief 裁判系统协议校验
 * @param StartByte
 * @return 校验完成数据包帧头
 */
uint8_t* Referee_Check_Frame(uint8_t* StartByte)
{
    static uint8_t *Start_Flag_Byte = NULL;//储存帧首地址位置
    static uint8_t CRC8_Result = 0;
    static uint16_t CRC16_Result = 0;
    Start_Flag_Byte = (uint8_t*)strchr((const char*)StartByte,0xA5);//字符串内寻找第0xA5，返回其地址
    if(Start_Flag_Byte == NULL)//未接收到开始帧返回NULL
    {
        return NULL;
    }
    else
    {
        CRC8_Result = GetCRC8CheckSum(Start_Flag_Byte,4,CRC8_INIT_NUM);//CRC包头校验
        if(CRC8_Result == Start_Flag_Byte[4])
        {
            CRC16_Result = GetCRC16CheckSum(Start_Flag_Byte,7 + (uint16_t)*(uint16_t*)&Start_Flag_Byte[1],CRC16_INIT_NUM);
            if(CRC16_Result == (uint16_t)*(uint16_t*)&Start_Flag_Byte[ 7 + (uint16_t)*(uint16_t*)&Start_Flag_Byte[1] ])//CRC包尾校验
            {
                return Start_Flag_Byte;
            }
        }
    }
}

/**
 * @brief 裁判系统协议校验
 * @param StartByte
 * @return 校验完成数据包帧头
 */
uint8_t* Referee_Check_Frame_C5(uint8_t* StartByte)
{
    static uint8_t *Start_Flag_Byte = NULL;//储存帧首地址位置
    static uint16_t CRC16_Result = 0;
    Start_Flag_Byte = (uint8_t*)strchr((const char*)StartByte,0xA5);//字符串内寻找第0xA5，返回其地址
    if(Start_Flag_Byte == NULL)//未接收到开始帧返回NULL
    {
        return NULL;
    }
    else if( Start_Flag_Byte[1] == 0x53)
        {
            CRC16_Result = GetCRC16CheckSum(Start_Flag_Byte,19,CRC16_INIT_NUM);
            if(CRC16_Result == (uint16_t)*(uint16_t*)&Start_Flag_Byte[ 19 ])//CRC包尾校验
            {
                return Start_Flag_Byte;
            }
        }

}


void C5_Image_Referee_UnpackRecData(uint16_t DataLength,uint8_t* buf_Data)
{
    // usart_printf("DataLength:%d\r\n",DataLength);
    static uint8_t *Referee_Pack = NULL;
    Referee_Pack = buf_Data;
    while (Referee_Pack != NULL)
    {
            switch ( ((C5_Imgae_Frame_t*)Referee_Pack)->sof )
            {

            case C5_ImageSof:
                {
                    C5_GetNewData.live_time = 100;
                    image_ctrl.rc.ch[0] = (int16_t)(( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[0] | (( C5_Imgae_Frame_t* )Referee_Pack)->Data[1] << 8)& 0x07ff;
                    image_ctrl.rc.ch[1] = (int16_t)(( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[1] >> 3 | (( C5_Imgae_Frame_t* )Referee_Pack)->Data[2] << 5)& 0x07ff;
                    image_ctrl.rc.ch[2] = (int16_t)(( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[2] >> 6 | (( C5_Imgae_Frame_t* )Referee_Pack)->Data[3] << 2 |
                                                        (( C5_Imgae_Frame_t* )Referee_Pack)->Data[4] << 10 )& 0x07ff;
                    image_ctrl.rc.ch[3] = (int16_t)(( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[4] >> 1 | (( C5_Imgae_Frame_t* )Referee_Pack)->Data[5] << 7)& 0x07ff;
                    image_ctrl.rc.s[0] =  ((( C5_Imgae_Frame_t* )Referee_Pack)->Data[5] >> 4) & 0x0003;
                    image_ctrl.rc.s[1] =  ((( C5_Imgae_Frame_t* )Referee_Pack)->Data[5] >> 6) & 0x0001;
                    image_ctrl.rc.s[2] =  ((( C5_Imgae_Frame_t* )Referee_Pack)->Data[5] >> 7) & 0x0001;
                    image_ctrl.rc.s[3] =  (( C5_Imgae_Frame_t* )Referee_Pack)->Data[6] & 0x0001;
                    image_ctrl.rc.ch[4] = (int16_t)(( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[6] >> 1 | (( C5_Imgae_Frame_t* )Referee_Pack)->Data[7] << 7)& 0x07ff;
                    image_ctrl.rc.s[4] =  ((( C5_Imgae_Frame_t* )Referee_Pack)->Data[7] >> 4) & 0x0001;
                    image_ctrl.Key_Mouse_raw.mouse_x = (int16_t)( ( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[8]  | ( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[9] << 8) & 0xffff;
                    image_ctrl.Key_Mouse_raw.mouse_y = (int16_t)( ( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[10]  | ( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[11] << 8) & 0xffff;
                    image_ctrl.Key_Mouse_raw.mouse_z = (int16_t)( ( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[12]  | ( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[13] << 8) & 0xffff;
                    image_ctrl.Key_Mouse_raw.left_button_down = ( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[14] & 0x0001;
                    image_ctrl.Key_Mouse_raw.right_button_down = ((( C5_Imgae_Frame_t* )Referee_Pack)->Data[14]>> 2) & 0x0001;
                    image_ctrl.Key_Mouse_raw.middle_button_down = ((( C5_Imgae_Frame_t* )Referee_Pack)->Data[14] >> 4) & 0x0001;
                    image_ctrl.Key_Mouse_raw.keyboard_value = (int16_t)( ( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[15]  | ( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[16] << 8) & 0xffff;

                    image_ctrl.rc.ch[0] -= C5_Remotec_Offset;
                    image_ctrl.rc.ch[1] -= C5_Remotec_Offset;
                    image_ctrl.rc.ch[2] -= C5_Remotec_Offset;
                    image_ctrl.rc.ch[3] -= C5_Remotec_Offset;
                    image_ctrl.rc.ch[4] -= C5_Remotec_Offset;
                    // usart_printf("%d\r\n",image_referee_data.keyboard_value);

                    Image_Referee_KEY_AND_MOUSE_Slove();
                    Image_Key_PortHandle();
                    key_move_control();
                    key_pump_control();
                }
            }
            break;

    }
    if(21 < DataLength)
    {//说明缓存区内数据比一帧数据要长
        *Referee_Pack = 0xFF;//把处理过的帧破坏掉，一定要加
        (( C5_Imgae_Frame_t* )Referee_Pack)->sof = 0x00;
        Referee_Pack += 21;
        Referee_Pack = Referee_Check_Frame_C5(Referee_Pack);
    }
    else
    {
        Referee_Pack = NULL;
    }
}
/**
 * @brief 数据解包
 * @param DataLength 传入数据长度
 * @param buf_Data DMA缓存数组
 */
void Image_Referee_UnpackRecData(uint16_t DataLength,uint8_t* buf_Data)
{
    // usart_printf("DataLength:%d\r\n",DataLength);
    static uint8_t *Referee_Pack = NULL;
    Referee_Pack = Referee_Check_Frame(buf_Data);
    while (Referee_Pack != NULL)
    {

        switch ( ((RefereeFullFrame_t*)Referee_Pack)->frame_cmd_id )
        {
            case SELF_CONTROL_INTERACTIVE_ID:
            {
                    // usart_printf("SELF_CONTROL_INTERACTIVE_ID\r\n");
                memset((char*)&Image_referee_recv_mesg.self_control_all_data,0,sizeof(self_control_data_t));
                memcpy((char*)&Image_referee_recv_mesg.self_control_all_data,(char*)&( (RefereeFullFrame_t*)Referee_Pack)->Data,((RefereeFullFrame_t *)Referee_Pack)->header.DataLength);
                ++ Image_Referee_UpdateFlag.self_control_all_data;
                    memcpy(&self_ctrl.yaw_4310,&Image_referee_recv_mesg.self_control_all_data.Data[0],4);
                    memcpy(&self_ctrl.pitch_10010,&Image_referee_recv_mesg.self_control_all_data.Data[4],4);
                    memcpy(&self_ctrl.pitch_3508,&Image_referee_recv_mesg.self_control_all_data.Data[8],4);
                    memcpy(&self_ctrl.roll_3508,&Image_referee_recv_mesg.self_control_all_data.Data[12],4);
                    memcpy(&self_ctrl.pitch_4310,&Image_referee_recv_mesg.self_control_all_data.Data[16],4);
                    memcpy(&self_ctrl.roll_2006,&Image_referee_recv_mesg.self_control_all_data.Data[20],4);

            }
            break;
            case KEY_AND_MOUSE_ID:
            {
                memcpy((char*)&Image_referee_recv_mesg.key_and_mouse,(char*)&( ( RefereeFullFrame_t* )Referee_Pack)->Data,sizeof(key_and_mouse_t));
                ++ Image_Referee_UpdateFlag.key_and_mouse;
                    // usart_printf("Key_And_Mouse_ID\r\n");

                    // usart_printf("%d\r\n",image_referee_data.keyboard_value);

                    }

            }
            break;
        }
        if(( ( RefereeFullFrame_t* )Referee_Pack)->header.DataLength +  9 < DataLength)
        {//说明缓存区内数据比一帧数据要长
            *Referee_Pack = 0xFF;//把处理过的帧破坏掉，一定要加
            (( RefereeFullFrame_t* )Referee_Pack)->frame_cmd_id = Clear_Flag;
            Referee_Pack += 9 + (( RefereeFullFrame_t* )Referee_Pack)->header.DataLength;
            Referee_Pack = Referee_Check_Frame(Referee_Pack);
        }
        else
        {
            Referee_Pack = NULL;
        }


        // // usart_printf("Self_Control_Interactive_ID\r\n");
        // usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",self_ctrl.yaw_4310,self_ctrl.pitch_10010,self_ctrl.pitch_3508,self_ctrl.roll_3508,self_ctrl.pitch_4310,self_ctrl.roll_2006);

        // SelfControler_SetDMASend((uint8_t *)&Image_referee_recv_mesg.self_control_all_data,sizeof(self_control_data_t));

}
uint8_t it_flag = 0;
void cImage_Referee::rxUserCALLBACK()
{
    DataRecLength = Image_Referee_RecDataBuffDepth -
                     __HAL_DMA_GET_COUNTER(which_uart_->hdmarx);//得到当前DMA收到了多少字节
    if(rx_buf_[0]== 0xA5)
    {
        Image_Referee_UnpackRecData(DataRecLength,rx_buf_);
        // usart_printf("0xA5\r\n");
    }
    else if(rx_buf_[0]== 0xA9)
    {
        C5_Image_Referee_UnpackRecData(DataRecLength,rx_buf_);
    }

    memset(rx_buf_,0,DataRecLength);
    DataRecLength = 0;
    it_flag = 1;
    // usart_printf("11111111111\r\n");
}

/**
 * 检查按键是否按下
 * @param which_key
 * @return
 */
bool Check_Key(uint16_t which_key)
{
    if(image_ctrl.Key_Mouse_raw.keyboard_value & which_key)
        return true;
    else
        return false;
}

void key_move_control()
{
    if(image_ctrl.key.W.Is_Click_Hold)
        k_vy = 0.5;
    else if(image_ctrl.key.S.Is_Click_Hold)
        k_vy = -0.5;
    else
        k_vy = 0;
    if(image_ctrl.key.A.Is_Click_Hold)
        k_vx = -0.5;
    else if(image_ctrl.key.D.Is_Click_Hold)
        k_vx = 0.5;
    else
        k_vx = 0;
    if(image_ctrl.key.SHIFT.Is_Click_Hold)
    {
        K_speed = 2;
    }
    else
    {
        K_speed = 1;
    }
    k_vz = image_ctrl.mouse.mouse_x;
    K_angle = image_ctrl.mouse.mouse_y;

}

void key_pump_control()
{

    if((image_ctrl.key.E.Is_Click_Once == 1 ) && image_ctrl.key.E.Now_State == 1 && image_ctrl.key.E.Last_State == 1)
    {
        pump_arm = 1;
        // send_message.Data[7] = 0x55;
    }
    else if((image_ctrl.key.E.Is_Click_Once == 0) && image_ctrl.key.E.Now_State == 1 && image_ctrl.key.E.Last_State == 1)
    {
        pump_arm = 0;
        // send_message.Data[7] = 0x00;
    }
    if(image_ctrl.key.R.Is_Click_Once == 1 && image_ctrl.key.R.Now_State == 1 && image_ctrl.key.R.Last_State == 1)
    {
        pump_save = 1;
        // send_message.Data[8] = 0xAA;
    }
    else if(image_ctrl.key.R.Is_Click_Once == 0 && image_ctrl.key.R.Now_State == 1 && image_ctrl.key.R.Last_State == 1)
    {
        pump_save = 0;
        // send_message.Data[8] = 0x00;
    }
        // usart_printf("%d,%d,%d\r\n",image_ctrl.key.E.Is_Click_Once,pump_flag,send_message.Data[7]);

}

void Image_Last_reserved(Key_and_Mouse_t_click* port)
{
    port->Last_State = port->Now_State;
}

void Image_KEY_Click_Slove(Key_and_Mouse_t_click* port)
{
    if (port->Now_State == 1 && port->Last_State == 0)
    {
        if (port->Is_Click_Once == 1)
            port->Is_Click_Once = 0;
        else
            port->Is_Click_Once = 1;

    }
    port->Is_Click_Hold = port->Now_State;
    // usart_printf("%d,%d\r\n",port->Is_Click_Hold,port->Is_Click_Once);
    port->Last_State = port->Now_State;


}

void Image_Referee_KEY_AND_MOUSE_Slove()
{
    image_ctrl.rc.Middle_Key = image_ctrl.rc.s[0];
    image_ctrl.rc.Pause_Key.Now_State = image_ctrl.rc.s[1];
    image_ctrl.rc.User_Key_L.Now_State = image_ctrl.rc.s[2];
    image_ctrl.rc.User_Key_R.Now_State = image_ctrl.rc.s[3];
    image_ctrl.rc.Rammc_Key =image_ctrl.rc.ch[4];
    image_ctrl.rc.Trigger_Key.Now_State = image_ctrl.rc.s[4];

    image_ctrl.mouse.mouse_x = image_ctrl.Key_Mouse_raw.mouse_x;
    image_ctrl.mouse.mouse_y = image_ctrl.Key_Mouse_raw.mouse_y;
    image_ctrl.mouse.mouse_z = image_ctrl.Key_Mouse_raw.mouse_z;
    image_ctrl.mouse.left_button_down.Now_State = image_ctrl.Key_Mouse_raw.left_button_down;
    image_ctrl.mouse.right_button_down.Now_State = image_ctrl.Key_Mouse_raw.right_button_down;
    image_ctrl.mouse.middle_button_down.Now_State = image_ctrl.Key_Mouse_raw.middle_button_down;

    image_ctrl.key.W.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_W);
    image_ctrl.key.S.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_S) >> 1;
    image_ctrl.key.A.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_A) >> 2;
    image_ctrl.key.D.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_D) >> 3;;
    image_ctrl.key.SHIFT.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_SHIFT) >> 4;
    image_ctrl.key.CONTRL.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_CTRL) >> 5;
    image_ctrl.key.Q.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_Q) >> 6;
    image_ctrl.key.E.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_E) >> 7;
    image_ctrl.key.R.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_R) >> 8;
    image_ctrl.key.F.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_F) >> 9;
    image_ctrl.key.G.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_G) >> 10;
    image_ctrl.key.Z.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_Z) >> 11;
    image_ctrl.key.X.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_X) >> 12;
    image_ctrl.key.C.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_C) >> 13;
    image_ctrl.key.V.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_V) >> 14;
    image_ctrl.key.B.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_B) >> 15;


}


void Image_Key_PortHandle(){
    Image_KEY_Click_Slove(&image_ctrl.mouse.left_button_down);
    Image_KEY_Click_Slove(&image_ctrl.mouse.middle_button_down);
    Image_KEY_Click_Slove(&image_ctrl.mouse.right_button_down);

    Image_KEY_Click_Slove(&image_ctrl.rc.Pause_Key);
    Image_KEY_Click_Slove(&image_ctrl.rc.User_Key_L);
    Image_KEY_Click_Slove(&image_ctrl.rc.User_Key_R);
    Image_KEY_Click_Slove(&image_ctrl.rc.Trigger_Key);

    Image_KEY_Click_Slove(&image_ctrl.key.W);
    Image_KEY_Click_Slove(&image_ctrl.key.S);
    Image_KEY_Click_Slove(&image_ctrl.key.A);
    Image_KEY_Click_Slove(&image_ctrl.key.D);
    Image_KEY_Click_Slove(&image_ctrl.key.SHIFT);
    Image_KEY_Click_Slove(&image_ctrl.key.CONTRL);
    Image_KEY_Click_Slove(&image_ctrl.key.Q);
    Image_KEY_Click_Slove(&image_ctrl.key.E);
    Image_KEY_Click_Slove(&image_ctrl.key.R);
    Image_KEY_Click_Slove(&image_ctrl.key.F);
    Image_KEY_Click_Slove(&image_ctrl.key.G);
    Image_KEY_Click_Slove(&image_ctrl.key.Z);
    Image_KEY_Click_Slove(&image_ctrl.key.X);
    Image_KEY_Click_Slove(&image_ctrl.key.C);
    Image_KEY_Click_Slove(&image_ctrl.key.V);
    Image_KEY_Click_Slove(&image_ctrl.key.B);
}

void Self_Control_solve()
{

}

/**
 * 保护检测 如果遥控器指定保护或遥控器断连
 * @return Car.is_protect true
 */
bool C5_portSetProtect()
{
    bool is_protect;
    C5_GetNewData.live_time--;
    if (C5_GetNewData.live_time <= 0)
        is_protect = true;
    else is_protect = false;
    return is_protect;
}

Image_ctrl_t Get_Image_Ctrl()
{
    return image_ctrl;
}


