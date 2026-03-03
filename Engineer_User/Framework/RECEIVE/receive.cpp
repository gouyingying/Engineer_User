//
// Created by ZONE7 on 2024/10/16.
//

#include "receive.hpp"
#include "image_referee.hpp"
#include "send.hpp"

// int pump_flag = 0;

float last_receive_buf1,last_receive_buf2,last_receive_buf3,last_receive_buf4,last_receive_buf5,last_receive_buf6;

receive receive_from_self_control(&huart3, 255, DMA_IDLE_IT);


receive::receive(UART_HandleTypeDef *uart, uint16_t size, eUSART_type type):cUSARTC(uart, size, type)
{}


void receive::rxUserCALLBACK()
{
    memcpy(receive_rx_buf_, rx_buf_, 29);
    //解包预处理
    //将串口收到的数据进行处理，新的数组以数字开头，便于之后字符转浮点数的运算
    memcpy(receive_buf_, &receive_rx_buf_[0], 29);
    Gyro_Solve();

    memset(receive_buf_,0,29);
}

void receive::Gyro_Solve()
{
    if(receive_buf_[0] == 0x03 && receive_buf_[1] == 0x02 )
    {
        last_receive_buf1 = receive_buf1;
        last_receive_buf2 = receive_buf2;
        last_receive_buf3 = receive_buf3;
        last_receive_buf4 = receive_buf4;
        last_receive_buf5 = receive_buf5;
        last_receive_buf6 = receive_buf6;
        memcpy(&receive_buf1,&receive_buf_[3],4);
        memcpy(&receive_buf2,&receive_buf_[7],4);
        memcpy(&receive_buf3,&receive_buf_[11],4);
        memcpy(&receive_buf4,&receive_buf_[15],4);
        memcpy(&receive_buf5,&receive_buf_[19],4);
        memcpy(&receive_buf6,&receive_buf_[23],4);
// usart_printf("receive_buf1:%.2f,receive_buf2:%.2f,receive_buf3:%.2f,receive_buf4:%.2f,receive_buf5:%.2f,receive_buf6:%.2f\r\n",receive_buf1,receive_buf2,receive_buf3,receive_buf4,receive_buf5,receive_buf6);



    // us
    // art_printf("111111111\r\n");

    // usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\r\n",angle1_init,angle2_init,angle3_init,angle4_init,angle5_init,angle6_init,angle_init_flag);
        switch(receive_buf_[27])
        {
            case 0x55:
                pump_flag = 1;
                send_message.Data[7] = 0x55;
                // usart_printf("pump_flag:%d\r\n",pump_flag);
                break;
            case 0x00:
                pump_flag = 0;
                send_message.Data[7] = 0x00;
                // usart_printf("pump_flag:%d\r\n",pump_flag);
                break;
            default:
                break;
        }
    }
    switch(receive_buf_[28])
    {
        case 0xAA:
            send_message.Data[8] = 0xAA;
            // usart_printf("pump_s_flag:%d\r\n",pump_s_flag);
            break;
        case 0x00:
            send_message.Data[8] = 0x00;
            // usart_printf("pump_s_flag:%d\r\n",pump_s_flag);
            break;
        default:
            break;
    }


}

float receive::Get_Angle1()
{

    // if(receive_buf1 > 270||receive_buf1 < -270)
    // {
    //     return 0;
    // }
    // else
    // {
        return (receive_buf1);
    // }
}

float receive::Get_Angle2()
{
    // if(receive_buf2 > 270||receive_buf2 < -270)
    // {
    //     return 0;
    // }
    // else
    // {
        return (receive_buf2);
    // }
}

float receive::Get_Angle3()
{
    // if(receive_buf3 > 270||receive_buf3 < -270)
    // {
    //     return 0;
    // }
    // else
    // {
        return (receive_buf3);
    // }
}

float receive::Get_Angle4()
{
    // if(receive_buf4 > 270||receive_buf4 < -270)
    // {
    //     return 0;
    // }
    // else
    // {
        return (receive_buf4);
    // }
}

float receive::Get_Angle5()
{
    // if(receive_buf5 > 270||receive_buf5 < -270)
    // {
    //     return 0;
    // }
    // else
    // {
        return (receive_buf5);
    // }
}

float receive::Get_Angle6()
{

    // if(receive_buf6 > 180||receive_buf6 < -180)
    // {
    //     return 0;
    // }
    // else
    // {
        return (receive_buf6);
    // }
}