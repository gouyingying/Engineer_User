//
// Created by ZONE7 on 2024/11/16.
//

#include "witimu.hpp"
#include "ledio.hpp"

witimu wit(&huart3, 255,DMA_IDLE_IT);


// void USART_RxCallback(uint8_t *rx_buf){};


witimu::witimu(UART_HandleTypeDef *uart, uint16_t size, eUSART_type type):cUSARTC(uart, size, type)
{}

void witimu::rxUserCALLBACK()
{
    memcpy(wit_rx_buf_, rx_buf_, WITIMU_RVSIZE);
    //解包预处理
    //将串口收到的数据进行处理，新的数组以数字开头，便于之后字符转浮点数的运算
    memcpy(wit_buf_, &wit_rx_buf_[0], 33);
    IMU_Data_Handler();

    memset(wit_buf_,0,33);
}

/**
 * @name   IMU_AngleIncreLoop
 * @brief  IMU角度连续化处理
 * @param  angle_now 当前IMU反馈角度
 * @retval angle_now 连续化后的IMU反馈角度
 */
float witimu::IMU_AngleIncreLoop(float angle_now)
{
    float this_angle;
    this_angle = angle_now;
    if ((this_angle - last_angle) > 300)
        rotate_times--;
    if ((this_angle - last_angle) < -300)
        rotate_times++;
    angle_now = this_angle + rotate_times * 360.0f;
    last_angle = this_angle;
    return angle_now;
}
/*解包计算编码器角度值
 *
 */
void witimu::IMU_Data_Handler()
{
     for (int8_t i = 0; i < 3; i++)
    {
        if (wit_buf_[11 * i] != IMU_HEAD)
        {
            // usart_printf("IMU_HEAD error\r\n");
            return;
        }

        switch (wit_buf_[i * 11 + 1])
        {
            case IMU_SPEED:
                speed[0] = (short) (((short) wit_buf_[i * 11 + 3] << 8) | wit_buf_[i * 11 + 2]) * 2000.0f / 32768.0f;
                speed[1] = (short) (((short) wit_buf_[i * 11 + 5] << 8) | wit_buf_[i * 11 + 4]) * 2000.0f / 32768.0f;
                speed[2] = (short) (((short) wit_buf_[i * 11 + 7] << 8) | wit_buf_[i * 11 + 6]) * 2000.0f / 32768.0f;
                Pitch_Speed = speed[0];
                Roll_Speed = speed[1];
                Yaw_Speed = speed[2];
                //usart_printf("%f,%f\r\n",Pih_Speed,Yaw_Speed);
                break;
            case IMU_ANGLE: //√
                angle[0] = (short) (((short) wit_buf_[i * 11 + 3] << 8) | wit_buf_[i * 11 + 2]) * 180.0f / 32768.0f;
                angle[1] = (short) (((short) wit_buf_[i * 11 + 5] << 8) | wit_buf_[i * 11 + 4]) * 180.0f / 32768.0f;
                angle[2] = (short) (((short) wit_buf_[i * 11 + 7] << 8) | wit_buf_[i * 11 + 6]) * 180.0f / 32768.0f;
//                NaiveAngle.roll = angle[1];
//                NaiveAngle.pitch = -angle[0]; //上位机在这里解算有问题
//                NaiveAngle.yaw = angle[2];

                Pitch_Angle = angle[0]; //与编码器方向一致
                Roll_Angle = angle[1];
                Yaw_Angle = IMU_AngleIncreLoop(angle[2]);//与编码器方向是一致的，自瞄时出了问题
                // usart_printf("%f,%f,%f\r\n",Pitch_Angle,Roll_Angle,Yaw_Angle);

                break;
            case IMU_QUATERNION:
//                quaternion.w = (short) (((short) IMU_RxBuf[i * 11 + 3] << 8) | IMU_RxBuf[i * 11 + 2]) / 32768.0f;
//                quaternion.x = (short) (((short) IMU_RxBuf[i * 11 + 5] << 8) | IMU_RxBuf[i * 11 + 4]) / 32768.0f;
//                quaternion.y = (short) (((short) IMU_RxBuf[i * 11 + 7] << 8) | IMU_RxBuf[i * 11 + 6]) / 32768.0f;
//                quaternion.z = (short) (((short) IMU_RxBuf[i * 11 + 9] << 8) | IMU_RxBuf[i * 11 + 8]) / 32768.0f;
                break;
        }
    }

}

float witimu::WIT_IMU_Angle(IMU_data Witch_angle)
{
    switch (Witch_angle)
    {
    case imu_yaw:
        return Yaw_Angle;
    case imu_pitch:
        return Pitch_Angle;
    case imu_roll:
        return Roll_Angle;
    default:
        return 0;
    }
}
float witimu::WIT_IMU_Speed(IMU_data which)
{
    switch (which)
    {
    case imu_yaw:
        return Yaw_Speed;
    case imu_pitch:
        return Pitch_Speed;
    case imu_roll:
        return Roll_Speed;
    default:
        return 0;
    }
}
void witimu::IMU_Clean_Rotate()
{
    rotate_times=0;
}



