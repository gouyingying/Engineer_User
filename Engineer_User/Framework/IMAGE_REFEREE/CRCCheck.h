//
// Created by Barbatos on 2023/1/16.
//

#ifndef CRCCHECK_H
#define CRCCHECK_H

#include "stm32f4xx.h"
#include "stdint.h"
#include "stdlib.h"

#define CRC8_INIT_NUM    0xff
#define CRC16_INIT_NUM  0xffff


uint8_t GetCRC8CheckSum(uint8_t *pch_message, uint16_t dw_length, uint8_t uc_crc8);

uint8_t VerifyCRC8CheckSum(uint8_t *pch_message, uint16_t dw_length);

void AppendCRC8CheckSum(uint8_t *pch_message, uint16_t dw_length);

uint16_t GetCRC16CheckSum(uint8_t *pch_message, uint32_t dw_length, uint16_t w_crc);

uint8_t VerifyCRC16CheckSum(uint8_t *pch_message, uint32_t dw_length);

void AppendCRC16CheckSum(uint8_t *pch_message, uint32_t dw_length);

#endif //CRCCHECK_H
