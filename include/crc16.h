/**
 * @file    crc16.h
 * @brief   Вычисление контрольной суммы CRC16 (полином 0xA001)
 */
#ifndef __CRC16_H__
#define __CRC16_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported functions ------------------------------------------------------- */

/**
 * @brief         Вычисление CRC16 
 * @param  frame  Указатель на буфер с данными
 * @param  len    Длина данных в байтах
 * @return        Вычисленное значение CRC16
 */
uint16_t CRC16_Calc(uint8_t *frame, uint16_t len);

#endif /* __CRC16_H__ */
