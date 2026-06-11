/**
 * @file    gps.h
 * @brief   Драйвер GPS-модуля NEO-M8N (приём и разбор NMEA-предложений)
 */
#ifndef __GPS_H__
#define __GPS_H__

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "bus_usart.h"

/* Exported define -----------------------------------------------------------*/
#define GPS_RxBufferSize      256  /* Размер приёмного буфера GPS */

#ifndef MAX_SENTENCE_LENGTH
#define MAX_SENTENCE_LENGTH   82   /* Максимальная длина NMEA-предложения */
#endif

/* Exported functions ------------------------------------------------------- */

/**
 * @brief          Инициализация GPS-модуля 
 * @param  husart  Указатель на дескриптор USART
 */
void GPS_Init(USART_HandleTypeDef *husart);

/**
 * @brief            Извлечение очередного NMEA-предложения из приёмного буфера
 * @param  sentence  Буфер для записи предложения 
 * @param  max_len   Максимальная длина буфера
 * @return           Статус чтения предложения NMEA
 */
uint8_t GPS_GetSentence(char *sentence, uint32_t max_len);

#endif /* __GPS_H__ */
