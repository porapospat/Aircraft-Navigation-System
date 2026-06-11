/**
 * @file    gps.c
 * @brief   Драйвер GPS-модуля NEO-M8N (приём и разбор NMEA-предложений)
 */

/* Includes ------------------------------------------------------------------*/
#include "gps.h"
#include <string.h>

/* Private functions ---------------------------------------------------------*/

/**
 * @brief          Инициализация GPS-модуля (настройка USART1)
 * @param  husart  Указатель на дескриптор USART
 */
void GPS_Init(USART_HandleTypeDef *husart)
{
    USART0_Init(husart);
}

/**
 * @brief            Извлечение очередного NMEA-предложения из приёмного буфера
 * @param  sentence  Буфер для записи предложения (с завершающим нулём)
 * @param  max_len   Максимальная длина буфера
 * @return           Полное предложение извлечено, предложение не получено
 */
uint8_t GPS_GetSentence(char *sentence, uint32_t max_len)
{
    static uint8_t in_sentence = 0;
    static uint32_t index = 0;
    static char temp_buffer[MAX_SENTENCE_LENGTH];
    uint8_t ch;
    
    while(UART_ReadChar(&ch))
    {
        if(ch == '$' && !in_sentence)
        {
            in_sentence = 1;
            index = 0;
            temp_buffer[index++] = ch;
        }
        else if(in_sentence)
        {
            if(ch == '\n')
            {
                temp_buffer[index] = '\0';
                in_sentence = 0;
                
                if(index > 6)
                {
                    strncpy(sentence, temp_buffer, max_len);
                    sentence[max_len-1] = '\0';
                    return true;
                }
                index = 0;
            }
            else if(index < (MAX_SENTENCE_LENGTH - 1))
            {
                temp_buffer[index++] = ch;
            }
            else
            {
                in_sentence = 0;
                index = 0;
            }
        }
    }
    
    return false;
}
