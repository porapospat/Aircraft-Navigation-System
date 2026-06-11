/**
 * @file    bus_usart.h
 * @brief   Драйвер USART 
 */
#ifndef __USART_H__
#define __USART_H__

/* Includes ------------------------------------------------------------------*/
#include "mik32_hal_usart.h"
#include "mik32_hal_irq.h"

/* Exported define -----------------------------------------------------------*/
#define BAUDRATE                  9600  /* Скорость обмена UART */
#define UART_TX_BYTE_TIMEOUT_MS   5     /* Таймаут передачи одного байта */
#define UART_TX_RECOVERY_RETRIES  1     /* Кол-во попыток recovery при сбое TX */

/* Exported functions ------------------------------------------------------- */

/**
 * @brief          Инициализация USART0 (передача данных MPU6050/GPS)
 * @param  husart  Указатель на дескриптор USART
 */
void USART0_Init(USART_HandleTypeDef *husart);

/**
 * @brief          Инициализация USART1 (приём данных от GPS-модуля)
 * @param  husart  Указатель на дескриптор USART
 */
void USART1_Init(USART_HandleTypeDef *husart);

/**
 * @brief          Обработчик прерываний UART0 
 * @param  husart  Указатель на дескриптор USART
 */
void UART0_IRQHandler(USART_HandleTypeDef *husart);

/**
 * @brief          Обработчик прерываний UART1 
 * @param  husart  Указатель на дескриптор USART
 */
void USART1_IRQHandler(USART_HandleTypeDef *husart);

/**
 * @brief          Отправка одного байта через UART 
 * @param  husart  Указатель на дескриптор USART
 * @param  Data    Байт для отправки
 */
void UART_SendByte(USART_HandleTypeDef *husart, uint8_t Data);

/**
 * @brief       Чтение одного байта из приёмного буфера
 * @param  ch   Указатель для записи прочитанного байта
 * @return      Статус передачи 
 */
int8_t UART_ReadChar(uint8_t *ch);
/**
 * @brief          Отправка массива данных через UART 
 * @param  husart  Указатель на дескриптор USART
 * @param  str     Указатель на буфер с данными
 * @param  len     Длина данных в байтах
 * @return         Статус передачи
 */
uint8_t UART_SendFrame(USART_HandleTypeDef *husart, uint8_t *str, uint32_t len);

#endif /* __USART_H__ */

