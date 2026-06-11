/**
 * @file    ui.h
 * @brief   Формирование и отправка пакетов данных MPU6050, GPS, BMP, INA219
 */
#ifndef UI_H
#define UI_H

/* Includes ------------------------------------------------------------------*/
#include "crc16.h"
#include "bus_usart.h"

/* Exported define -----------------------------------------------------------*/
#define MSG_HEADER          0x7B7E /* Заголовок пакета */

/** 
 * @brief Параметры пакета MPU6050
 */
#define MSG_MPU_ID          0x01   /* Индекс пакета MPU6050 */
#define MSG_DATA_SIZE       36     /* Размер поля данных: 9 x 4 */

/**
 * @brief Параметры пакета GPS
 */
#define GPS_MSG_ID          0x02   /* Индекс пакета GPS */
#define GPS_MSG_DATA_SIZE   12     /* Размер поля данных: 3 x 4 (широта, долгота) */
#define GPS_STATUS_VALID    0x01   /* GPS-данные валидны */
#define GPS_STATUS_INVALID  0x00   /* GPS-данные невалидны */

/**
 * @brief Параметры пакета BMP
 */
#define BMP_MSG_ID          0x03   /* Индекс пакета BMP */
#define BMP_MSG_DATA_SIZE   8      /* Размер поля данных: 2 x 4 (давление, температура) */

/**
 * @brief Параметры пакета INA219
 */
#define INA219_MSG_ID        0x04  /* Идентификатор пакета INA219 */
#define INA219_MSG_DATA_SIZE 12    /* Размер поля данных: 3 x 4 (U, I, P) */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Структура бинарного пакета данных MPU6050
 */
typedef struct
{
    uint16_t HDR;      /* Заголовок пакета MSG_HEADER */
    uint8_t  ID;       /* Идентификатор MSG_MPU_ID */
    uint8_t  LEN;      /* Длина поля данных в байтах */
    float    DATA[9];  /* Данные: [крен, тангаж, рыскание, Ax, Ay, Az, Gx, Gy, Gz] */
    uint16_t CRC16;    /* Контрольная сумма CRC16 */
} sMPU_MSG;

/** 
 * @brief Структура бинарного пакета данных GPS 
 */
typedef struct
{
    uint16_t HDR;      /* Заголовок пакета MSG_HEADER */
    uint8_t  ID;       /* Идентификатор GPS_MSG_ID */
    uint8_t  LEN;      /* Длина поля данных в байтах */
    float    DATA[3];  /* Данные: [широта, долгота, высота] */
    uint16_t CRC16;    /* Контрольная сумма CRC16 */
} sGPS_MSG;

/** 
 * @brief Структура бинарного пакета данных BMP 
 */
typedef struct
{
    uint16_t HDR;      /* Заголовок пакета MSG_HEADER */
    uint8_t  ID;       /* Идентификатор BMP_MSG_ID */
    uint8_t  LEN;      /* Длина поля данных в байтах */
    float    DATA[2];  /* Данные: [давление, температура] */
    uint16_t CRC16;    /* Контрольная сумма CRC16 */
}sBMP_MSG;

/**
 * @brief Структура бинарного пакета данных INA219
 */
typedef struct
{
    uint16_t HDR;      /* Заголовок пакета MSG_HEADER */
    uint8_t  ID;       /* Идентификатор INA219_MSG_ID */
    uint8_t  LEN;      /* Длина поля данных в байтах */
    float    DATA[3];  /* Данные: [напряжение, ток, мощность] */
    uint16_t CRC16;    /* Контрольная сумма CRC16 */
} sINA219_MSG;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief          Формирование и отправка пакета данных MPU6050
 * @param  husart  Указатель на дескриптор USART
 * @param  roll    Крен 
 * @param  pitch   Тангаж 
 * @param  yaw     Рыскание 
 * @param  Ax      Ускорение по оси X 
 * @param  Ay      Ускорение по оси Y 
 * @param  Az      Ускорение по оси Z 
 * @param  Gx      Угловая скорость X 
 * @param  Gy      Угловая скорость Y 
 * @param  Gz      Угловая скорость Z 
 */
uint8_t UI_SendMPUMSG(USART_HandleTypeDef *husart,
                      float roll, float pitch, float yaw,
                      float Ax, float Ay, float Az,
                      float Gx, float Gy, float Gz);

/**
 * @brief             Формирование и отправка пакета данных GPS
 * @param  husart     Указатель на дескриптор USART
 * @param  latitude   Широта 
 * @param  longitude  Долгота 
 * @param  gps_valid  Флаг валидности
 */
uint8_t UI_SendGPSMSG(USART_HandleTypeDef *husart,
                      double latitude, double longitude, uint8_t gps_valid, float height);
                   
/**
 * @brief               Формирование и отправка пакета данных BMP
 * @param  husart       Указатель на дескриптор USART
 * @param  pressure     Давление
 * @param  temperature  Температура
 */
uint8_t UI_SendBMPMSG(USART_HandleTypeDef *husart, float pressure, float temperature);

/**
 * @brief               Формирование и отправка пакета данных INA219
 * @param  husart       Указатель на дескриптор USART
 * @param  bus_voltage  Напряжение, В
 * @param  current      Ток, А
 * @param  power        Мощность, Вт
 */
uint8_t UI_SendINA219MSG(USART_HandleTypeDef *husart, float bus_voltage, float current, float power);

#endif /* __UI_H__ */
