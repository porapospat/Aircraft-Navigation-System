/**
 * @file    bus_i2c.h
 * @brief   Драйвер I2C  
 */
#ifndef __BUS_I2C_H__
#define __BUS_I2C_H__

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "mik32_hal_i2c.h"
#include "xprintf.h"

/* Exported functions ------------------------------------------------------- */

/**
 * @brief        Инициализация I2C0 в режиме ведущего
 * @param  I2Cx  Указатель на обработчик I2C
 */
void I2C0_Init(I2C_HandleTypeDef *I2Cx);

/**
 * @brief        Инициализация I2C1 в режиме ведущего
 * @param  I2Cx  Указатель на обработчик I2C
 */
void I2C1_Init(I2C_HandleTypeDef *I2Cx);

/**
 * @brief               Запись одного байта в регистр устройства
 * @param  hi2c         Указатель на обработчик I2C
 * @param  device_addr  I2C-адрес ведомого устройства
 * @param  reg_addr     Адрес регистра
 * @param  data         Байт для записи
 * @return              HAL_OK при успехе, иначе код ошибки
 */
HAL_StatusTypeDef I2C_write_byte(I2C_HandleTypeDef *hi2c, uint8_t device_addr,
                                 uint8_t reg_addr, uint8_t data);

/**
 * @brief               Чтение одного байта из регистра устройства
 * @param  hi2c         Указатель на обработчик I2C
 * @param  device_addr  I2C-адрес ведомого устройства
 * @param  reg_addr     Адрес регистра
 * @param  data         Указатель для записи прочитанного байта
 * @return              HAL_OK при успехе, иначе код ошибки
 */
HAL_StatusTypeDef I2C_read_byte(I2C_HandleTypeDef *hi2c, uint8_t device_addr,
                                uint8_t reg_addr, uint8_t *data);

/**
 * @brief               Чтение блока данных из последовательных регистров
 * @param  hi2c         Указатель на обработчик I2C
 * @param  device_addr  I2C-адрес ведомого устройства
 * @param  reg_addr     Адрес начального регистра
 * @param  data         Буфер для записи прочитанных данных
 * @param  len          Количество байт для чтения
 * @return              HAL_OK при успехе, иначе код ошибки
 */
HAL_StatusTypeDef I2C_read_data(I2C_HandleTypeDef *hi2c, uint8_t device_addr,
                                uint8_t reg_addr, uint8_t *data, uint16_t len);

/**
 * @brief               Запись блока данных в последовательные регистры
 * @param  hi2c         Указатель на обработчик I2C
 * @param  device_addr  I2C-адрес ведомого устройства
 * @param  reg_addr     Адрес начального регистра
 * @param  data         Буфер с данными для записи
 * @param  len          Количество байт для записи
 * @return              HAL_OK при успехе, иначе код ошибки
 */
HAL_StatusTypeDef I2C_write_data(I2C_HandleTypeDef *hi2c, uint8_t device_addr,
                                 uint8_t reg_addr, uint8_t *data, uint16_t len);

/**
 * @brief        Сканирование I2C-шины (адреса 0x08..0x77)
 * @param  hi2c  Указатель на обработчик I2C
 */
void I2C_Scan(I2C_HandleTypeDef *hi2c);

#endif /* __BUS_I2C_H__ */
