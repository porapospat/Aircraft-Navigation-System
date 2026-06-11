/**
 * @file    ina219.h
 * @brief   Драйвер датчика тока и напряжения INA219
 */
#ifndef __INA219_H__
#define __INA219_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "bus_i2c.h"

/* Exported define -----------------------------------------------------------*/
#define INA219_ADDRESS                          (0x40)

#define INA219_REG_CONFIG                       (0x00)
#define INA219_REG_BUSVOLTAGE                   (0x02)
#define INA219_REG_POWER                        (0x03)
#define INA219_REG_CURRENT                      (0x04)
#define INA219_REG_CALIBRATION                  (0x05)

#define INA219_CONFIG_BVOLTAGERANGE_32V         (0x2000)
#define INA219_CONFIG_GAIN_8_320MV              (0x1800)
#define INA219_CONFIG_BADCRES_12BIT             (0x0180)
#define INA219_CONFIG_SADCRES_12BIT_1S_532US    (0x0018)
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS (0x0007)

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    I2C_HandleTypeDef *hi2c;  /* Обработчик I2C-интерфейса */
    uint8_t Address;          /* I2C-адрес устройства */
} INA219_t;

/* Exported functions --------------------------------------------------------*/
/**
 * @brief           Инициализация INA219
 * @param ina219    Указатель на обработчик INA219
 * @param i2c       Указатель на обработчик I2C 
 * @param Address   Адрес устройства (0x40)
 * @return          1 при успешной инициализации, 0 при ошибке
 */ 
uint8_t INA219_Init(INA219_t *ina219, I2C_HandleTypeDef *i2c, uint8_t Address);

/**
 * @brief        Получение напряжения шины
 * @param ina219 Указатель на обработчик INA219
 * @return       Напряжение в милливольтах
 */
uint16_t INA219_ReadBusVoltage(INA219_t *ina219);

/**
 * @brief        Получение тока
 * @param ina219 Указатель на обработчик INA219
 * @return       Ток в миллиамперах
 */
int16_t INA219_ReadCurrent(INA219_t *ina219);

/**
 * @brief        Получение мощности
 * @param ina219 Указатель на обработчик INA219
 * @return       Мощность в милливаттах
 */
uint16_t INA219_ReadPower(INA219_t *ina219);

#endif /* __INA219_H__ */
