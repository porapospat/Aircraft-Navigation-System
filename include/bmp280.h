/**
 * @file  bmp280.h
 * @brief Драйвер датчика BMP280
 */

#ifndef __BMP280_H__
#define __BMP280_H__

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "bus_i2c.h"

/* Exported define -----------------------------------------------------------*/
#define BMP280_I2C_ADDR                      (0x76U << 1)  /* адрес BMP280 */
#define BMP280_DEFAULT_CHIP_ID               (0x58)

#define BMP280_CHIP_ID_REG                   (0xD0)  /* Регистр идентификатора чипа */
#define BMP280_RESET_REG                     (0xE0)  /* Регистр программного сброса */
#define BMP280_STATUS_REG                    (0xF3)  /* Регистр статуса */
#define BMP280_CTRL_MEAS_REG                 (0xF4)  /* Регистр управления измерениями */
#define BMP280_CONFIG_REG                    (0xF5)  /* Регистр конфигурации */
#define BMP280_PRESSURE_MSB_REG              (0xF7)  /* Давление, старший байт */
#define BMP280_PRESSURE_LSB_REG              (0xF8)  /* Давление, младший байт */
#define BMP280_PRESSURE_XLSB_REG             (0xF9)  /* Давление, младшие 4 бита */
#define BMP280_TEMPERATURE_MSB_REG           (0xFA)  /* Температура, старший байт */
#define BMP280_TEMPERATURE_LSB_REG           (0xFB)  /* Температура, младший байт */
#define BMP280_TEMPERATURE_XLSB_REG          (0xFC)  /* Температура, младшие 4 бита */
/** Режимы питания (биты [1:0] регистра CTRL_MEAS): сон, одиночное, непрерывное */
#define BMP280_SLEEP_MODE                    (0x00)
#define BMP280_FORCED_MODE                   (0x01)
#define BMP280_NORMAL_MODE                   (0x03)

/** Бит 3 STATUS: идёт преобразование (данные ещё не обновлены) */
#define BMP280_STATUS_MEASURING              (0x08)

#define BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG             (0x88)
#define BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH       (24)
#define BMP280_DATA_SIZE                                    (6)

#define BMP280_OVERSAMP_SKIPPED          (0x00)
#define BMP280_OVERSAMP_1X               (0x01)
#define BMP280_OVERSAMP_2X               (0x02)
#define BMP280_OVERSAMP_4X               (0x03)
#define BMP280_OVERSAMP_8X               (0x04)
#define BMP280_OVERSAMP_16X              (0x05)

#define BMP280_PRESSURE_OSR              (BMP280_OVERSAMP_8X)
#define BMP280_TEMPERATURE_OSR           (BMP280_OVERSAMP_1X)
/* Непрерывный цикл измерений (иначе одиночный режим даёт одно измерение и «залипание») */
#define BMP280_MODE                      (BMP280_PRESSURE_OSR << 2 | BMP280_TEMPERATURE_OSR << 5 | BMP280_NORMAL_MODE)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Статус драйвера BMP280
 */
typedef enum
{
    BMP280_STATE_OK = 0,   /* Операция выполнена успешно */
    BMP280_STATE_ERROR     /* Ошибка I2C или некорректный идентификатор чипа */
} BMP280_StateTypeDef;

/**
 * @brief Калибровочные коэффициенты датчика
 */
typedef struct BMP280_Params{
    uint16_t dig_T1; /* Калибровочный коэффициент T1 */
    int16_t dig_T2;  /* Калибровочный коэффициент T2 */
    int16_t dig_T3;  /* Калибровочный коэффициент T3 */
    uint16_t dig_P1; /* Калибровочный коэффициент P1 */
    int16_t dig_P2;  /* Калибровочный коэффициент P2 */
    int16_t dig_P3;  /* Калибровочный коэффициент P3 */
    int16_t dig_P4;  /* Калибровочный коэффициент P4 */
    int16_t dig_P5;  /* Калибровочный коэффициент P5 */
    int16_t dig_P6;  /* Калибровочный коэффициент P6 */
    int16_t dig_P7;  /* Калибровочный коэффициент P7 */
    int16_t dig_P8;  /* Калибровочный коэффициент P8 */
    int16_t dig_P9;  /* Калибровочный коэффициент P9 */
    int32_t t_fine;  /* Промежуточное значение для расчёта температуры */
    uint32_t chip_id;  /* Идентификатор чипа */
} BMP280_Params;

/**
 * @brief Обработчик датчика BMP280
 */
typedef struct
{
    uint8_t addr;                 /* I2C-адрес устройства */
    BMP280_StateTypeDef state;    /* Текущий статус драйвера */
    I2C_HandleTypeDef *hi2c;      /* Обработчик I2C-интерфейса */
    BMP280_Params calib;          /* Калибровочные коэффициенты */
    int32_t raw_pressure;         /* Сырое значение давления */
    int32_t raw_temperature;      /* Сырое значение температуры */
} BMP280_HandleTypeDef;

/* Exported functions ------------------------------------------------------- */

/**
 * @brief       Инициализация датчика BMP280 и чтение калибровочных коэффициентов.
 * @param hbmp  Указатель на обработчик BMP280
 * @param hi2c  Указатель на обработчик I2C интерфейса
 * @param addr  I2C-адрес датчика
 * @return      BMP280_STATE_OK при успехе, иначе BMP280_STATE_ERROR
 */
BMP280_StateTypeDef BMP280_Init(BMP280_HandleTypeDef *hbmp,
                                I2C_HandleTypeDef *hi2c,
                                uint8_t addr);

/**
 * @brief       Запуск измерений и чтение сырых данных давления и температуры.
 * @param hbmp  Указатель на обработчик BMP280
 * @return      BMP280_STATE_OK при успехе, иначе BMP280_STATE_ERROR
 */
BMP280_StateTypeDef BMP280_ReadAll(BMP280_HandleTypeDef *hbmp);

/**
 * @brief  Преобразование сырых значений температуры с учётом калибровочных коэффициентов
 * @param  adc_T Сырое значение АЦП температуры
 * @return       Температура
 */
int32_t BMP280_Compensate_T(BMP280_HandleTypeDef *hbmp, int32_t adc_T);

/**
 * @brief Преобразование сырых значений давления с учётом калибровочных коэффициентов
 * @param adc_P Сырое значение АЦП давления
 */
uint32_t BMP280_Compensate_P(BMP280_HandleTypeDef *hbmp, int32_t adc_P);

/**
 * @brief             Получение компенсированных значений давления и температуры.
 * @param hbmp        Указатель на обработчик BMP280
 * @param pressure    Давление, Па
 * @param temperature Температура, °C
 */
void BMP280_Calculate(BMP280_HandleTypeDef *hbmp,
                      int32_t *pressure,
                      int32_t *temperature);

/**
 * @brief             Вывод давления и температуры в формате целая/дробная часть
 * @param pressure    Давление, Па
 * @param temperature Температура, °C
 */
void BMP280_Print(int32_t pressure, int32_t temperature);

#endif /* __BMP280_H__ */
