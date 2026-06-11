/**
 * @file    bmp280.c
 * @brief   Реализация драйвера BMP280
 */

/* Includes ------------------------------------------------------------------*/
#include "bmp280.h"

/* Private variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief           Проверка идентификатора чипа и чтение калибровочных коэффициентов
 * @param  hbmp     Указатель на обработчик BMP280
 * @return          true, если датчик найден и калибровка прочитана успешно
 */
static bool BMP280_Detect(BMP280_HandleTypeDef *hbmp)
{
    uint8_t chip_id = 0;
    HAL_StatusTypeDef status;

    if (hbmp == NULL || hbmp->hi2c == NULL)
        return false;

    status = I2C_read_byte(hbmp->hi2c, hbmp->addr, BMP280_CHIP_ID_REG, &chip_id);
    
    /* Ошибка чтения идентификатора чипа */
    if (status != HAL_OK)
        return false;

    hbmp->calib.chip_id = chip_id;

    /* BMP280 не найден */
    if (chip_id != BMP280_DEFAULT_CHIP_ID)
        return false;

    /* Только калибровочные регистры */
    status = I2C_read_data(hbmp->hi2c,
                           hbmp->addr,
                           BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG,
                           (uint8_t *)&hbmp->calib,
                           BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH);

    return (status == HAL_OK);
}

/**
 * @brief        Инициализация BMP280 и запуск измерений
 * @param  hbmp  Указатель на обработчик BMP280
 * @param  hi2c  Указатель на обработчик I2C
 * @param  addr  I2C адрес датчика
 * @return       BMP280_STATE_OK при успешной инициализации, иначе BMP280_STATE_ERROR.
 */
BMP280_StateTypeDef BMP280_Init(BMP280_HandleTypeDef *hbmp,
                                I2C_HandleTypeDef *hi2c,
                                uint8_t addr)
{
    uint8_t addr8;
    uint8_t addr7;

    if (hbmp == NULL || hi2c == NULL)
        return BMP280_STATE_ERROR;

    hbmp->hi2c  = hi2c;
    
    addr8 = addr;
    addr7 = (uint8_t)(addr >> 1);
   
    hbmp->addr  = addr8;
    hbmp->state = BMP280_STATE_ERROR;
    hbmp->raw_pressure = 0;
    hbmp->raw_temperature = 0;

    if (!BMP280_Detect(hbmp))
    {
        hbmp->addr = addr7;
        if (!BMP280_Detect(hbmp))
            return BMP280_STATE_ERROR;
    }
    
    if (I2C_write_byte(hbmp->hi2c, hbmp->addr, BMP280_CTRL_MEAS_REG, BMP280_MODE) != HAL_OK)
        return BMP280_STATE_ERROR;

    hbmp->state = BMP280_STATE_OK;
    return BMP280_STATE_OK;
}

/**
 * @brief       Чтение сырых данных с датчика
 * @param hbmp  Указатель на обработчик BMP280
 * @return      BMP280_STATE_OK при успехе, иначе BMP280_STATE_ERROR
*/
BMP280_StateTypeDef BMP280_ReadAll(BMP280_HandleTypeDef *hbmp)
{
    uint8_t data[BMP280_DATA_SIZE];

    if (hbmp == NULL || hbmp->hi2c == NULL)
        return BMP280_STATE_ERROR;

    if (I2C_read_data(hbmp->hi2c, hbmp->addr, BMP280_PRESSURE_MSB_REG, data, BMP280_DATA_SIZE) != HAL_OK)
        return BMP280_STATE_ERROR;
    
    hbmp->raw_pressure = (int32_t)((((uint32_t)(data[0])) << 12) | (((uint32_t)(data[1])) << 4) | ((uint32_t)data[2] >> 4));
    hbmp->raw_temperature = (int32_t)((((uint32_t)(data[3])) << 12) | (((uint32_t)(data[4])) << 4) | ((uint32_t)data[5] >> 4));

    return BMP280_STATE_OK;
}    

/** 
* @brief  Преобразование сырых значений температуры с учётом калибровочных коэффициентов
* @param  adc_T
*/
int32_t BMP280_Compensate_T(BMP280_HandleTypeDef *hbmp, int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)hbmp->calib.dig_T1 << 1))) *
            ((int32_t)hbmp->calib.dig_T2)) >> 11;

    var2 = (((((adc_T >> 4) - ((int32_t)hbmp->calib.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)hbmp->calib.dig_T1))) >> 12) *
            ((int32_t)hbmp->calib.dig_T3)) >> 14;

    hbmp->calib.t_fine = var1 + var2;
    T = (hbmp->calib.t_fine * 5 + 128) >> 8;

    return T;
}

/**
* @brief Преобразование сырых значений давления с учётом калибровочных коэффициентов
* @param adc_P
*/
uint32_t BMP280_Compensate_P(BMP280_HandleTypeDef *hbmp, int32_t adc_P)
{
    int64_t  var1, var2, P;

    var1 = ((int64_t)hbmp->calib.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)hbmp->calib.dig_P6;
    var2 = var2 + ((var1*(int64_t)hbmp->calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)hbmp->calib.dig_P4) << 35);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)hbmp->calib.dig_P1) >> 33;
    if (var1 == 0)
        return 0;
    P = 1048576 - adc_P;
    P = (((P << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)hbmp->calib.dig_P9) * (P >> 13) * (P >> 13)) >> 25;
    var2 = (((int64_t)hbmp->calib.dig_P8) * P) >> 19;
    P = ((P + var1 + var2) >> 8) + (((int64_t)hbmp->calib.dig_P7) << 4);
    
    return P;
}

/**
* @brief             Получение результатов измерений
* @param hbmp        Указатель на обработчик BMP280
* @param pressure    Давление, Па 
* @param temperature Температура, °C
*/
void BMP280_Calculate(BMP280_HandleTypeDef *hbmp, int32_t *pressure, int32_t *temperature)
{
    int32_t t;
    uint32_t p;

    t = BMP280_Compensate_T(hbmp, hbmp->raw_temperature);
    p = BMP280_Compensate_P(hbmp, hbmp->raw_pressure);

    if (temperature)
        *temperature = t;

    if (pressure)
        *pressure = (int32_t)(p / 256);
}

/**
 * @brief             Вывод давления и температуры в формате целая/дробная часть
 * @param pressure    Давление, Па 
 * @param temperature Температура, °C
 */
void BMP280_Print(int32_t pressure, int32_t temperature)
{
    int temp_int = (int)(temperature / 100);
    int temp_frac = (int)(temperature % 100);

    if (temp_frac < 0)
        temp_frac = -temp_frac;

    xprintf("BMP data:\n");
    xprintf("Pressure: %d  ", (int)pressure);
    xprintf("Temperature: %d.%d\n\n", temp_int, temp_frac);
}