/**
 * @file    bus_i2c.c
 * @brief   Реализация драйвера I2C для МК MIK32
 */

/* Includes ------------------------------------------------------------------*/
#include "bus_i2c.h"

/* Private functions ---------------------------------------------------------*/

/**
 * @brief        Инициализация I2C0 в режиме ведущего
 * @param  I2Cx  Указатель на обработчик I2C
 */
void I2C0_Init(I2C_HandleTypeDef *I2Cx)
{
    I2Cx->Instance = I2C_0;

    I2Cx->Init.Mode = HAL_I2C_MODE_MASTER;

    I2Cx->Init.DigitalFilter = I2C_DIGITALFILTER_OFF;
    I2Cx->Init.AnalogFilter = I2C_ANALOGFILTER_DISABLE;
    I2Cx->Init.AutoEnd = I2C_AUTOEND_ENABLE;

    I2Cx->Clock.PRESC = 1;
    I2Cx->Clock.SCLDEL = 15;
    I2Cx->Clock.SDADEL = 15;
    I2Cx->Clock.SCLH = 75;
    I2Cx->Clock.SCLL = 75;

    if(HAL_I2C_Init(I2Cx)!=HAL_OK)
    {
        xprintf("I2C0_Init error\n");
    }
}

/**
 * @brief        Инициализация I2C1 в режиме ведущего
 * @param  I2Cx  Указатель на обработчик I2C
 */
void I2C1_Init(I2C_HandleTypeDef *I2Cx)
{
    I2Cx->Instance = I2C_1;

    I2Cx->Init.Mode = HAL_I2C_MODE_MASTER;

    I2Cx->Init.DigitalFilter = I2C_DIGITALFILTER_OFF;
    I2Cx->Init.AnalogFilter = I2C_ANALOGFILTER_DISABLE;
    I2Cx->Init.AutoEnd = I2C_AUTOEND_ENABLE;

    I2Cx->Clock.PRESC = 1;
    I2Cx->Clock.SCLDEL = 15;
    I2Cx->Clock.SDADEL = 15;
    I2Cx->Clock.SCLH = 75;
    I2Cx->Clock.SCLL = 75;

    if(HAL_I2C_Init(I2Cx)!=HAL_OK)
    {
        xprintf("I2C1_Init error\n");
    }
}

/**
 * @brief               Запись одного байта в регистр устройства
 * @param  hi2c         Указатель на обработчик I2C
 * @param  device_addr  I2C-адрес ведомого устройства
 * @param  reg_addr     Адрес регистра
 * @param  data         Байт для записи
 * @return              HAL_OK при успехе, иначе код ошибки
 */
HAL_StatusTypeDef I2C_write_byte(I2C_HandleTypeDef *hi2c, uint8_t device_addr, uint8_t reg_addr, uint8_t data)
{
    HAL_StatusTypeDef status;
    status = I2C_write_data(hi2c, device_addr, reg_addr, &data, 1);

    return status;
}

/**
 * @brief               Чтение одного байта из регистра устройства
 * @param  hi2c         Указатель на обработчик I2C
 * @param  device_addr  I2C-адрес ведомого устройства
 * @param  reg_addr     Адрес регистра
 * @param  data         Указатель для записи прочитанного байта
 * @return              HAL_OK при успехе, иначе код ошибки
 */
HAL_StatusTypeDef I2C_read_byte(I2C_HandleTypeDef *hi2c, uint8_t device_addr,
                               uint8_t reg_addr, uint8_t *data)
{
    HAL_StatusTypeDef status;
    status = I2C_read_data(hi2c, device_addr, reg_addr, data, 1);

    return status;
}

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
                               uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Transmit(hi2c, device_addr, &reg_addr, 1, I2C_TIMEOUT_DEFAULT);
    if (status != HAL_OK)
    {
        HAL_I2C_Reset(hi2c);
        return status;
    }

    status = HAL_I2C_Master_Receive(hi2c, device_addr, data, len, I2C_TIMEOUT_DEFAULT);
    if (status != HAL_OK)
    {
       HAL_I2C_Reset(hi2c);
    }
    return status;
}

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
                                uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;

    uint8_t buffer[len + 1];
    buffer[0] = reg_addr;
    memcpy(&buffer[1], data, len);

    status = HAL_I2C_Master_Transmit(hi2c, device_addr, buffer, len + 1, I2C_TIMEOUT_DEFAULT);
    if (status != HAL_OK)
    {
       HAL_I2C_Reset(hi2c);
    }
    return status;
}

void I2C_Scan(I2C_HandleTypeDef *hi2c)
{
    uint8_t dummy = 0;
    xprintf("I2C scan (0x08..0x77):\n");
    for (uint8_t addr = 0x08; addr <= 0x77; addr++)
    {
        HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(hi2c, addr, &dummy, 1, I2C_TIMEOUT_DEFAULT);
        if (st == HAL_OK)
            xprintf("  found device at 0x%02X\n", addr);
        else
            HAL_I2C_Reset(hi2c);
    }
    xprintf("I2C scan done\n");
}
