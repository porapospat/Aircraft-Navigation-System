/**
 * @file    ina219.c
 * @brief   Реализация драйвера INA219
 */

/* Includes ------------------------------------------------------------------*/
#include "ina219.h"

/* Private variables ---------------------------------------------------------*/
static uint16_t calibrationValue = 0;
static int16_t currentDivider_mA = 0;
static int16_t powerMultiplier_mW = 0;

/* Private functions ---------------------------------------------------------*/
static HAL_StatusTypeDef INA219_Write16(INA219_t *ina219, uint8_t reg, uint16_t value)
{
    uint8_t tx[2];

    if (ina219 == NULL || ina219->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    tx[0] = (uint8_t)(value >> 8);
    tx[1] = (uint8_t)(value & 0xFF);

    return I2C_write_data(ina219->hi2c, ina219->Address, reg, tx, 2);
}

/** @brief 16-битного значения из регистра INA219 */
static uint16_t INA219_Read16(INA219_t *ina219, uint8_t reg)
{
    uint8_t rx[2] = {0};

    if (ina219 == NULL || ina219->hi2c == NULL)
    {
        return 0;
    }

    if (I2C_read_data(ina219->hi2c, ina219->Address, reg, rx, 2) != HAL_OK)
    {
        return 0;
    }

    return (uint16_t)((((uint16_t)rx[0]) << 8) | rx[1]);
}

/** @brief Проверка отклика устройства по указанному I2C-адресу */
static uint8_t INA219_ProbeAddress(INA219_t *ina219, uint8_t addr)
{
    uint8_t rx[2] = {0};

    ina219->Address = addr;
    return (I2C_read_data(ina219->hi2c, ina219->Address, INA219_REG_CONFIG, rx, 2) == HAL_OK) ? 1U : 0U;
}

/** @brief Калибровка INA219 для диапазона 32 В / 2 А */
static uint8_t INA219_SetCalibration32V2A(INA219_t *ina219)
{
    uint16_t config;

    calibrationValue = 4096;
    currentDivider_mA = 10;
    powerMultiplier_mW = 2;

    if (INA219_Write16(ina219, INA219_REG_CALIBRATION, calibrationValue) != HAL_OK)
    {
        return 0U;
    }

    config = INA219_CONFIG_BVOLTAGERANGE_32V |
             INA219_CONFIG_GAIN_8_320MV |
             INA219_CONFIG_BADCRES_12BIT |
             INA219_CONFIG_SADCRES_12BIT_1S_532US |
             INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

    if (INA219_Write16(ina219, INA219_REG_CONFIG, config) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}

/* Exported functions --------------------------------------------------------*/
/**
 * @brief           Инициализация INA219
 * @param ina219    Указатель на обработчик INA219
 * @param i2c       Указатель на обработчик I2C 
 * @param Address   Адрес устройства (0x40)
 * @return 1 при успешной инициализации, 0 при ошибке
 */
uint8_t INA219_Init(INA219_t *ina219, I2C_HandleTypeDef *i2c, uint8_t Address)
{
    uint8_t addr8;
    uint8_t addr7;

    if (ina219 == NULL || i2c == NULL)
    {
        return 0;
    }

    ina219->hi2c = i2c;
    addr8 = Address;
    addr7 = (uint8_t)(Address >> 1);

    if (!INA219_ProbeAddress(ina219, addr8) && !INA219_ProbeAddress(ina219, addr7))
    {
        return 0;
    }

    if (!INA219_SetCalibration32V2A(ina219))
    {
        return 0;
    }

    return 1;
}

/**
 * @brief Получение результатов измерений напряжения
 * @param ina219 Указатель на обработчик INA219
 * @return Напряжение шины в милливольтах
 */
uint16_t INA219_ReadBusVoltage(INA219_t *ina219)
{
    uint16_t raw = INA219_Read16(ina219, INA219_REG_BUSVOLTAGE);

    raw >>= 3;
    raw *= 4U;

    return raw;
}

/**
 * @brief Получение результатов измерений тока
 * @param ina219 Указатель на обработчик INA219
 * @return Ток в миллиамперах
 */
int16_t INA219_ReadCurrent(INA219_t *ina219)
{
    int16_t raw;

    if (currentDivider_mA == 0)
    {
        return 0;
    }

    raw = (int16_t)INA219_Read16(ina219, INA219_REG_CURRENT);
    return (int16_t)(raw / currentDivider_mA);
}

/**
 * @brief Получение результатов измерений мощности
 * @param ina219 Указатель на обработчик INA219
 * @return Мощность в милливаттах
 */
uint16_t INA219_ReadPower(INA219_t *ina219)
{
    uint16_t raw = INA219_Read16(ina219, INA219_REG_POWER);
    return (uint16_t)(raw * (uint16_t)powerMultiplier_mW);
}
