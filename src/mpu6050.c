/**
 * @file    mpu6050.c
 * @brief   Реализация драйвера MPU6050
 */

/* Includes ------------------------------------------------------------------*/
#include "mpu6050.h"

/* Defines ------------------------------------------------------------------*/

/** @brief Чувствительность гироскопа */
#define MPU6050_GYRO_SENS_250   ((float)131)
#define MPU6050_GYRO_SENS_500   ((float)65.5)
#define MPU6050_GYRO_SENS_1000  ((float)32.8)
#define MPU6050_GYRO_SENS_2000  ((float)16.4)

/** @brief Чувствительность акселерометра */
#define MPU6050_ACCE_SENS_2     ((float)16384)
#define MPU6050_ACCE_SENS_4     ((float)8192)
#define MPU6050_ACCE_SENS_8     ((float)4096)
#define MPU6050_ACCE_SENS_16    ((float)2048)

/* Private function prototypes -----------------------------------------------*/
static int32_t median3(int32_t a, int32_t b, int32_t c);

/* Private functions ---------------------------------------------------------*/
/**
 * @brief                             Инициализация MPU6050
 * @param  mpu6050                    Указатель на структуру данных MPU6050
 * @param  hi2c                       Обработчик I2C
 * @param  DeviceNumber               Номер устройства (AD0)
 * @param  AccelerometerSensitivity   Диапазон акселерометра
 * @param  GyroscopeSensitivity       Диапазон гироскопа
 * @return                            Результат операции 
 */
MPU6050_Result_t MPU6050_Init(MPU6050_t *mpu6050, I2C_HandleTypeDef *hi2c, 
                             MPU6050_Device_t DeviceNumber,
                             MPU6050_Accelerometer_t AccelerometerSensitivity,
                             MPU6050_Gyroscope_t GyroscopeSensitivity)
{
    uint8_t temp;
    HAL_StatusTypeDef status;
    uint8_t addr8;
    uint8_t addr7;
    mpu6050->hi2c = hi2c;

    addr8 = (uint8_t)(MPU6050_I2C_ADDR | ((uint8_t)DeviceNumber << 1));
    addr7 = (uint8_t)((MPU6050_I2C_ADDR >> 1) | (uint8_t)DeviceNumber);
    mpu6050->addr = addr8;
    
    /* Аппаратный сброс */
    status = I2C_write_byte(mpu6050->hi2c, mpu6050->addr, MPU6050_PWR_MGMT_1, 0x80);
    if (status != HAL_OK)
    {
        mpu6050->addr = addr7;
        status = I2C_write_byte(mpu6050->hi2c, mpu6050->addr, MPU6050_PWR_MGMT_1, 0x80);
        if (status != HAL_OK)
        {
            return MPU6050_Result_DeviceNotConnected;
        }
    }
    HAL_DelayMs(100);

    if (I2C_write_byte(mpu6050->hi2c, mpu6050->addr, MPU6050_PWR_MGMT_1, 0x00) != HAL_OK)
    {
        return MPU6050_Result_DeviceNotConnected;
    }
    HAL_DelayMs(100);
    
    /* Проверка WHO_AM_I */
    if (I2C_read_byte(mpu6050->hi2c, mpu6050->addr, MPU6050_WHO_AM_I, &temp) != HAL_OK)
    {
        return MPU6050_Result_DeviceNotConnected;
    }
    
    if (temp != MPU6050_I_AM && temp != MPU6050_WHO_AM_I_ALT)
    {
        return MPU6050_Result_DeviceInvalid;
    }
    
    I2C_write_byte(mpu6050->hi2c, mpu6050->addr, MPU6050_PWR_MGMT_1, 0x01);
    HAL_DelayMs(10);
    
    I2C_write_byte(mpu6050->hi2c, mpu6050->addr, MPU6050_CONFIG, 0x04);

    MPU6050_SetDataRate(mpu6050, 19);
    MPU6050_SetAccelerometer(mpu6050, AccelerometerSensitivity);
    MPU6050_SetGyroscope(mpu6050, GyroscopeSensitivity);
    
    mpu6050->Gx_Offset = 0;
    mpu6050->Gy_Offset = 0;
    mpu6050->Gz_Offset = 0;
    mpu6050->Ax_Offset = 0;
    mpu6050->Ay_Offset = 0;
    mpu6050->Az_Offset = 0;
    
    return MPU6050_Result_Ok;
}

/**
 * @brief                        Установка диапазона гироскопа
 * @param  mpu6050               Указатель на структуру данных MPU6050
 * @param  GyroscopeSensitivity  Новый диапазон
 * @return                       Результат операции 
 */
MPU6050_Result_t MPU6050_SetGyroscope(MPU6050_t* mpu6050,
                                     MPU6050_Gyroscope_t GyroscopeSensitivity)
{
    uint8_t temp;
    
    if (I2C_read_byte(mpu6050->hi2c, mpu6050->addr, MPU6050_GYRO_CONFIG, &temp) != HAL_OK)
    {
        return MPU6050_Result_Error;
    }
    
    temp = (temp & 0xE7) | ((uint8_t)GyroscopeSensitivity << 3);
    
    if (I2C_write_byte(mpu6050->hi2c, mpu6050->addr, MPU6050_GYRO_CONFIG, temp) != HAL_OK)
    {
        return MPU6050_Result_Error;
    }
    
    switch (GyroscopeSensitivity)
    {
        case MPU6050_Gyroscope_250s:
            mpu6050->Gyro_Mult = 1.0f / MPU6050_GYRO_SENS_250;
            break;
        case MPU6050_Gyroscope_500s:
            mpu6050->Gyro_Mult = 1.0f / MPU6050_GYRO_SENS_500;
            break;
        case MPU6050_Gyroscope_1000s:
            mpu6050->Gyro_Mult = 1.0f / MPU6050_GYRO_SENS_1000;
            break;
        case MPU6050_Gyroscope_2000s:
            mpu6050->Gyro_Mult = 1.0f / MPU6050_GYRO_SENS_2000;
            break;
        default:
            break;
    }
    
    return MPU6050_Result_Ok;
}

/**
 * @brief                            Установка диапазона акселерометра
 * @param  mpu6050                   Указатель на структуру данных MPU6050
 * @param  AccelerometerSensitivity  Новый диапазон
 * @return                           Результат операции 
 */
MPU6050_Result_t MPU6050_SetAccelerometer(MPU6050_t* mpu6050,
                                         MPU6050_Accelerometer_t AccelerometerSensitivity)
{
    uint8_t temp;
    
    if (I2C_read_byte(mpu6050->hi2c, mpu6050->addr, MPU6050_ACCEL_CONFIG, &temp) != HAL_OK)
    {
        return MPU6050_Result_Error;
    }
    
    temp = (temp & 0xE7) | ((uint8_t)AccelerometerSensitivity << 3);
    
    if (I2C_write_byte(mpu6050->hi2c, mpu6050->addr, MPU6050_ACCEL_CONFIG, temp) != HAL_OK)
    {
        return MPU6050_Result_Error;
    }
    
    switch (AccelerometerSensitivity)
    {
        case MPU6050_Accelerometer_2G:
            mpu6050->Acce_Mult = 1.0f / MPU6050_ACCE_SENS_2;
            break;
        case MPU6050_Accelerometer_4G:
            mpu6050->Acce_Mult = 1.0f / MPU6050_ACCE_SENS_4;
            break;
        case MPU6050_Accelerometer_8G:
            mpu6050->Acce_Mult = 1.0f / MPU6050_ACCE_SENS_8;
            break;
        case MPU6050_Accelerometer_16G:
            mpu6050->Acce_Mult = 1.0f / MPU6050_ACCE_SENS_16;
            break;
        default:
            break;
    }
    
    return MPU6050_Result_Ok;
}

/**
 * @brief           Установка частоты дискретизации 
 * @param  mpu6050  Указатель на структуру данных MPU6050
 * @param  rate     Значение делителя
 * @return          Результат операции 
 */
MPU6050_Result_t MPU6050_SetDataRate(MPU6050_t* mpu6050, uint8_t rate)
{
    if (I2C_write_byte(mpu6050->hi2c, mpu6050->addr, MPU6050_SMPLRT_DIV, rate) != HAL_OK)
    {
        return MPU6050_Result_Error;
    }
    
    return MPU6050_Result_Ok;
}

/**
 * @brief           Чтение данных акселерометра (6 байт)
 * @param  mpu6050  Указатель на структуру данных MPU6050
 * @return          Результат операции 
 */
MPU6050_Result_t MPU6050_ReadAccelerometer(MPU6050_t* mpu6050)
{
    uint8_t data[6];
    
    if (I2C_read_data(mpu6050->hi2c, mpu6050->addr, MPU6050_ACCEL_XOUT_H, data, 6) != HAL_OK)
    {
        return MPU6050_Result_Error;
    }
    
    mpu6050->Ax_Raw = (int16_t)((data[0] << 8) | data[1]);
    mpu6050->Ay_Raw = (int16_t)((data[2] << 8) | data[3]);
    mpu6050->Az_Raw = (int16_t)((data[4] << 8) | data[5]);
    
    mpu6050->Ax = (float)mpu6050->Ax_Raw * mpu6050->Acce_Mult;
    mpu6050->Ay = (float)mpu6050->Ay_Raw * mpu6050->Acce_Mult;
    mpu6050->Az = (float)mpu6050->Az_Raw * mpu6050->Acce_Mult;
    
    return MPU6050_Result_Ok;
}

/**
 * @brief           Чтение данных гироскопа (6 байт)
 * @param  mpu6050  Указатель на структуру данных MPU6050
 * @return          Результат операции 
 */
MPU6050_Result_t MPU6050_ReadGyroscope(MPU6050_t* mpu6050)
{
    uint8_t data[6];
    
    if (I2C_read_data(mpu6050->hi2c, mpu6050->addr, MPU6050_GYRO_XOUT_H, data, 6) != HAL_OK)
    {
        return MPU6050_Result_Error;
    }
    
    mpu6050->Gx_Raw = (int16_t)((data[0] << 8) | data[1]) - mpu6050->Gx_Offset;
    mpu6050->Gy_Raw = (int16_t)((data[2] << 8) | data[3]) - mpu6050->Gy_Offset;
    mpu6050->Gz_Raw = (int16_t)((data[4] << 8) | data[5]) - mpu6050->Gz_Offset;
    
    mpu6050->Gx = (float)mpu6050->Gx_Raw * mpu6050->Gyro_Mult;
    mpu6050->Gy = (float)mpu6050->Gy_Raw * mpu6050->Gyro_Mult;
    mpu6050->Gz = (float)mpu6050->Gz_Raw * mpu6050->Gyro_Mult;
    
    return MPU6050_Result_Ok;
}

/**
 * @brief           Чтение температуры датчика
 * @param  mpu6050  Указатель на структуру данных MPU6050
 * @return          Результат операции 
 */
MPU6050_Result_t MPU6050_ReadTemperature(MPU6050_t* mpu6050)
{
    uint8_t data[2];
    int16_t temp;
    
    if (I2C_read_data(mpu6050->hi2c, mpu6050->addr, MPU6050_TEMP_OUT_H, data, 2) != HAL_OK)
    {
        return MPU6050_Result_Error;
    }
    
    temp = (int16_t)((data[0] << 8) | data[1]);
    mpu6050->Temperature = (float)temp / 340.0f + 36.53f;
    
    return MPU6050_Result_Ok;
}

/**
 * @brief           Чтение всех данных 
 * @param  mpu6050  Указатель на структуру данных MPU6050
 * @return          Результат операции 
 */
MPU6050_Result_t MPU6050_ReadAll(MPU6050_t* mpu6050)
{
    uint8_t data[14];
    int16_t temp;
    
    if (I2C_read_data(mpu6050->hi2c, mpu6050->addr, MPU6050_ACCEL_XOUT_H, data, 14) != HAL_OK)
    {
        return MPU6050_Result_Error;
    }
    
    mpu6050->Ax_Raw = (int16_t)((data[0] << 8) | data[1]) - mpu6050->Ax_Offset;
    mpu6050->Ay_Raw = (int16_t)((data[2] << 8) | data[3]) - mpu6050->Ay_Offset;
    mpu6050->Az_Raw = (int16_t)((data[4] << 8) | data[5]) - mpu6050->Az_Offset;
    
    mpu6050->Ax = (float)mpu6050->Ax_Raw * mpu6050->Acce_Mult;
    mpu6050->Ay = (float)mpu6050->Ay_Raw * mpu6050->Acce_Mult;
    mpu6050->Az = (float)mpu6050->Az_Raw * mpu6050->Acce_Mult;
    
    temp = (int16_t)((data[6] << 8) | data[7]);
    mpu6050->Temperature = (float)temp / 340.0f + 36.53f;
    
    mpu6050->Gx_Raw = (int16_t)((data[8] << 8) | data[9]) - mpu6050->Gx_Offset;
    mpu6050->Gy_Raw = (int16_t)((data[10] << 8) | data[11]) - mpu6050->Gy_Offset;
    mpu6050->Gz_Raw = (int16_t)((data[12] << 8) | data[13]) - mpu6050->Gz_Offset;
    
    mpu6050->Gx = (float)mpu6050->Gx_Raw * mpu6050->Gyro_Mult;
    mpu6050->Gy = (float)mpu6050->Gy_Raw * mpu6050->Gyro_Mult;
    mpu6050->Gz = (float)mpu6050->Gz_Raw * mpu6050->Gyro_Mult;
    
    return MPU6050_Result_Ok;
}

/**
 * @brief     Медиана трёх значений 
 * @param  a  Первое значение
 * @param  b  Второе значение
 * @param  c  Третье значение
 * @return    Медианное значение
 */
static int32_t median3(int32_t a, int32_t b, int32_t c)
{
    if ((a <= b && b <= c) || (c <= b && b <= a))
        return b;
    else if ((b <= a && a <= c) || (c <= a && a <= b))
        return a;
    else
        return c;
}

/**
 * @brief           Полная калибровка MPU6050 
 * @param  mpu      Указатель на структуру данных MPU6050
 * @param  samples  Количество отсчётов для усреднения
 * @return          Результат операции 
 */
MPU6050_Result_t MPU6050_CalibrateSensor(MPU6050_t* mpu, uint16_t samples)
{       
    mpu->Ax_Offset = 0; mpu->Ay_Offset = 0; mpu->Az_Offset = 0;
    mpu->Gx_Offset = 0; mpu->Gy_Offset = 0; mpu->Gz_Offset = 0;
    
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    uint16_t i;
    
    /* Калибровка гироскопа и акселерометра с медианным фильтром */
    int32_t ax_buf[3] = {0}, ay_buf[3] = {0}, az_buf[3] = {0};
    int32_t gx_buf[3] = {0}, gy_buf[3] = {0}, gz_buf[3] = {0};
    uint8_t idx = 0;
    uint16_t valid_count = 0;
    
    for (i = 0; i < samples; i++)
    {
        if (MPU6050_ReadAll(mpu) != MPU6050_Result_Ok)
            continue;

        /* Заполняем буфер для медианного фильтра */
        ax_buf[idx] = mpu->Ax_Raw;
        ay_buf[idx] = mpu->Ay_Raw;
        az_buf[idx] = mpu->Az_Raw;
        gx_buf[idx] = mpu->Gx_Raw;
        gy_buf[idx] = mpu->Gy_Raw;
        gz_buf[idx] = mpu->Gz_Raw;
        idx = (idx + 1) % 3;
        
        /* После заполнения первых 3 значений используем медианный фильтр */
        if (i >= 2)
        {
            ax_sum += median3(ax_buf[0], ax_buf[1], ax_buf[2]);
            ay_sum += median3(ay_buf[0], ay_buf[1], ay_buf[2]);
            az_sum += median3(az_buf[0], az_buf[1], az_buf[2]);
            gx_sum += median3(gx_buf[0], gx_buf[1], gx_buf[2]);
            gy_sum += median3(gy_buf[0], gy_buf[1], gy_buf[2]);
            gz_sum += median3(gz_buf[0], gz_buf[1], gz_buf[2]);
        }
        else
        {
            /* Для первых двух измерений используем среднее */
            ax_sum += mpu->Ax_Raw;
            ay_sum += mpu->Ay_Raw;
            az_sum += mpu->Az_Raw;
            gx_sum += mpu->Gx_Raw;
            gy_sum += mpu->Gy_Raw;
            gz_sum += mpu->Gz_Raw;
        }
        
        valid_count++;
        HAL_DelayMs(5);
    }
    
    if (valid_count == 0)
        return MPU6050_Result_Error;
    
     /* Расчёт смещений акселерометра */
    mpu->Ax_Offset = ax_sum / valid_count;
    mpu->Ay_Offset = ay_sum / valid_count;
    mpu->Az_Offset = (az_sum / valid_count) - (int16_t)(1.0f / mpu->Acce_Mult);
    
     /* Расчёт смещений гироскопа */
    mpu->Gx_Offset = gx_sum / valid_count;
    mpu->Gy_Offset = gy_sum / valid_count;
    mpu->Gz_Offset = gz_sum / valid_count;
    
    mpu->Base_Temperature = mpu->Temperature;

    /* Температурная компенсация смещения по умолчанию выключена (коэффициенты = 0) */
    mpu->Gyro_TempCoeff[0] = 0.0f;
    mpu->Gyro_TempCoeff[1] = 0.0f;
    mpu->Gyro_TempCoeff[2] = 0.0f;

    /* Сохраняем базовую температуру для температурной компенсации */
    mpu->Base_Temperature = mpu->Temperature;
    
    return MPU6050_Result_Ok;
}

/**
 * @brief           Включение прерывания «данные готовы»
 * @param  mpu6050  Указатель на структуру данных MPU6050
 * @return          Результат операции 
 */
MPU6050_Result_t MPU6050_EnableInterrupts(MPU6050_t* mpu6050)
{
    uint8_t temp;
    
    if (I2C_write_byte(mpu6050->hi2c, mpu6050->addr, MPU6050_INT_ENABLE, 0x01) != HAL_OK)
    {
        return MPU6050_Result_Error;
    }
    
    if (I2C_read_byte(mpu6050->hi2c, mpu6050->addr, MPU6050_INT_PIN_CFG, &temp) != HAL_OK)
    {
        return MPU6050_Result_Error;
    }
    
    temp |= 0x10;
    
    if (I2C_write_byte(mpu6050->hi2c, mpu6050->addr, MPU6050_INT_PIN_CFG, temp) != HAL_OK)
    {
        return MPU6050_Result_Error;
    }
    
    return MPU6050_Result_Ok;
}

/**
 * @brief           Отключение прерываний MPU6050
 * @param  mpu6050  Указатель на структуру данных MPU6050
 * @return          Результат операции 
 */
MPU6050_Result_t MPU6050_DisableInterrupts(MPU6050_t* mpu6050)
{
    if (I2C_write_byte(mpu6050->hi2c, mpu6050->addr, MPU6050_INT_ENABLE, 0x00) != HAL_OK)
    {
        return MPU6050_Result_Error;
    }
    
    return MPU6050_Result_Ok;
}

/**
 * @brief                   Чтение и сброс флагов прерываний
 * @param  mpu6050          Указатель на структуру данных MPU6050
 * @param  InterruptsStruct Указатель на структуру для записи статуса
 * @return                  Результат операции 
 */
MPU6050_Result_t MPU6050_ReadInterrupts(MPU6050_t* mpu6050,
                                       MPU6050_Interrupt_t* InterruptsStruct)
{
    uint8_t read;
    
    InterruptsStruct->Status = 0;
    
    if (I2C_read_byte(mpu6050->hi2c, mpu6050->addr, MPU6050_INT_STATUS, &read) != HAL_OK)
    {
        return MPU6050_Result_Error;
    }
    
    InterruptsStruct->Status = read;
    
    return MPU6050_Result_Ok;
}
