/**
 * @file    mpu6050.h
 * @brief   Драйвер датчика MPU6050 
 */

#ifndef __MPU6050_H__
#define __MPU6050_H__

/* Includes ------------------------------------------------------------------*/
#include "bus_i2c.h"
#include "xprintf.h"

/* Exported define -----------------------------------------------------------*/

/** 
 * @brief MPU6050_DataRate Предустановки частоты дискретизации (SMPLRT_DIV)
 * @details Частота дискретизации = частота выхода гироскопа / (1 + SMPLRT_DIV)
 */
#define MPU6050_DataRate_8KHz   0   /* 8 кГц */
#define MPU6050_DataRate_4KHz   1   /* 4 кГц */
#define MPU6050_DataRate_2KHz   3   /* 2 кГц */
#define MPU6050_DataRate_1KHz   7   /* 1 кГц */
#define MPU6050_DataRate_500Hz  15  /* 500 Гц */
#define MPU6050_DataRate_250Hz  31  /* 250 Гц */
#define MPU6050_DataRate_125Hz  63  /* 125 Гц */
#define MPU6050_DataRate_100Hz  79  /* 100 Гц */

/** 
 * @brief MPU6050_Address I2C-адрес и идентификаторы WHO_AM_I
 */
#define MPU6050_I2C_ADDR        0xD0  /* I2C-адрес по умолчанию (7-битный: 0x68 << 1) */
#define MPU6050_I_AM            0x68  /* WHO_AM_I MPU6050 */
#define MPU6050_WHO_AM_I_ALT    0x70  /* WHO_AM_I для MPU9250 */

/** 
 * @brief MPU6050_Registers Адреса регистров MPU6050
 */
#define MPU6050_AUX_VDDIO       0x01  /* Конфигурация VDDIO */
#define MPU6050_SMPLRT_DIV      0x19  /* Делитель частоты дискретизации */
#define MPU6050_CONFIG          0x1A  /* Конфигурация (DLPF, FSYNC) */
#define MPU6050_GYRO_CONFIG     0x1B  /* Конфигурация гироскопа */
#define MPU6050_ACCEL_CONFIG    0x1C  /* Конфигурация акселерометра */
#define MPU6050_MOTION_THRESH   0x1F  /* Порог детектирования движения */
#define MPU6050_INT_PIN_CFG     0x37  /* Конфигурация пина прерывания */
#define MPU6050_INT_ENABLE      0x38  /* Включение прерываний */
#define MPU6050_INT_STATUS      0x3A  /* Статус прерываний */
#define MPU6050_ACCEL_XOUT_H    0x3B  /* Акселерометр X, старший байт */
#define MPU6050_ACCEL_XOUT_L    0x3C  /* Акселерометр X, младший байт */
#define MPU6050_ACCEL_YOUT_H    0x3D  /* Акселерометр Y, старший байт */
#define MPU6050_ACCEL_YOUT_L    0x3E  /* Акселерометр Y, младший байт */
#define MPU6050_ACCEL_ZOUT_H    0x3F  /* Акселерометр Z, старший байт */
#define MPU6050_ACCEL_ZOUT_L    0x40  /* Акселерометр Z, младший байт */
#define MPU6050_TEMP_OUT_H      0x41  /* Температура, старший байт */
#define MPU6050_TEMP_OUT_L      0x42  /* Температура, младший байт */
#define MPU6050_GYRO_XOUT_H     0x43  /* Гироскоп X, старший байт */
#define MPU6050_GYRO_XOUT_L     0x44  /* Гироскоп X, младший байт */
#define MPU6050_GYRO_YOUT_H     0x45  /* Гироскоп Y, старший байт */
#define MPU6050_GYRO_YOUT_L     0x46  /* Гироскоп Y, младший байт */
#define MPU6050_GYRO_ZOUT_H     0x47  /* Гироскоп Z, старший байт */
#define MPU6050_GYRO_ZOUT_L     0x48  /* Гироскоп Z, младший байт */
#define MPU6050_MOT_DETECT_STATUS 0x61 /* Статус детектирования движения */
#define MPU6050_SIGNAL_PATH_RESET 0x68 /* Сброс сигнального пути */
#define MPU6050_MOT_DETECT_CTRL 0x69  /* Управление детектированием движения */
#define MPU6050_USER_CTRL       0x6A  /* Пользовательское управление */
#define MPU6050_PWR_MGMT_1      0x6B  /* Управление питанием 1 */
#define MPU6050_PWR_MGMT_2      0x6C  /* Управление питанием 2 */
#define MPU6050_FIFO_COUNTH     0x72  /* Счётчик FIFO, старший байт */
#define MPU6050_FIFO_COUNTL     0x73  /* Счётчик FIFO, младший байт */
#define MPU6050_FIFO_R_W        0x74  /* Чтение/запись FIFO */
#define MPU6050_WHO_AM_I        0x75  /* Регистр идентификации устройства */

/* Exported types ------------------------------------------------------------*/

/** 
 * @brief Выбор устройства на шине I2C (состояние пина AD0) 
 */
typedef enum {
    MPU6050_Device_0 = 0x00, /* AD0 = низкий уровень (адрес 0x68) */
    MPU6050_Device_1 = 0x01  /* AD0 = высокий уровень (адрес 0x69) */
} MPU6050_Device_t;

/** 
 * @brief Коды результатов операций с MPU6050 
 */
typedef enum {
    MPU6050_Result_Ok = 0x00,          /* Операция выполнена успешно */
    MPU6050_Result_Error,              /* Неизвестная ошибка */
    MPU6050_Result_DeviceNotConnected, /* Устройство не обнаружено на шине I2C */
    MPU6050_Result_DeviceInvalid       /* WHO_AM_I не соответствует MPU6050 */
} MPU6050_Result_t;

/** 
 * @brief Диапазон измерения акселерометра 
 */
typedef enum {
    MPU6050_Accelerometer_2G  = 0x00, /* ±2g  */
    MPU6050_Accelerometer_4G  = 0x01, /* ±4g  */
    MPU6050_Accelerometer_8G  = 0x02, /* ±8g  */
    MPU6050_Accelerometer_16G = 0x03  /* ±16g */
} MPU6050_Accelerometer_t;

/** 
 * @brief Диапазон измерения гироскопа 
 */
typedef enum {
    MPU6050_Gyroscope_250s  = 0x00, /* ±250 °/с  */
    MPU6050_Gyroscope_500s  = 0x01, /* ±500 °/с  */
    MPU6050_Gyroscope_1000s = 0x02, /* ±1000 °/с */
    MPU6050_Gyroscope_2000s = 0x03  /* ±2000 °/с */
} MPU6050_Gyroscope_t;

/** 
 * @brief Основная структура данных датчика MPU6050 
 */
typedef struct {   I2C_HandleTypeDef *hi2c; /* Обработчик I2C интерфейса */
    uint8_t addr;            /* I2C-адрес устройства */
    float Gyro_Mult;         /* Коэффициент: raw → °/с */
    float Acce_Mult;         /* Коэффициент: raw → g */
    
    float Ax, Ay, Az;                    /* Данные акселерометра */
    float Gx, Gy, Gz;                    /* Данные гироскопа */
    int16_t Ax_Raw, Ay_Raw, Az_Raw;      /* Сырые данные акселерометра */
    int16_t Gx_Raw, Gy_Raw, Gz_Raw;      /* Сырые данные гироскопа */
    
    int16_t Ax_Offset, Ay_Offset, Az_Offset; /* Калибровочные смещения акселерометра */
    int16_t Gx_Offset, Gy_Offset, Gz_Offset; /* Калибровочные смещения гироскопа */
    
    float Temperature;       /* Температура (°C) */

    float Gyro_TempCoeff[3]; /* Температурные коэффициенты гироскопа (X, Y, Z) */
    float Acce_TempCoeff[3]; /* Температурные коэффициенты акселерометра (X, Y, Z) */
    float Base_Temperature;  /* Базовая температура калибровки (°C) */
} MPU6050_t;

/** 
 * @brief Структура статуса прерываний MPU6050 
 */
typedef union {
    struct {
        uint8_t DataReady : 1;       /* Данные готовы */
        uint8_t reserved2 : 2;       /* Зарезервировано */
        uint8_t Master : 1;          /* Прерывание режима I2C-ведущего */
        uint8_t FifoOverflow : 1;    /* Переполнение FIFO */
        uint8_t reserved1 : 1;       /* Зарезервировано */
        uint8_t MotionDetection : 1; /* Обнаружено движение */
        uint8_t reserved0 : 1;       /* Зарезервировано */
    } F;
    uint8_t Status; /* Побитовый доступ к регистру статуса */
} MPU6050_Interrupt_t;

/* Exported functions ------------------------------------------------------- */

/**
 * @brief                             Инициализация датчика MPU6050
 * @param  DataStruct                 Указатель на структуру данных MPU6050
 * @param  hi2c                       Обработчик I2C интерфейса
 * @param  DeviceNumber               Номер устройства (AD0 = LOW/HIGH)
 * @param  AccelerometerSensitivity   Диапазон акселерометра
 * @param  GyroscopeSensitivity       Диапазон гироскопа
 * @return                            Результат операции 
 */
MPU6050_Result_t MPU6050_Init(MPU6050_t *DataStruct, I2C_HandleTypeDef *hi2c, 
                             MPU6050_Device_t DeviceNumber,
                             MPU6050_Accelerometer_t AccelerometerSensitivity,
                             MPU6050_Gyroscope_t GyroscopeSensitivity);

/**
 * @brief                        Установка диапазона гироскопа
 * @param  DataStruct            Указатель на структуру данных MPU6050
 * @param  GyroscopeSensitivity  Новый диапазон 
 * @return                       Результат операции 
 */
MPU6050_Result_t MPU6050_SetGyroscope(MPU6050_t* DataStruct,
                                     MPU6050_Gyroscope_t GyroscopeSensitivity);

/**
 * @brief                              Установка диапазона акселерометра
 * @param  DataStruct                  Указатель на структуру данных MPU6050
 * @param  AccelerometerSensitivity    Новый диапазон
 * @return                             Результат операции 
 */
MPU6050_Result_t MPU6050_SetAccelerometer(MPU6050_t* DataStruct,
                                         MPU6050_Accelerometer_t AccelerometerSensitivity);

/**
 * @brief              Установка частоты дискретизации
 * @param  DataStruct  Указатель на структуру данных MPU6050
 * @param  rate        Значение делителя 
 * @return             Результат операции 
 */
MPU6050_Result_t MPU6050_SetDataRate(MPU6050_t* DataStruct, uint8_t rate);

/**
 * @brief              Включение прерываний MPU6050 
 * @param  DataStruct  Указатель на структуру данных MPU6050
 * @return             Результат операции 
 */
MPU6050_Result_t MPU6050_EnableInterrupts(MPU6050_t* DataStruct);

/**
 * @brief              Отключение прерываний MPU6050
 * @param  DataStruct  Указатель на структуру данных MPU6050
 * @return             Результат операции 
 */
MPU6050_Result_t MPU6050_DisableInterrupts(MPU6050_t* DataStruct);

/**
 * @brief                    Чтение и сброс флагов прерываний
 * @param  DataStruct        Указатель на структуру данных MPU6050
 * @param  InterruptsStruct  Указатель на структуру для записи статуса
 * @return                   Результат операции 
 */
MPU6050_Result_t MPU6050_ReadInterrupts(MPU6050_t* DataStruct,
                                       MPU6050_Interrupt_t* InterruptsStruct);

/**
 * @brief              Чтение данных акселерометра
 * @param  DataStruct  Указатель на структуру данных MPU6050
 * @return             Результат операции 
 */
MPU6050_Result_t MPU6050_ReadAccelerometer(MPU6050_t* DataStruct);

/**
 * @brief              Чтение данных гироскопа
 * @param  DataStruct  Указатель на структуру данных MPU6050
 * @return             Результат операции 
 */
MPU6050_Result_t MPU6050_ReadGyroscope(MPU6050_t* DataStruct);

/**
 * @brief              Чтение температуры датчика
 * @param  DataStruct  Указатель на структуру данных MPU6050
 * @return             Результат операции 
 */
MPU6050_Result_t MPU6050_ReadTemperature(MPU6050_t* DataStruct);

/**
 * @brief              Чтение всех данных (акселерометр, гироскоп, температура) за одну транзакцию I2C
 * @param  DataStruct  Указатель на структуру данных MPU6050
 * @return             Результат операции 
 */
MPU6050_Result_t MPU6050_ReadAll(MPU6050_t* DataStruct);

/**
 * @brief              Калибровка гироскопа
 * @param  DataStruct  Указатель на структуру данных MPU6050
 * @param  samples     Количество отсчётов для усреднения
 * @return             Результат операции 
 */
MPU6050_Result_t MPU6050_CalibrateGyro(MPU6050_t* DataStruct, uint16_t samples);

/**
 * @brief              Калибровка акселерометра
 * @param  DataStruct  Указатель на структуру данных MPU6050
 * @param  samples     Количество отсчётов для усреднения
 * @return             Результат операции 
 */
MPU6050_Result_t MPU6050_CalibrateAccel(MPU6050_t* DataStruct, uint16_t samples);
/**
 * @brief              Полная калибровка датчика 
 * @note               Датчик должен быть неподвижен, ось Z направлена вверх
 * @param  DataStruct  Указатель на структуру данных MPU6050
 * @param  samples     Количество отсчётов для усреднения
 * @return             Результат операции 
 */
MPU6050_Result_t MPU6050_CalibrateSensor(MPU6050_t* DataStruct, uint16_t samples);

#endif /* __MPU6050_H__ */             
