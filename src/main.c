/**
* @brief Проект реализует прием, обработку и передачу телеметрии с борта на 
* наземную станцию управления
*/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "bmp280.h"
#include "ina219.h"
#include "mpu6050.h"
#include "ahrsimu.h"
#include "minmea.h"
#include "gps.h"
#include "ui.h"

/* Defines -------------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USART_HandleTypeDef husart0;  
USART_HandleTypeDef husart1; 
I2C_HandleTypeDef hi2c0; 
I2C_HandleTypeDef hi2c1;      

BMP280_HandleTypeDef hbmp280;
INA219_t hina219;
MPU6050_t mpu6050;
AHRSIMU_t ahrs;

char sentence[MAX_SENTENCE_LENGTH]; 

/* Private function prototypes -----------------------------------------------*/
static void Periph_Init(void);
void Process_MPU6050_Data(void);
void Process_GGA_Data(void);
void Process_BMP_Data(void);
void Process_INA219_Data(void);

int main()
{    
    SystemClock_Config();
    Periph_Init();

    HAL_USART_Print(&husart1, "start...\n", USART_TIMEOUT_DEFAULT);    
  
    __HAL_PCC_EPIC_CLK_ENABLE();
    HAL_EPIC_MaskLevelSet(HAL_EPIC_UART_0_MASK | HAL_EPIC_UART_1_MASK); 
    HAL_IRQ_EnableInterrupts();

    /* Разрешить прерывания по признаку "регистр приемника не пуст" */
    HAL_USART_RXNE_EnableInterrupt(&husart0);

    while (1)
    {       
        /* крен, тангаж и курс с MPU6050 */ 
        Process_MPU6050_Data();
        /* широта и долгота из GGA предложения GPS модуля */
        Process_GGA_Data(); 
        /* давление и температура с BMP280 */
        Process_BMP_Data();   
        /* напряжение, ток и мощность с INA219 */
        Process_INA219_Data();
               
        HAL_DelayMs(5);
    }
}

/** @brief  Инициализация периферии */
static void Periph_Init(void)
{ 
    I2C0_Init(&hi2c0);
    I2C1_Init(&hi2c1);
    USART0_Init(&husart0);
    USART1_Init(&husart1);

    BMP280_Init(&hbmp280, &hi2c1, BMP280_I2C_ADDR);
    INA219_Init(&hina219, &hi2c1, INA219_ADDRESS);

    if (MPU6050_Init(&mpu6050, &hi2c0, MPU6050_Device_0, MPU6050_Accelerometer_2G, MPU6050_Gyroscope_250s) == MPU6050_Result_Ok)
    {
        MPU6050_CalibrateSensor(&mpu6050, 100);
        AHRSIMU_Init(&ahrs, 31.0f, 0.1f, 0.0f);
    }
}

/** @brief  Обработчик прерываний */
void trap_handler()
{
    if (EPIC_CHECK_UART_0())
    {
        UART0_IRQHandler(&husart0);
    }
    
    if (EPIC_CHECK_UART_1())
    {
        USART1_IRQHandler(&husart1);
    }
    
    HAL_EPIC_Clear(0xFFFFFFFF);
}

/**
 * @brief  Чтение MPU6050, обновление AHRS-фильтра и отправка пакета данных
 * Крен и тангаж — из фильтра Madgwick (коррекция по акселерометру).
 */
void Process_MPU6050_Data(void)
{	
    static float yaw_gyro = 0.0f;
    static uint32_t prev_cycle = 0;
    static uint8_t first_run = 1;

    if (MPU6050_ReadAll(&mpu6050) == MPU6050_Result_Ok) 
    {
        uint32_t now = read_mcycle();
        float dt;

        if (first_run) {
            dt = DT_DEFAULT;
            first_run = 0;
        } else {
            uint32_t delta = now - prev_cycle;
            dt = (float)delta / (float)CPU_FREQ_HZ;
            if (dt < DT_MIN || dt > DT_MAX)
                dt = DT_DEFAULT;
        }
        prev_cycle = now;

        ahrs._sampleRate = dt;

        AHRSIMU_UpdateIMU(&ahrs, 
                        AHRSIMU_DEG2RAD(mpu6050.Gx), AHRSIMU_DEG2RAD(mpu6050.Gy), AHRSIMU_DEG2RAD(mpu6050.Gz),  
                        mpu6050.Ax, mpu6050.Ay, mpu6050.Az); 

        float gz = mpu6050.Gz;
        if (fast_fabsf(gz) < GYRO_DEAD_ZONE)
            gz = 0.0f;

        yaw_gyro += gz * dt;
        if (yaw_gyro > 180.0f)  yaw_gyro -= 360.0f;
        if (yaw_gyro < -180.0f) yaw_gyro += 360.0f;

        UI_SendMPUMSG(&husart1, ahrs.Roll, ahrs.Pitch, yaw_gyro, 
                        mpu6050.Ax, mpu6050.Ay, mpu6050.Az,
                        mpu6050.Gx, mpu6050.Gy, mpu6050.Gz);
    }
}

/** @brief  Разбор GGA-предложения от GPS и отправка координат.*/
void Process_GGA_Data(void)
{
    if (GPS_GetSentence(sentence, MAX_SENTENCE_LENGTH))
    {
        if (minmea_sentence_id(sentence, false) == MINMEA_SENTENCE_GGA)
        {
            struct minmea_sentence_gga frame;
            if (minmea_parse_gga(&frame, sentence))
            {
                double lat = minmea_tocoord(&frame.latitude);
                double lon = minmea_tocoord(&frame.longitude);
                float height = (float)minmea_tofloat(&frame.altitude); /* метры */

                uint8_t gps_valid = (frame.fix_quality > 0) ? GPS_STATUS_VALID : GPS_STATUS_INVALID;

                UI_SendGPSMSG(&husart1, lat, lon, gps_valid, height);
            }
        }
    }
}

/** @brief Отправка данных с датчика BMP280 - температуры и давления */
void Process_BMP_Data()
{
    int32_t pressure = 0;
    int32_t temperature = 0;

    if (BMP280_ReadAll(&hbmp280) == BMP280_STATE_OK)
    {        
        BMP280_Calculate(&hbmp280, &pressure, &temperature);
        UI_SendBMPMSG(&husart1, pressure, temperature);
    }    
}

/** @brief Отправка данных с датчика INA219 — напряжения, тока и мощности */
void Process_INA219_Data()
{
    UI_SendINA219MSG(&husart1, INA219_ReadBusVoltage(&hina219), INA219_ReadCurrent(&hina219), INA219_ReadPower(&hina219));
}