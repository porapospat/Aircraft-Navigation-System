/**
 * @file    ui.c
 * @brief   Формирование и отправка бинарных пакетов MPU6050, GPS, BMP, INA219
 */

/* Includes ------------------------------------------------------------------*/
#include "ui.h"

/* Private variables ---------------------------------------------------------*/
static sGPS_MSG GPS_MSG_TX; /* Буфер передачи пакета GPS */
static sMPU_MSG MPU_MSG_TX; /* Буфер передачи пакета MPU6050 */
static sBMP_MSG BMP_MSG_TX; /* Буфер передачи пакета BMP280 */
static sINA219_MSG INA219_MSG_TX; /* Буфер передачи пакета INA219 */

/* Private functions ---------------------------------------------------------*/

/**
 * @brief          Формирование и отправка пакета данных MPU6050
 * @param  husart  Указатель на дескриптор USART
 * @param  roll    Крен 
 * @param  pitch   Тангаж 
 * @param  yaw     Курс (рыскание) 
 * @param  Ax      Ускорение X 
 * @param  Ay      Ускорение Y 
 * @param  Az      Ускорение Z 
 * @param  Gx      Угловая скорость X 
 * @param  Gy      Угловая скорость Y 
 * @param  Gz      Угловая скорость Z 
 */
uint8_t UI_SendMPUMSG(USART_HandleTypeDef *husart, float roll, float pitch, float yaw, float Ax, float Ay, float Az, float Gx, float Gy, float Gz)
{
	MPU_MSG_TX.HDR  = MSG_HEADER;
    MPU_MSG_TX.ID 	= MSG_MPU_ID;
    MPU_MSG_TX.LEN  = MSG_DATA_SIZE;
	
    MPU_MSG_TX.DATA[0] = roll;
    MPU_MSG_TX.DATA[1] = pitch;
    MPU_MSG_TX.DATA[2] = yaw;
    MPU_MSG_TX.DATA[3] = Ax;
    MPU_MSG_TX.DATA[4] = Ay;
    MPU_MSG_TX.DATA[5] = Az;
    MPU_MSG_TX.DATA[6] = Gx;
    MPU_MSG_TX.DATA[7] = Gy;
    MPU_MSG_TX.DATA[8] = Gz;
    
    uint16_t packet_size = sizeof(MPU_MSG_TX.HDR) + sizeof(MPU_MSG_TX.ID) + 
                       sizeof(MPU_MSG_TX.LEN) + sizeof(MPU_MSG_TX.DATA) + 2;
    
	MPU_MSG_TX.CRC16 = CRC16_Calc((uint8_t*)&MPU_MSG_TX.DATA, sizeof(MPU_MSG_TX.DATA));
    
    return UART_SendFrame(husart, (uint8_t*)&MPU_MSG_TX, packet_size);
}

/**
 * @brief             Формирование и отправка пакета данных GPS
 * @param  husart     Указатель на дескриптор USART
 * @param  latitude   Широта 
 * @param  longitude  Долгота 
 * @param  gps_valid  Флаг валидности
 */
uint8_t UI_SendGPSMSG(USART_HandleTypeDef *husart, double latitude, double longitude, uint8_t gps_valid, float height)
{    
    GPS_MSG_TX.HDR 	= MSG_HEADER;
    GPS_MSG_TX.ID 	= GPS_MSG_ID;
    GPS_MSG_TX.LEN 	= GPS_MSG_DATA_SIZE;
    
    if (gps_valid == 0) {
        GPS_MSG_TX.DATA[0] = 0.0f;  
        GPS_MSG_TX.DATA[1] = 0.0f;  
    } else {
        GPS_MSG_TX.DATA[0] = (float)latitude;  
        GPS_MSG_TX.DATA[1] = (float)longitude; 
	}
    
    GPS_MSG_TX.DATA[2] = height;

    uint16_t packet_size = sizeof(GPS_MSG_TX.HDR) + sizeof(GPS_MSG_TX.ID) + 
                          sizeof(GPS_MSG_TX.LEN) + sizeof(GPS_MSG_TX.DATA) + 2;

    GPS_MSG_TX.CRC16 = CRC16_Calc((uint8_t*)&GPS_MSG_TX.DATA, sizeof(GPS_MSG_TX.DATA));

    return UART_SendFrame(husart, (uint8_t*)&GPS_MSG_TX, packet_size);
}


/**
 * @brief               Формирование и отправка пакета данных BMP
 * @param  husart       Указатель на дескриптор USART
 * @param  pressure     Давление
 * @param  temperature  Температура
 */
uint8_t UI_SendBMPMSG(USART_HandleTypeDef *husart, float pressure, float temperature)
{
    BMP_MSG_TX.HDR = MSG_HEADER;
    BMP_MSG_TX.ID = BMP_MSG_ID;
    BMP_MSG_TX.LEN = BMP_MSG_DATA_SIZE;

    /* Измеренные значения давления и температуры */
    BMP_MSG_TX.DATA[0] = pressure;
    BMP_MSG_TX.DATA[1] = temperature;

    uint16_t packet_size = sizeof(BMP_MSG_TX.HDR) + sizeof(BMP_MSG_TX.ID) + 
                       sizeof(BMP_MSG_TX.LEN) + sizeof(BMP_MSG_TX.DATA) + 2;
    
	BMP_MSG_TX.CRC16 = CRC16_Calc((uint8_t*)&BMP_MSG_TX.DATA, sizeof(BMP_MSG_TX.DATA));
    
    return UART_SendFrame(husart, (uint8_t*)&BMP_MSG_TX, packet_size);
}

/**
 * @brief               Формирование и отправка пакета данных INA219
 * @param  husart       Указатель на дескриптор USART
 * @param  bus_voltage  Напряжение, В
 * @param  current      Ток, А
 * @param  power        Мощность, Вт
 */
uint8_t UI_SendINA219MSG(USART_HandleTypeDef *husart, float bus_voltage, float current, float power)
{
    INA219_MSG_TX.HDR = MSG_HEADER;
    INA219_MSG_TX.ID = INA219_MSG_ID;
    INA219_MSG_TX.LEN = INA219_MSG_DATA_SIZE;

    INA219_MSG_TX.DATA[0] = bus_voltage;
    INA219_MSG_TX.DATA[1] = current;
    INA219_MSG_TX.DATA[2] = power;

    uint16_t packet_size = sizeof(INA219_MSG_TX.HDR) + sizeof(INA219_MSG_TX.ID) +
                           sizeof(INA219_MSG_TX.LEN) + sizeof(INA219_MSG_TX.DATA) + 2;

    INA219_MSG_TX.CRC16 = CRC16_Calc((uint8_t*)&INA219_MSG_TX.DATA, sizeof(INA219_MSG_TX.DATA));

    return UART_SendFrame(husart, (uint8_t*)&INA219_MSG_TX, packet_size);
}
