/**
 * @file    ahrsimu.h
 * @brief   Алгоритм AHRS/IMU на основе фильтра Madgwick
 */
#ifndef __AHRSIMU_H__
#define __AHRSIMU_H__

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "mpu_math.h"

/* Exported define -----------------------------------------------------------*/
#define AHRSIMU_PI              3.14159265358979324f     
#define AHRSIMU_RAD2DEG(x)      ((x) * 57.2957795f)     /* Радианы -> градусы */
#define AHRSIMU_DEG2RAD(x)      ((x) * 0.0174532925f)   /* Градусы -> радианы */

/**
 * @brief           Установка коэффициента beta фильтра Madgwick
 * @param  AHRSIMU  Указатель на структуру AHRSIMU_t
 * @param  beta     Новое значение коэффициента
 */
#define AHRSIMU_SetBeta(AHRSIMU, beta)  ((AHRSIMU)->_beta = (beta))

/* Exported types ------------------------------------------------------------*/

/** 
 * @brief Структура состояния AHRS/IMU фильтра 
 */
typedef struct _AHRSIMU_t
{
    float Roll;        /* Крен */
    float Pitch;       /* Тангаж */
    float Yaw;         /* Рыскание */
    float Inclination; /* Магнитное склонение */
    
    float _beta;       /* Коэффициент коррекции акселерометра */
    float _q0;         /* Компонент кватерниона w */
    float _q1;         /* Компонент кватерниона x */
    float _q2;         /* Компонент кватерниона y */
    float _q3;         /* Компонент кватерниона z */
    float _sampleRate; /* Период дискретизации (1 / частота), с */
} AHRSIMU_t;
/* Exported functions ------------------------------------------------------- */

/**
 * @brief               Инициализация AHRS/IMU фильтра
 * @param  AHRSIMU      Указатель на структуру AHRSIMU_t
 * @param  sampleRate   Частота обновления фильтра 
 * @param  beta         Коэффициент beta фильтра Madgwick
 * @param  inclination  Магнитное склонение 
 */
void AHRSIMU_Init(AHRSIMU_t* AHRSIMU, float sampleRate, float beta, float inclination);

/**
 * @brief           Обновление AHRS с данными акселерометра, гироскопа и магнитометра
 * @param  AHRSIMU  Указатель на структуру AHRSIMU_t
 * @param  gx       Угловая скорость X 
 * @param  gy       Угловая скорость Y 
 * @param  gz       Угловая скорость Z 
 * @param  ax       Ускорение X 
 * @param  ay       Ускорение Y 
 * @param  az       Ускорение Z 
 * @param  mx       Магнитное поле X
 * @param  my       Магнитное поле Y
 * @param  mz       Магнитное поле Z
 */
void AHRSIMU_UpdateAHRS(AHRSIMU_t* AHRSIMU, float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz);

/**
 * @brief           Обновление IMU с данными акселерометра и гироскопа (без магнитометра)
 * @param  AHRSIMU  Указатель на структуру AHRSIMU_t
 * @param  gx       Угловая скорость X 
 * @param  gy       Угловая скорость Y 
 * @param  gz       Угловая скорость Z 
 * @param  ax       Ускорение X 
 * @param  ay       Ускорение Y 
 * @param  az       Ускорение Z 
 */
void AHRSIMU_UpdateIMU(AHRSIMU_t* AHRSIMU, float gx, float gy, float gz,
                       float ax, float ay, float az);

#endif /* __AHRSIMU_H__ */
