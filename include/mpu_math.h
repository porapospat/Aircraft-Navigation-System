/**
 * @file    mpu_math.h
 * @brief   Математические функции для работы с MPU/AHRS.
 */

#ifndef MPU_MATH_H
#define MPU_MATH_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported define -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 * @brief     Функция модуля числа 
 * @param  x  Входное значение
 * @return    Абсолютное значение аргумента 
 */
float fast_fabsf(float x);

/**
 * @brief     Вычисление обратного квадратного корня 1 / sqrt(x)
 * @param  x  Входное положительное значение
 * @return    Приближённое значение 1 / sqrt(x)
 */
float fast_inv_sqrtf(float x);

/**
 * @brief     Вычисление квадратного корня sqrt(x)
 * @param  x  Входное положительное значение
 * @return    Приближённое значение квадратного корня sqrt(x)
 */
float fast_sqrtf(float x);

/**
 * @brief     Вычисление atan2(y, x) в радианах
 * @param  y  Координата Y
 * @param  x  Координата X
 * @return    Угол в радианах в диапазоне [-pi, pi]
 */
float fast_atan2f(float y, float x);

/**
 * @brief      Вычисление arcsin(x) в радианах
 * @param  x   Входное значение в диапазоне [-1, 1]
 * @return     Приближённое значение угла в радианах в диапазоне [-pi/2, pi/2]
 */
float fast_asinf(float x);

#endif /* __MPU_MATH_H__ */
