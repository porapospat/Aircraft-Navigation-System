/**
 * @file    mpu_math.c
 * @brief   Математические функции для работы с MPU/AHRS.
 */

 /* Includes ------------------------------------------------------------------*/
#include "mpu_math.h"

/* Private functions ---------------------------------------------------------*/
/**
 * @brief     Функция модуля числа 
 * @param  x  Входное значение
 * @return    Абсолютное значение аргумента 
 */
float fast_fabsf(float x)
{
    return (x < 0.0f) ? -x : x;
}

/**
 * @brief     Вычисление обратного квадратного корня 1 / sqrt(x)
 * @param  x  Входное положительное значение
 * @return    Приближённое значение 1 / sqrt(x)
 */
float fast_inv_sqrtf(float x)
{
    union { float f; int32_t i; } conv;
    conv.f = x;
    conv.i = 0x5F3759DF - (conv.i >> 1);
    conv.f = conv.f * (1.5f - 0.5f * x * conv.f * conv.f);
    return conv.f;
}

/**
 * @brief     Вычисление квадратного корня sqrt(x)
 * @param  x  Входное положительное значение
 * @return    Приближённое значение квадратного корня sqrt(x)
 */
float fast_sqrtf(float x)
{
    return x * fast_inv_sqrtf(x);
}

/**
 * @brief     Вычисление atan2(y, x) в радианах
 * @param  y  Координата Y
 * @param  x  Координата X
 * @return    Угол в радианах в диапазоне [-pi, pi]
 */
float fast_atan2f(float y, float x)
{
    float ax = fast_fabsf(x);
    float ay = fast_fabsf(y);
    float mn = (ax < ay) ? ax : ay;
    float mx = (ax > ay) ? ax : ay;

    if (mx < 1e-12f)
        return 0.0f;

    float a = mn / mx;
    float s = a * a;

    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;

    if (ay > ax)
        r = 1.57079637f - r;
    if (x < 0.0f)
        r = 3.14159274f - r;
    if (y < 0.0f)
        r = -r;

    return r;
}

/**
 * @brief      Вычисление arcsin(x) в радианах
 * @param  x   Входное значение в диапазоне [-1, 1]
 * @return     Приближённое значение угла в радианах в диапазоне [-pi/2, pi/2]
 */
float fast_asinf(float x)
{
    float ax = fast_fabsf(x);
    if (ax > 1.0f)
        ax = 1.0f;

    float t = fast_sqrtf(1.0f - ax);
    float r = ((0.0742610f * ax - 0.2121144f) * ax + 1.5707288f);
    r = 1.57079637f - t * r;

    return (x < 0.0f) ? -r : r;
}