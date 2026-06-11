/**
 * @file    board.h
 * @brief   Базовая конфигурация 
 */
#ifndef __BOARD_H__
#define __BOARD_H__

/* Includes ------------------------------------------------------------------*/
#include "mik32_hal_pcc.h"

/* Defines ------------------------------------------------------------------*/
#define CPU_FREQ_HZ       32000000UL  /* Тактовая частота SCR1 (32 МГц) */
#define DT_DEFAULT        0.032f      /* Период цикла по умолчанию (с) */
#define DT_MIN            0.002f      /* Минимально допустимый dt */
#define DT_MAX            0.200f      /* Максимально допустимый dt */
#define GYRO_DEAD_ZONE    0.2f        /* Мёртвая зона гироскопа (°/с) */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief   Конфигурация системы тактирования 
 * @details Настраивает источник тактового сигнала, делители частоты
 *          AHB/APB и источник тактирования RTC
 */
void SystemClock_Config(void);
uint32_t read_mcycle(void);

#endif /* __BOARD_H__ */
