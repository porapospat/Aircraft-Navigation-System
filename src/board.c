/**
 * @file    board.c
 * @brief   Базовая конфигурация 
 */

/* Includes ------------------------------------------------------------------*/
#include "board.h"

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Конфигурация системы тактирования 
 */
void SystemClock_Config(void)
{
    PCC_InitTypeDef PCC_OscInit = {0};
    PCC_OscInit.OscillatorEnable = PCC_OSCILLATORTYPE_ALL;
    PCC_OscInit.FreqMon.OscillatorSystem = PCC_OSCILLATORTYPE_OSC32M;
    PCC_OscInit.FreqMon.ForceOscSys = PCC_FORCE_OSC_SYS_UNFIXED;
    PCC_OscInit.FreqMon.Force32KClk = PCC_FREQ_MONITOR_SOURCE_OSC32K;
    PCC_OscInit.AHBDivider = 0;
    PCC_OscInit.APBMDivider = 0;
    PCC_OscInit.APBPDivider = 0;
    PCC_OscInit.HSI32MCalibrationValue = 128;
    PCC_OscInit.LSI32KCalibrationValue = 8;
    PCC_OscInit.RTCClockSelection = PCC_RTC_CLOCK_SOURCE_AUTO;
    PCC_OscInit.RTCClockCPUSelection = PCC_CPU_RTC_CLOCK_SOURCE_OSC32K;
    HAL_PCC_Config(&PCC_OscInit);
}

/**
 * @brief  Чтение счётчика тактов процессора (CSR mcycle, ядро SCR1)
 * @return Текущее значение 32-битного счётчика тактов
 */
 uint32_t read_mcycle(void)
{
    uint32_t val;
    __asm__ volatile("csrr %0, mcycle" : "=r"(val));
    return val;
}