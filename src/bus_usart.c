/**
 * @file    bus_usart.c
 * @brief   Реализация драйвера USART 
 */

/* Includes ------------------------------------------------------------------*/
#include "bus_usart.h"

/* Private define ------------------------------------------------------------*/
#define countof(a)           (sizeof(a) / sizeof(*(a)))

#define RxBufferSize          254  /* Размер приёмного кольцевого буфера */
#define TxBufferSize          254  /* Размер передающего кольцевого буфера */

/* Private variables ---------------------------------------------------------*/
static uint8_t RxBuffer[RxBufferSize];
static volatile uint32_t RxHead = 0;
static volatile uint32_t RxTail = 0;

static uint8_t TxBuffer[TxBufferSize];
static volatile uint32_t TxHead = 0;
static volatile uint32_t TxTail = 0;

/* Private function prototypes -----------------------------------------------*/
static void CircIdxInc(volatile uint32_t *p, uint32_t sz);
static uint32_t RxFree(void);
static uint32_t TxFree(void);
static void RxPut(uint8_t ch);
static void TxPut(uint8_t ch);
static uint8_t RxGet(void);
static uint8_t TxGet(void);
static void UART_TxRecover(USART_HandleTypeDef *husart);
static uint8_t UART_SendFrameTry(USART_HandleTypeDef *husart, uint8_t *str, uint32_t len);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Инициализация USART0 
 * @param  husart  Указатель на дескриптор USART
 */
void USART0_Init(USART_HandleTypeDef *husart)
{
    husart->Instance = UART_0;
    husart->transmitting = Enable;
    husart->receiving = Enable;
    husart->frame = Frame_8bit;
    husart->parity_bit = Disable;
    husart->parity_bit_inversion = Disable;
    husart->bit_direction = LSB_First;
    husart->data_inversion = Disable;
    husart->tx_inversion = Disable;
    husart->rx_inversion = Disable;
    husart->swap = Disable;
    husart->lbm = Disable;
    husart->stop_bit = StopBit_1;
    husart->mode = Asynchronous_Mode;
    husart->xck_mode = XCK_Mode3;
    husart->last_byte_clock = Disable;
    husart->overwrite = Disable;
    husart->rts_mode = AlwaysEnable_mode;
    husart->dma_tx_request = Disable;
    husart->dma_rx_request = Disable;
    husart->channel_mode = Duplex_Mode;
    husart->tx_break_mode = Disable;
    husart->Interrupt.ctsie = Disable;
    husart->Interrupt.eie = Disable;
    husart->Interrupt.idleie = Disable;
    husart->Interrupt.lbdie = Disable;
    husart->Interrupt.peie = Disable;
    husart->Interrupt.rxneie = Enable;  /* Прерывание по приёму */
    husart->Interrupt.tcie = Disable;
    husart->Interrupt.txeie = Disable;
    husart->Modem.rts = Disable;
    husart->Modem.cts = Disable;
    husart->Modem.dtr = Disable;
    husart->Modem.dcd = Disable;
    husart->Modem.dsr = Disable;
    husart->Modem.ri = Disable;
    husart->Modem.ddis = Disable;
    husart->baudrate = BAUDRATE;
    HAL_USART_Init(husart);
}

/**
 * @brief          Инициализация USART1 
 * @param  husart  Указатель на дескриптор USART
 */
void USART1_Init(USART_HandleTypeDef *husart)
{
    husart->Instance = UART_1;    
    husart->transmitting = Enable;
    husart->receiving = Enable;
    husart->frame = Frame_8bit;
    husart->parity_bit = Disable;
    husart->parity_bit_inversion = Disable;
    husart->bit_direction = LSB_First;
    husart->data_inversion = Disable;
    husart->tx_inversion = Disable;
    husart->rx_inversion = Disable;
    husart->swap = Disable;
    husart->lbm = Disable;
    husart->stop_bit = StopBit_1;
    husart->mode = Asynchronous_Mode;
    husart->xck_mode = XCK_Mode3;
    husart->last_byte_clock = Disable;
    husart->overwrite = Disable;
    husart->rts_mode = AlwaysEnable_mode;
    husart->dma_tx_request = Disable;
    husart->dma_rx_request = Disable;
    husart->channel_mode = Duplex_Mode;
    husart->tx_break_mode = Disable;
    husart->Interrupt.ctsie = Disable;
    husart->Interrupt.eie = Disable;
    husart->Interrupt.idleie = Disable;
    husart->Interrupt.lbdie = Disable;
    husart->Interrupt.peie = Disable;
    husart->Interrupt.rxneie = Disable;
    husart->Interrupt.tcie = Disable;
    husart->Interrupt.txeie = Disable;
    husart->Modem.rts = Disable; 
    husart->Modem.cts = Disable; 
    husart->Modem.dtr = Disable; 
    husart->Modem.dcd = Disable; 
    husart->Modem.dsr = Disable; 
    husart->Modem.ri = Disable;  
    husart->Modem.ddis = Disable;
    husart->baudrate = BAUDRATE;
    
    HAL_USART_Init(husart);
}

/**
 * @brief          Обработчик прерываний UART0 (приём данных с gps модуля)
 * @param  husart  Указатель на дескриптор USART
 */
void UART0_IRQHandler(USART_HandleTypeDef *husart)
{
    if (HAL_USART_RXNE_ReadFlag(husart))
    {
        if (RxFree() > 0)
        {
            RxPut(HAL_USART_ReadByte(husart));
        }
        HAL_USART_RXNE_ClearFlag(husart);
    }

}

/**
 * @brief          Обработчик прерываний UART1 (передача данных)
 * @param  husart  Указатель на дескриптор USART
 */
void USART1_IRQHandler(USART_HandleTypeDef *husart)
{
    
    if (HAL_USART_RXNE_ReadFlag(husart))
    {
        if (RxFree() > 0)
        {            
            RxPut(HAL_USART_ReadByte(husart));
        }
        HAL_USART_RXNE_ClearFlag(husart);
    }

    if (HAL_USART_TXE_ReadFlag(husart))
    {
        if (TxTail != TxHead)
        {
            uint8_t data = TxGet();
            HAL_USART_Transmit(husart, data, USART_TIMEOUT_DEFAULT);
        }
        else
        {
            HAL_USART_TXE_DisableInterrupt(husart);
            HAL_USART_TXC_EnableInterrupt(husart);
        }
    }

    if (HAL_USART_TXC_ReadFlag(husart))
    {
        HAL_USART_TXC_DisableInterrupt(husart);
        HAL_USART_TXC_ClearFlag(husart);
    }
}

/**
 * @brief          Отправка байта через кольцевой буфер с прерыванием TXE
 * @param  husart  Указатель на дескриптор USART
 * @param  Data    Байт для отправки
 */
void UART_SendByte(USART_HandleTypeDef *husart, uint8_t Data)
{
    if (TxFree() == 0)
    {
        return; 
    }
    
    HAL_USART_TXE_DisableInterrupt(husart);
    TxPut(Data);
    HAL_USART_TXE_EnableInterrupt(husart);
}

/**
 * @brief      Чтение байта из приёмного кольцевого буфера
 * @param  ch  Указатель для записи прочитанного байта
 * @return     Байт прочитан, иначе буфер пуст
 */
int8_t UART_ReadChar(uint8_t *ch)
{
    if(RxTail != RxHead)
    {
        *ch = RxGet();
        return true;                                      
    }
    return false;
}

/**
 * @brief          Поллинговая отправка массива байт
 * @param  husart  Указатель на дескриптор USART
 * @param  str     Указатель на буфер с данными
 * @param  len     Длина данных в байтах
 * @return         Все байты отправлены, иначе ошибка таймаута
 */
uint8_t UART_SendFrame(USART_HandleTypeDef *husart, uint8_t *str, uint32_t len)
{
    if (UART_SendFrameTry(husart, str, len))
    {
        return true;
    }

    for (uint8_t retry = 0; retry < UART_TX_RECOVERY_RETRIES; retry++)
    {
        UART_TxRecover(husart);
        if (UART_SendFrameTry(husart, str, len))
        {
            return true;
        }
    }

    return false;
}

/**
 * @brief      Инкремент индекса кольцевого буфера с переносом
 * @param  p   Указатель на индекс
 * @param  sz  Размер буфера
 */
static void CircIdxInc(volatile uint32_t *p, uint32_t sz)
{
    uint32_t idx = *p;
    if (++idx >= sz)
    {
        idx -= sz;
    }
    *p = idx;
}

/**
 * @brief  Количество свободных байт в приёмном буфере
 * @return Количество свободных байт
 */
static uint32_t RxFree(void)
{
    uint32_t RxFree = countof(RxBuffer) + RxTail - RxHead;

    if (RxFree > countof(RxBuffer))
    {
        RxFree -= countof(RxBuffer);
    }

    return RxFree;
}

/**
 * @brief  Количество свободных байт в передающем буфере
 * @return Количество свободных байт
 */
static uint32_t TxFree()
{
   uint32_t TxFree = countof(TxBuffer) + TxTail - TxHead;

   if (TxFree > countof(TxBuffer))
   {
      TxFree -= countof(TxBuffer);
   }
   return TxFree;
 }

/**
 * @brief      Добавление байта в приёмный буфер
 * @param  ch  Байт для добавления
 */
static void RxPut(uint8_t ch)
{
    RxBuffer[RxHead] = ch;
    CircIdxInc (&RxHead, countof(RxBuffer));
}

/**
 * @brief      Добавление байта в передающий буфер
 * @param  ch  Байт для добавления
 */
static void TxPut(uint8_t ch)
{
   TxBuffer[TxHead] = ch;
   CircIdxInc (&TxHead, countof(TxBuffer));
}

/**
 * @brief  Извлечение байта из приёмного буфера
 * @return Прочитанный байт
 */
static uint8_t RxGet(void)
{
    uint8_t ch = RxBuffer[RxTail];
    CircIdxInc (&RxTail, countof(RxBuffer));

    return ch;
}

/**
 * @brief  Извлечение байта из передающего буфера
 * @return Прочитанный байт
 */
static uint8_t TxGet(void)
{
   uint8_t ch = TxBuffer[TxTail];
   CircIdxInc (&TxTail, countof(TxBuffer));

   return ch;
}

/**
 * @brief Попытка отправить кадр без восстановления
 */
static uint8_t UART_SendFrameTry(USART_HandleTypeDef *husart, uint8_t *str, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        if (!HAL_USART_Transmit(husart, str[i], UART_TX_BYTE_TIMEOUT_MS))
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief Восстановление UART TX после ошибки (переинициализация + очистка TX буфера)
 */
static void UART_TxRecover(USART_HandleTypeDef *husart)
{
    HAL_USART_TXE_DisableInterrupt(husart);
    HAL_USART_TXC_DisableInterrupt(husart);

    TxHead = 0;
    TxTail = 0;

    HAL_USART_Init(husart);
}
