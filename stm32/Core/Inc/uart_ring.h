/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    uart_ring.h
  * @brief   Interrupt-driven UART RX ring buffer API.
  *
  * Wave 2 Task 8: ISR-safe write (USART1 RX interrupt), main-loop read.
  * - 256-byte static ring buffer, no malloc, no DMA
  * - Overflow discards oldest byte and increments overrun counter
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_RING_H
#define __UART_RING_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/
#define UART_RX_BUF_SIZE 256

/* Exported functions prototypes ---------------------------------------------*/
void uart_ring_init(UART_HandleTypeDef* huart);
bool uart_ring_read_byte(uint8_t* out);
uint16_t uart_ring_available(void);
uint16_t uart_ring_overrun_count(void);

#ifdef __cplusplus
}
#endif

#endif /* __UART_RING_H */