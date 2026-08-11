/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    uart_ring.c
  * @brief   Interrupt-driven UART RX ring buffer implementation.
  *
  * Wave 2 Task 8:
  * - ISR (HAL_UART_RxCpltCallback) writes bytes at rx_head_, re-arms the
  *   single-byte receive interrupt. Minimal: no printf, no I2C, no OLED,
  *   no blocking calls.
  * - Main loop reads bytes at rx_tail_ via uart_ring_read_byte().
  * - Overflow discards the oldest byte and increments rx_overrun_.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "uart_ring.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef* huart_ = NULL;
static uint8_t rx_buf_[UART_RX_BUF_SIZE];
static volatile uint16_t rx_head_ = 0;
static volatile uint16_t rx_tail_ = 0;
static uint16_t rx_overrun_ = 0;
static uint8_t rx_byte_;  /* single byte receive buffer */

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize the ring buffer and start single-byte RX interrupt.
  * @param  huart: pointer to the UART handle (e.g. &huart1)
  * @retval None
  */
void uart_ring_init(UART_HandleTypeDef* huart)
{
  huart_ = huart;
  rx_head_ = 0;
  rx_tail_ = 0;
  rx_overrun_ = 0;
  HAL_UART_Receive_IT(huart_, &rx_byte_, 1);
}

/**
  * @brief  UART RX complete callback (weak in HAL, overridden here).
  *         Called from USART1_IRQHandler via HAL_UART_IRQHandler.
  *         ISR context: must stay minimal and non-blocking.
  * @param  huart: pointer to the UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
  if (huart->Instance != USART1) return;

  uint16_t next_head = (rx_head_ + 1) % UART_RX_BUF_SIZE;

  if (next_head == rx_tail_)
  {
    /* Buffer full — discard oldest byte */
    rx_tail_ = (rx_tail_ + 1) % UART_RX_BUF_SIZE;
    rx_overrun_++;
  }

  rx_buf_[rx_head_] = rx_byte_;
  rx_head_ = next_head;

  /* Re-arm interrupt for next byte */
  HAL_UART_Receive_IT(huart_, &rx_byte_, 1);
}

/**
  * @brief  Read one byte from the ring buffer (main-loop context).
  * @param  out: pointer to destination byte
  * @retval true if a byte was read, false if buffer empty
  */
bool uart_ring_read_byte(uint8_t* out)
{
  if (rx_head_ == rx_tail_)
  {
    return false;  /* buffer empty */
  }

  *out = rx_buf_[rx_tail_];
  rx_tail_ = (rx_tail_ + 1) % UART_RX_BUF_SIZE;
  return true;
}

/**
  * @brief  Number of bytes currently available in the ring buffer.
  * @retval available byte count
  */
uint16_t uart_ring_available(void)
{
  if (rx_head_ >= rx_tail_)
  {
    return rx_head_ - rx_tail_;
  }
  else
  {
    return UART_RX_BUF_SIZE - rx_tail_ + rx_head_;
  }
}

/**
  * @brief  Number of bytes discarded due to buffer overflow.
  * @retval overrun count
  */
uint16_t uart_ring_overrun_count(void)
{
  return rx_overrun_;
}

/* USER CODE BEGIN 2 */
/* USER CODE END 2 */