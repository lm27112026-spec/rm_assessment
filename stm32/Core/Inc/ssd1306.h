/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ssd1306.h
  * @brief   SSD1306 128x64 OLED (I2C) text-only driver API.
  *
  * Wave 2 Task 10:
  * - Text-only rendering with built-in 6x8 font (ASCII 0x20-0x7E)
  * - 1024-byte framebuffer, page-based flush over I2C1
  * - All functions are blocking (HAL_I2C_Master_Transmit) and must be
  *   called from main-loop context only, NEVER from an ISR.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SSD1306_H
#define __SSD1306_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <stdint.h>

/* Exported defines ----------------------------------------------------------*/
#define SSD1306_I2C_ADDR  0x78   /* 0x3C << 1 */

/* Exported functions prototypes ---------------------------------------------*/
void ssd1306_init(I2C_HandleTypeDef* hi2c);
void ssd1306_clear(void);
void ssd1306_draw_string(uint8_t x_col, uint8_t y_page, const char* str);
void ssd1306_draw_char(uint8_t x_col, uint8_t y_page, char c);
void ssd1306_flush(void);
void ssd1306_set_cursor(uint8_t x, uint8_t y);

#ifdef __cplusplus
}
#endif

#endif /* __SSD1306_H */