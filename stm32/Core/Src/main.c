/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body.
  *
  * Wave 1 Task 4: STM32 CubeMX project skeleton.
  * - HAL init, SystemClock (HSE 8MHz -> PLL -> 72MHz), GPIO, USART1, I2C1
  * - USART1 RX interrupt enabled (single byte receive)
  * - PC13 LED blinks on each received frame (Wave 2 Task 9)
  *
  * NOTE: huart1 / hi2c1 are defined in usart.c / i2c.c (standard CubeMX
  * pattern) and declared extern via usart.h / i2c.h.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_ring.h"
#include "frame_parser.h"
#include "ssd1306.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* Latest parsed frame, consumed by OLED display (Wave 2 Task 10) */
static payload_t latest_payload_;
static uint32_t last_valid_frame_ms_ = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  /* Start interrupt-driven UART RX ring buffer (Wave 2 Task 8) */
  uart_ring_init(&huart1);

  /* Initialize SSD1306 OLED display (Wave 2 Task 10) */
  ssd1306_init(&hi2c1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t byte;
    payload_t parsed;
    uint8_t frame_ready = 0;

    /* Consume ring buffer and feed frame parser (Wave 2 Tasks 8/9) */
    while (uart_ring_read_byte(&byte))
    {
      parser_feed(byte, &parsed);
      if (parser_has_frame())
      {
        latest_payload_ = parsed;
        last_valid_frame_ms_ = HAL_GetTick();
        frame_ready = 1;
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  /* blink on frame */
      }
    }

    /* OLED display update (100ms interval, Wave 2 Task 10) */
    static uint32_t last_display_ms = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_display_ms >= 100)
    {
      last_display_ms = now;

      ssd1306_clear();

      char line[22];

      /* Row 1: frame counter */
      sprintf(line, "FC: %5u", latest_payload_.frame_counter);
      ssd1306_draw_string(0, 0, line);

      /* Row 2: target X, Y */
      if (latest_payload_.target_present)
      {
        sprintf(line, "X:%+5d Y:%+5d",
                latest_payload_.target_x, latest_payload_.target_y);
      }
      else
      {
        sprintf(line, "X: --- Y: ---");
      }
      ssd1306_draw_string(0, 1, line);

      /* Row 3: distance + tracker state */
      sprintf(line, "D:%5dmm S:%u",
              latest_payload_.distance, latest_payload_.tracker_state);
      ssd1306_draw_string(0, 2, line);

      /* Row 4: digit + confidence (= xor_checksum for now) */
      sprintf(line, "DGT:%u XOR:%02X",
              latest_payload_.digit, latest_payload_.xor_checksum);
      ssd1306_draw_string(0, 3, line);

      /* Row 5: errors + ring buffer available */
      sprintf(line, "ERR:%u RX:%u",
              parser_error_count(), uart_ring_available());
      ssd1306_draw_string(0, 4, line);

      /* LINK LOST check (250ms without valid frame) */
      if (now - last_valid_frame_ms_ > 250)
      {
        ssd1306_draw_string(0, 6, "  LINK LOST");
      }

      ssd1306_flush();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   /* 36 MHz */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;   /* 72 MHz */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */