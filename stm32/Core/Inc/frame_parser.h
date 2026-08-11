/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    frame_parser.h
  * @brief   Length-delimited frame parser API (mirrors communication/frame.hpp).
  *
  * Wave 2 Task 9: state machine WAIT_HEADER -> READING_PAYLOAD (12 bytes)
  * -> WAIT_FOOTER -> back to WAIT_HEADER. Fixed PAYLOAD_LEN=12, no search
  * functions, no dynamic allocation. XOR checksum verified at higher level.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FRAME_PARSER_H
#define __FRAME_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/
#define FRAME_HEADER  0xAA
#define FRAME_FOOTER  0xBB
#define PAYLOAD_LEN   12

/* Exported types ------------------------------------------------------------*/
typedef enum {
    WAIT_HEADER = 0,
    READING_PAYLOAD,
    WAIT_FOOTER,
    READY,
    ERR
} parser_state_t;

/* Field order MUST match frame::Payload in communication/frame.hpp exactly:
   frame_counter(u16), target_present(u8), target_x(i16), target_y(i16),
   distance(i16), tracker_state(u8), digit(u8), xor_checksum(u8) = 12 bytes.
   #pragma pack(1) mirrors frame.hpp's #pragma pack(push,1) mechanism. */
#pragma pack(push, 1)
typedef struct {
    uint16_t frame_counter;
    uint8_t  target_present;
    int16_t  target_x;
    int16_t  target_y;
    int16_t  distance;
    uint8_t  tracker_state;
    uint8_t  digit;
    uint8_t  xor_checksum;
} payload_t;
#pragma pack(pop)

/* Exported functions prototypes ---------------------------------------------*/
parser_state_t parser_feed(uint8_t byte, payload_t* out);
bool parser_has_frame(void);
void parser_reset(void);
uint16_t parser_error_count(void);

#ifdef __cplusplus
}
#endif

#endif /* __FRAME_PARSER_H */