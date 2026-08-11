/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    frame_parser.c
  * @brief   Length-delimited frame parser state machine.
  *
  * Wave 2 Task 9: mirrors frame::Parser in communication/frame.cpp.
  * - WAIT_HEADER -> READING_PAYLOAD (exactly PAYLOAD_LEN=12 bytes)
  *   -> WAIT_FOOTER -> back to WAIT_HEADER
  * - Length-based delimiting: read 12 bytes after 0xAA, then expect 0xBB.
  *   No strchr/memchr/memmem, no dynamic allocation, no OLED/I2C here.
  * - A completed frame stays pending (parser_has_frame() == true) until the
  *   next byte is fed, mirroring Parser::extract() consuming ready_.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "frame_parser.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static parser_state_t state_ = WAIT_HEADER;
static uint8_t buf_[PAYLOAD_LEN];
static uint8_t idx_ = 0;
static uint16_t errors_ = 0;
static bool ready_ = false;
static payload_t latest_;

/* Private user code ---------------------------------------------------------*/
/**
  * @brief  Feed one byte into the parser state machine.
  * @param  byte: next byte from the UART ring buffer
  * @param  out:  if non-NULL and a frame completed on this call, receives a
  *               copy of the parsed payload
  * @retval current parser state
  */
parser_state_t parser_feed(uint8_t byte, payload_t* out) {
    /* A pending frame is consumed by the next byte fed (like Parser::extract) */
    if (ready_) {
        ready_ = false;
    }

    switch (state_) {
    case WAIT_HEADER:
        if (byte == FRAME_HEADER) {
            state_ = READING_PAYLOAD;
            idx_ = 0;
        } else {
            errors_++;
            state_ = WAIT_HEADER;
        }
        break;

    case READING_PAYLOAD:
        buf_[idx_++] = byte;
        if (idx_ >= PAYLOAD_LEN) {
            state_ = WAIT_FOOTER;
        }
        break;

    case WAIT_FOOTER:
        if (byte == FRAME_FOOTER) {
            /* Copy buffer to payload */
            memcpy(&latest_, buf_, PAYLOAD_LEN);
            ready_ = true;
            state_ = WAIT_HEADER;
        } else {
            errors_++;
            state_ = WAIT_HEADER;
        }
        break;

    default:
        state_ = WAIT_HEADER;
        break;
    }

    if (out != NULL && ready_) {
        memcpy(out, &latest_, sizeof(payload_t));
    }

    return state_;
}

/**
  * @brief  Check whether a complete frame is pending extraction.
  * @retval true if a frame completed and has not been consumed yet
  */
bool parser_has_frame(void) {
    return ready_;
}

/**
  * @brief  Reset parser to initial state, discarding any partial frame.
  */
void parser_reset(void) {
    state_ = WAIT_HEADER;
    idx_ = 0;
    ready_ = false;
    memset(buf_, 0, PAYLOAD_LEN);
    memset(&latest_, 0, sizeof(payload_t));
}

/**
  * @brief  Total number of framing errors (bad header/footer bytes).
  *         Lifetime counter, not cleared by parser_reset().
  * @retval error counter
  */
uint16_t parser_error_count(void) {
    return errors_;
}