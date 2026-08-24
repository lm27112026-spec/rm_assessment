#include "frame.hpp"

#include <cstring>

namespace frame {

std::array<uint8_t, FRAME_LEN> build(const Payload& p) {
    std::array<uint8_t, FRAME_LEN> frame{};
    frame[0] = HEADER;

    const uint8_t* src = reinterpret_cast<const uint8_t*>(&p);
    for (size_t i = 0; i < PAYLOAD_LEN - 1; ++i) {
        frame[1 + i] = src[i];
    }

    uint8_t xor_val = 0;
    for (size_t i = 0; i < PAYLOAD_LEN - 1; ++i) {
        xor_val ^= frame[1 + i];
    }
    frame[1 + PAYLOAD_LEN - 1] = xor_val;

    frame[FRAME_LEN - 1] = FOOTER;
    return frame;
}

Parser::State Parser::feed(uint8_t byte) {
    if (state_ == State::READY) {
        state_ = State::WAIT_HEADER;
    }

    switch (state_) {
    case State::WAIT_HEADER:
        if (byte == HEADER) {
            idx_ = 0;
            state_ = State::READING_PAYLOAD;
        } else {
            ++errors_;
        }
        break;

    case State::READING_PAYLOAD:
        buf_[idx_++] = byte;
        if (idx_ == PAYLOAD_LEN) {
            state_ = State::WAIT_FOOTER;
        }
        break;

    case State::WAIT_FOOTER:
        if (byte == FOOTER) {
            std::memcpy(&latest_, buf_.data(), PAYLOAD_LEN);
            if (queue_count_ < queue_.size()) {
                const size_t queue_tail = (queue_head_ + queue_count_) % queue_.size();
                queue_[queue_tail] = latest_;
                ++queue_count_;
                ready_ = true;
            } else {
                ++errors_;
            }
            idx_ = 0;
            state_ = State::READY;
        } else {
            ++errors_;
            idx_ = 0;
            state_ = State::WAIT_HEADER;
        }
        break;

    case State::READY:
        break;

    case State::ERROR:
        state_ = State::WAIT_HEADER;
        break;
    }

    return state_;
}

bool Parser::has_frame() const {
    return queue_count_ > 0;
}

Payload Parser::extract() {
    Payload payload{};
    if (queue_count_ > 0) {
        payload = queue_[queue_head_];
        queue_head_ = (queue_head_ + 1) % queue_.size();
        --queue_count_;
    }
    ready_ = queue_count_ > 0;
    if (state_ == State::READY) {
        state_ = ready_ ? State::READY : State::WAIT_HEADER;
    }
    return payload;
}

size_t Parser::error_count() const {
    return errors_;
}

} // namespace frame
