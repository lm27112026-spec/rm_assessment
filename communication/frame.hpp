#pragma once

#include <cstdint>
#include <array>

namespace frame {

constexpr uint8_t HEADER = 0xAA;
constexpr uint8_t FOOTER = 0xBB;
constexpr size_t PAYLOAD_LEN = 12;
constexpr size_t FRAME_LEN = 14;

#pragma pack(push, 1)
struct Payload {
    uint16_t frame_counter;
    uint8_t  target_present;
    int16_t  target_x;
    int16_t  target_y;
    int16_t  distance;
    uint8_t  tracker_state;
    uint8_t  digit;
    uint8_t  xor_checksum;
};
#pragma pack(pop)

std::array<uint8_t, FRAME_LEN> build(const Payload& p);

class Parser {
public:
    enum class State { WAIT_HEADER, READING_PAYLOAD, WAIT_FOOTER, READY, ERROR };

    State feed(uint8_t byte);
    bool has_frame() const;
    Payload extract();
    size_t error_count() const;

private:
    State state_ = State::WAIT_HEADER;
    std::array<uint8_t, PAYLOAD_LEN> buf_{};
    size_t idx_ = 0;
    size_t errors_ = 0;
    bool ready_ = false;
    Payload latest_{};
};

} // namespace frame