#include "frame.hpp"

namespace frame {

std::array<uint8_t, FRAME_LEN> build(const Payload& p) {
    (void)p;
    std::array<uint8_t, FRAME_LEN> frame{};
    // TODO: implement in Wave 2 Task 6
    frame[0] = HEADER;
    frame[FRAME_LEN - 1] = FOOTER;
    return frame;
}

Parser::State Parser::feed(uint8_t byte) {
    (void)byte;
    // TODO: implement in Wave 2 Task 6
    return state_;
}

bool Parser::has_frame() const {
    return false;
}

Payload Parser::extract() {
    return Payload{};
}

size_t Parser::error_count() const {
    return 0;
}

} // namespace frame