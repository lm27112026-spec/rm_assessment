#include <array>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

#include "frame.hpp"

namespace
{

class TestFailure : public std::runtime_error
{
public:
  explicit TestFailure(const std::string & message) : std::runtime_error(message) {}
};

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw TestFailure(message);
  }
}

bool same_payload(const frame::Payload & lhs, const frame::Payload & rhs)
{
  return std::memcmp(&lhs, &rhs, sizeof(frame::Payload)) == 0;
}

uint8_t payload_xor(const frame::Payload & payload)
{
  const auto * bytes = reinterpret_cast<const uint8_t *>(&payload);
  uint8_t checksum = 0;
  for (std::size_t i = 0; i < frame::PAYLOAD_LEN - 1; ++i) {
    checksum ^= bytes[i];
  }
  return checksum;
}

frame::Payload with_checksum(frame::Payload payload)
{
  payload.xor_checksum = payload_xor(payload);
  return payload;
}

frame::Payload feed_and_extract(const std::array<uint8_t, frame::FRAME_LEN> & bytes)
{
  frame::Parser parser;
  for (const uint8_t byte : bytes) {
    parser.feed(byte);
  }
  require(parser.has_frame(), "Parser did not report a complete frame");
  return parser.extract();
}

void test_build_produces_correct_frame_shape()
{
  const frame::Payload payload = with_checksum({1, 1, 200, 100, 0, 2, 5, 0});
  const auto bytes = frame::build(payload);

  require(bytes.size() == frame::FRAME_LEN, "Frame length is not 14 bytes");
  require(bytes[0] == frame::HEADER, "Frame header is not 0xAA");
  require(bytes[13] == frame::FOOTER, "Frame footer is not 0xBB");
}

void test_build_feed_extract_round_trip()
{
  const frame::Payload payload = with_checksum({42, 1, -123, 456, 789, 3, 9, 0});
  const frame::Payload parsed = feed_and_extract(frame::build(payload));

  require(same_payload(parsed, payload), "Round-trip payload does not match original");
}

void test_xor_checksum_is_correct()
{
  frame::Payload payload{0x1234, 1, 0x0102, -300, 4000, 7, 0x55, 0};
  const auto bytes = frame::build(payload);

  uint8_t checksum = 0;
  for (std::size_t i = 1; i <= 11; ++i) {
    checksum ^= bytes[i];
  }
  require(bytes[12] == checksum, "Frame checksum byte is not XOR of payload bytes 0..10");
}

void test_parser_resets_on_bad_header()
{
  frame::Parser parser;
  const frame::Parser::State state = parser.feed(0x00);

  require(state == frame::Parser::State::WAIT_HEADER, "Bad header did not keep parser waiting");
  require(parser.error_count() == 1, "Bad header did not increment error count");
  require(!parser.has_frame(), "Bad header unexpectedly produced a frame");
}

void test_parser_resets_on_bad_footer()
{
  const frame::Payload payload = with_checksum({5, 1, 10, 20, 30, 4, 2, 0});
  const auto bytes = frame::build(payload);
  frame::Parser parser;

  for (std::size_t i = 0; i < frame::FRAME_LEN - 1; ++i) {
    parser.feed(bytes[i]);
  }
  const frame::Parser::State state = parser.feed(0x00);

  require(state == frame::Parser::State::WAIT_HEADER, "Bad footer did not reset parser");
  require(parser.error_count() == 1, "Bad footer did not increment error count");
  require(!parser.has_frame(), "Bad footer unexpectedly produced a frame");
}

void test_truncated_frame_has_no_frame()
{
  const frame::Payload payload = with_checksum({6, 1, 10, 20, 30, 4, 2, 0});
  const auto bytes = frame::build(payload);
  frame::Parser parser;

  for (std::size_t i = 0; i < frame::FRAME_LEN - 1; ++i) {
    parser.feed(bytes[i]);
  }

  require(!parser.has_frame(), "Truncated frame incorrectly reported complete frame");
}

void test_two_consecutive_frames()
{
  const frame::Payload first = with_checksum({7, 1, 11, 22, 33, 1, 4, 0});
  const frame::Payload second = with_checksum({8, 0, -11, -22, -33, 2, 255, 0});
  const auto first_bytes = frame::build(first);
  const auto second_bytes = frame::build(second);
  frame::Parser parser;

  for (const uint8_t byte : first_bytes) {
    parser.feed(byte);
  }
  for (const uint8_t byte : second_bytes) {
    parser.feed(byte);
  }

  require(parser.has_frame(), "First consecutive frame was not ready");
  require(same_payload(parser.extract(), first), "First consecutive frame payload mismatch");
  require(parser.has_frame(), "Second consecutive frame was not ready");
  require(same_payload(parser.extract(), second), "Second consecutive frame payload mismatch");
}

void test_payload_can_contain_footer_byte()
{
  const frame::Payload payload = with_checksum({0x00BB, 0xBB, 0x00BB, 0x01BB, 0x02BB, 0xBB, 0xBB, 0});
  const frame::Payload parsed = feed_and_extract(frame::build(payload));

  require(same_payload(parsed, payload), "Payload containing 0xBB did not parse by fixed length");
}

void test_zero_payload_round_trip()
{
  const frame::Payload payload = with_checksum({0, 0, 0, 0, 0, 0, 0, 0});
  const frame::Payload parsed = feed_and_extract(frame::build(payload));

  require(same_payload(parsed, payload), "Zero payload round-trip mismatch");
}

void test_max_values_round_trip()
{
  const frame::Payload payload = with_checksum({65535, 255, 32767, 32767, 32767, 255, 255, 0});
  const frame::Payload parsed = feed_and_extract(frame::build(payload));

  require(same_payload(parsed, payload), "Max values round-trip mismatch");
}

void run_test(const std::string & name, void (*test)())
{
  try {
    test();
    std::cout << "[PASS] " << name << '\n';
  } catch (const std::exception & error) {
    std::cerr << "[FAIL] " << name << ": " << error.what() << '\n';
    throw;
  }
}

}  // namespace

int main()
{
  try {
    run_test("build produces correct frame shape", test_build_produces_correct_frame_shape);
    run_test("build feed extract round trip", test_build_feed_extract_round_trip);
    run_test("xor checksum is correct", test_xor_checksum_is_correct);
    run_test("parser resets on bad header", test_parser_resets_on_bad_header);
    run_test("parser resets on bad footer", test_parser_resets_on_bad_footer);
    run_test("truncated frame has no frame", test_truncated_frame_has_no_frame);
    run_test("two consecutive frames", test_two_consecutive_frames);
    run_test("payload can contain footer byte", test_payload_can_contain_footer_byte);
    run_test("zero payload round trip", test_zero_payload_round_trip);
    run_test("max values round trip", test_max_values_round_trip);
  } catch (const std::exception &) {
    return 1;
  }
  return 0;
}
