// communication/tests/loopback_test.cpp
// PC-side end-to-end loopback test.
//
// Mode A (default): pure software loopback — frame::build() -> Parser
//                   round-trip over 1000 random payloads, no hardware.
// Mode B (--port):  hardware loopback over a real serial port (TX->RX
//                   shorted). Returns 1 with a message if the port fails.

#include "frame.hpp"
#include "mySerial.hpp"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <random>
#include <string>

namespace
{

bool matches(const frame::Payload & parsed, const frame::Payload & original)
{
  return parsed.frame_counter == original.frame_counter &&
         parsed.target_present == original.target_present &&
         parsed.target_x == original.target_x &&
         parsed.target_y == original.target_y &&
         parsed.distance == original.distance &&
         parsed.tracker_state == original.tracker_state &&
         parsed.digit == original.digit;
}

}  // namespace

int main(int argc, char * argv[])
{
  std::string port;
  int frames = 1000;
  int baud = 115200;

  // Parse args (simple, matching serial_loopback style)
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
      port = argv[++i];
    } else if (std::strcmp(argv[i], "--frames") == 0 && i + 1 < argc) {
      frames = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--baud") == 0 && i + 1 < argc) {
      baud = std::atoi(argv[++i]);
    }
  }

  std::mt19937 rng(42);  // fixed seed for reproducibility
  std::uniform_int_distribution<uint16_t> u16_dist(0, 65535);
  std::uniform_int_distribution<unsigned> u8_dist(0, 255);
  std::uniform_int_distribution<int16_t> i16_dist(-32768, 32767);

  // Build a random payload (xor_checksum is recalculated by frame::build)
  auto random_payload = [&]() -> frame::Payload {
    frame::Payload p{};
    p.frame_counter = u16_dist(rng);
    p.target_present = static_cast<uint8_t>(u8_dist(rng) & 1);
    p.target_x = i16_dist(rng);
    p.target_y = i16_dist(rng);
    p.distance = 0;
    p.tracker_state = static_cast<uint8_t>(u8_dist(rng));
    p.digit = static_cast<uint8_t>(u8_dist(rng) % 10);
    p.xor_checksum = 0;
    return p;
  };

  int matched = 0;
  int failed = 0;

  if (port.empty()) {
    // MODE A: Pure software loopback (no hardware needed)
    std::cout << "[loopback_test] Mode A: software loopback, "
              << frames << " frames\n";

    for (int n = 0; n < frames; ++n) {
      const frame::Payload original = random_payload();
      const auto frame_bytes = frame::build(original);

      // Parse back
      frame::Parser parser;
      for (const uint8_t byte : frame_bytes) {
        parser.feed(byte);
      }

      if (parser.has_frame()) {
        const frame::Payload parsed = parser.extract();
        // Compare (ignore xor_checksum - build recalculates it)
        if (matches(parsed, original)) {
          std::cout << "[" << (n + 1) << "/" << frames << "] MATCH=OK\n";
          ++matched;
        } else {
          std::cerr << "[" << (n + 1) << "/" << frames
                    << "] MATCH=FAIL (field mismatch)\n";
          ++failed;
        }
      } else {
        std::cerr << "[" << (n + 1) << "/" << frames
                  << "] MATCH=FAIL (no frame parsed)\n";
        ++failed;
      }
    }
  } else {
    // MODE B: Hardware loopback via mySerial (TX->RX shorted)
    std::cout << "[loopback_test] Mode B: hardware loopback on "
              << port << ", " << frames << " frames\n";

    mySerial serial(port, baud);

    // Probe the port: send() fails immediately when the port did not open.
    const uint8_t probe = 0x00;
    if (!serial.send(&probe, 1)) {
      std::cerr << "[loopback_test] Mode B: failed to open port '" << port
                << "' (is the device connected?)\n";
      return 1;
    }
    // Drain the echoed probe byte so it cannot corrupt the first frame.
    uint8_t drain = 0;
    (void)serial.receive(&drain, 1, 1000);

    for (int n = 0; n < frames; ++n) {
      const frame::Payload original = random_payload();
      const auto frame_bytes = frame::build(original);

      // Send over the wire; a loopback cable echoes the bytes back.
      if (!serial.send(frame_bytes.data(), frame_bytes.size())) {
        std::cerr << "[" << (n + 1) << "/" << frames
                  << "] MATCH=FAIL (send error)\n";
        ++failed;
        continue;
      }

      std::array<uint8_t, frame::FRAME_LEN> echo{};
      if (!serial.receive(echo.data(), echo.size(), 1000)) {
        std::cerr << "[" << (n + 1) << "/" << frames
                  << "] MATCH=FAIL (receive timeout)\n";
        ++failed;
        continue;
      }

      // Parse back
      frame::Parser parser;
      for (const uint8_t byte : echo) {
        parser.feed(byte);
      }

      if (parser.has_frame()) {
        const frame::Payload parsed = parser.extract();
        if (matches(parsed, original)) {
          std::cout << "[" << (n + 1) << "/" << frames << "] MATCH=OK\n";
          ++matched;
        } else {
          std::cerr << "[" << (n + 1) << "/" << frames
                    << "] MATCH=FAIL (field mismatch)\n";
          ++failed;
        }
      } else {
        std::cerr << "[" << (n + 1) << "/" << frames
                  << "] MATCH=FAIL (no frame parsed)\n";
        ++failed;
      }
    }
  }

  std::cout << "\n[loopback_test] Results: " << matched << "/" << frames
            << " OK, " << failed << " FAILED\n";

  return (failed == 0) ? 0 : 1;
}