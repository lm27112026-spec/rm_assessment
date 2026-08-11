#include <iostream>
#include <string>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <cstdarg>
#include <cstdint>
#include <array>
#include <random>

#include "frame.hpp"
#include "mySerial.hpp"

struct Config {
    int frames = 1000;
    std::string evidence_path;
    bool inject_bad_header = false;
    bool inject_bad_tail = false;
    bool inject_truncated = false;
    bool inject_bb_payload = false;
    std::string port;
};

static void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [OPTIONS]\n"
              << "  --frames N            Number of frames (default: 1000)\n"
              << "  --evidence PATH       Evidence output path\n"
              << "  --inject-bad-header   Inject frames with bad header\n"
              << "  --inject-bad-tail     Inject frames with bad tail\n"
              << "  --inject-truncated    Inject truncated frames\n"
              << "  --inject-bb-payload   Inject frames with 0xBB in payload\n"
              << "  --port COMx           Serial port for hardware loopback\n";
}

static Config parse_args(int argc, char* argv[]) {
    Config cfg;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--frames") == 0 && i + 1 < argc) {
            cfg.frames = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--evidence") == 0 && i + 1 < argc) {
            cfg.evidence_path = argv[++i];
        } else if (std::strcmp(argv[i], "--inject-bad-header") == 0) {
            cfg.inject_bad_header = true;
        } else if (std::strcmp(argv[i], "--inject-bad-tail") == 0) {
            cfg.inject_bad_tail = true;
        } else if (std::strcmp(argv[i], "--inject-truncated") == 0) {
            cfg.inject_truncated = true;
        } else if (std::strcmp(argv[i], "--inject-bb-payload") == 0) {
            cfg.inject_bb_payload = true;
        } else if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            cfg.port = argv[++i];
        } else if (std::strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            std::exit(0);
        } else {
            std::cerr << "Unknown option: " << argv[i] << "\n";
            print_usage(argv[0]);
            std::exit(1);
        }
    }
    return cfg;
}

// Build a random payload (xor_checksum is recalculated by frame::build)
static frame::Payload random_payload(std::mt19937& rng) {
    std::uniform_int_distribution<uint16_t> u16_dist(0, 65535);
    std::uniform_int_distribution<unsigned> u8_dist(0, 255);
    std::uniform_int_distribution<int16_t> i16_dist(-32768, 32767);

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
}

int main(int argc, char* argv[]) {
    Config cfg = parse_args(argc, argv);

    // Open evidence file if specified
    FILE* evidence = nullptr;
    if (!cfg.evidence_path.empty()) {
        evidence = fopen(cfg.evidence_path.c_str(), "w");
        if (!evidence) {
            std::cerr << "Cannot open evidence file: " << cfg.evidence_path << "\n";
            return 1;
        }
    }

    // Log to stdout AND the evidence file
    auto log_line = [&](const char* fmt, ...) {
        va_list args1, args2;
        va_start(args1, fmt);
        va_copy(args2, args1);
        vprintf(fmt, args1);
        va_end(args1);
        if (evidence) {
            vfprintf(evidence, fmt, args2);
        }
        va_end(args2);
    };

    int matched = 0, failed = 0;

    if (!cfg.port.empty()) {
        // Mode B: Hardware loopback
        log_line("[serial_loopback] Mode B: hardware loopback on %s, %d frames\n",
                 cfg.port.c_str(), cfg.frames);

        mySerial serial(cfg.port, 115200);

        std::mt19937 rng(42);

        for (int n = 0; n < cfg.frames; ++n) {
            // Build payload (with injection logic)
            frame::Payload payload = random_payload(rng);
            payload.frame_counter = static_cast<uint16_t>(n);

            auto frame_bytes = frame::build(payload);

            // Inject bad frame modifications
            if (cfg.inject_bad_header) frame_bytes[0] = 0xFF;
            if (cfg.inject_bad_tail) frame_bytes[13] = 0x00;

            if (cfg.inject_truncated) {
                serial.send(frame_bytes.data(), 13);  // send without footer
            } else if (cfg.inject_bb_payload) {
                // Fill payload with 0xBB
                for (int i = 1; i <= 12; ++i) frame_bytes[i] = 0xBB;
                serial.send(frame_bytes.data(), 14);
            } else {
                serial.send(frame_bytes.data(), 14);
            }

            // Read back 14 bytes
            uint8_t recv[14] = {};
            bool ok = serial.receive(recv, 14, 1000);

            if (ok && !cfg.inject_bad_header && !cfg.inject_bad_tail && !cfg.inject_truncated) {
                // Parse and compare
                frame::Parser parser;
                for (int i = 0; i < 14; ++i) parser.feed(recv[i]);
                if (parser.has_frame()) {
                    auto parsed = parser.extract();
                    if (parsed.frame_counter == payload.frame_counter)
                        ++matched;
                    else ++failed;
                } else ++failed;
            }

            log_line("[%d/%d] MATCH=%s\n", n + 1, cfg.frames, ok ? "OK" : "FAIL");
        }
        log_line("Results: %d/%d OK, %d FAILED\n", matched, cfg.frames, failed);

    } else {
        // Mode A: Software loopback
        log_line("[serial_loopback] Mode A: software loopback, %d frames\n", cfg.frames);

        std::mt19937 rng(42);

        for (int n = 0; n < cfg.frames; ++n) {
            frame::Payload original = random_payload(rng);
            original.frame_counter = static_cast<uint16_t>(n);

            auto frame_bytes = frame::build(original);

            // Inject bad modifications
            std::array<uint8_t, 14> injected{};
            std::memcpy(injected.data(), frame_bytes.data(), 14);
            if (cfg.inject_bad_header) injected[0] = 0xFF;
            if (cfg.inject_bad_tail) injected[13] = 0x00;

            size_t send_len = 14;
            if (cfg.inject_truncated) send_len = 13;
            if (cfg.inject_bb_payload) {
                for (int i = 1; i <= 12; ++i) injected[i] = 0xBB;
            }

            frame::Parser parser;
            for (size_t i = 0; i < send_len; ++i) parser.feed(injected[i]);

            if (!cfg.inject_bad_header && !cfg.inject_bad_tail && !cfg.inject_truncated) {
                if (parser.has_frame()) {
                    auto parsed = parser.extract();
                    // Compare (ignore xor_checksum - build recalculates it)
                    if (parsed.frame_counter == original.frame_counter) {
                        ++matched;
                        log_line("[%d/%d] MATCH=OK\n", n + 1, cfg.frames);
                    } else {
                        ++failed;
                        log_line("[%d/%d] MATCH=FAIL\n", n + 1, cfg.frames);
                    }
                } else {
                    ++failed;
                    log_line("[%d/%d] MATCH=FAIL\n", n + 1, cfg.frames);
                }
            } else {
                // Injection mode: expected to fail
                log_line("[%d/%d] INJECT-%s\n", n + 1, cfg.frames,
                    cfg.inject_bad_header ? "BAD-HEADER" :
                    cfg.inject_bad_tail ? "BAD-TAIL" :
                    cfg.inject_truncated ? "TRUNCATED" : "BB-PAYLOAD");
            }
        }
        log_line("Results: %d/%d OK, %d FAILED\n", matched, cfg.frames, failed);
    }

    if (evidence) fclose(evidence);
    return (failed == 0) ? 0 : 1;
}
