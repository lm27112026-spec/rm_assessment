#include <iostream>
#include <string>
#include <cstring>
#include <cstdlib>

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

int main(int argc, char* argv[]) {
    Config cfg = parse_args(argc, argv);

    std::cout << "[serial_loopback] Configuration:\n"
              << "  frames: " << cfg.frames << "\n"
              << "  evidence: " << (cfg.evidence_path.empty() ? "(none)" : cfg.evidence_path) << "\n"
              << "  port: " << (cfg.port.empty() ? "(none - simulation mode)" : cfg.port) << "\n"
              << "  inject_bad_header: " << (cfg.inject_bad_header ? "yes" : "no") << "\n"
              << "  inject_bad_tail: " << (cfg.inject_bad_tail ? "yes" : "no") << "\n"
              << "  inject_truncated: " << (cfg.inject_truncated ? "yes" : "no") << "\n"
              << "  inject_bb_payload: " << (cfg.inject_bb_payload ? "yes" : "no") << "\n";

    // TODO Wave 2 Task 12: implement actual loopback logic
    std::cout << "[serial_loopback] Stub: real logic in Wave 2 Task 12\n";

    return 0;
}