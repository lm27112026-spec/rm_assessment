// Wave 3 Task 13: vision-to-serial integration demo.
// Reads frames via io::myCamera, runs the auto_aim::Detector pipeline,
// converts the best armor into a frame::Payload, and sends it over serial at 20 Hz.
#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <list>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include "armor.hpp"
#include "detector.hpp"
#include "frame.hpp"
#include "img_tools.hpp"
#include "camera_exposure.hpp"
#include "myCamera.hpp"
#include "mySerial.hpp"

namespace
{

struct Config
{
  std::string port;
  int baud = 115200;
  std::string video_source = "0";
  std::string model_path = "learning/assets/tiny_resnet.onnx";
  int max_frames = 0;  // 0 = infinite
  std::string evidence_path;
  bool no_send = false;
};

void print_usage(const char * prog)
{
  std::cout << "Usage: " << prog << " [OPTIONS]\n"
            << "  --port COMx       Serial port (required for serial output)\n"
            << "  --baud N          Baud rate (default: 115200)\n"
            << "  --video PATH      Video file (default: camera 0)\n"
            << "  --count N         Max frames (default: 0 = infinite)\n"
            << "  --evidence PATH   Log output path (default: stdout only)\n"
            << "  --model PATH      Digit classifier ONNX path\n"
            << "  --no-send         Vision only, no serial output (dry run)\n";
}

Config parse_args(int argc, char ** argv)
{
  Config cfg;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--port" && i + 1 < argc) {
      cfg.port = argv[++i];
    } else if (arg == "--baud" && i + 1 < argc) {
      cfg.baud = std::atoi(argv[++i]);
    } else if (arg == "--video" && i + 1 < argc) {
      cfg.video_source = argv[++i];
    } else if (arg == "--count" && i + 1 < argc) {
      cfg.max_frames = std::atoi(argv[++i]);
    } else if (arg == "--evidence" && i + 1 < argc) {
      cfg.evidence_path = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      cfg.model_path = argv[++i];
    } else if (arg == "--no-send") {
      cfg.no_send = true;
    } else if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      std::exit(0);
    } else {
      std::cerr << "Unknown option: " << arg << "\n";
      print_usage(argv[0]);
      std::exit(1);
    }
  }
  return cfg;
}

// Map ArmorName to the digit byte carried by frame::Payload.
// 1..5 for the numbered armors, 255 (unknown) for sentry/outpost/base/not_armor.
uint8_t digit_from_armor_name(auto_aim::ArmorName name)
{
  switch (name) {
    case auto_aim::one: return 1;
    case auto_aim::two: return 2;
    case auto_aim::three: return 3;
    case auto_aim::four: return 4;
    case auto_aim::five: return 5;
    default: return 255;
  }
}

// Pick the highest-confidence armor as the serial target.
const auto_aim::Armor * best_armor(const std::list<auto_aim::Armor> & armors)
{
  const auto_aim::Armor * best = nullptr;
  for (const auto & armor : armors) {
    if (best == nullptr || armor.confidence > best->confidence) {
      best = &armor;
    }
  }
  return best;
}

}  // namespace

int main(int argc, char ** argv)
{
  Config cfg = parse_args(argc, argv);

  // Evidence log (stdout only when no path is given)
  std::ofstream log;
  if (!cfg.evidence_path.empty()) {
    log.open(cfg.evidence_path);
  }

  auto log_msg = [&](const std::string & msg) {
    std::cout << msg << "\n";
    if (log.is_open()) {
      log << msg << "\n";
    }
  };

  // Serial output (skipped in dry-run mode or without a port)
  std::unique_ptr<mySerial> serial;
  if (!cfg.port.empty() && !cfg.no_send) {
    serial = std::make_unique<mySerial>(cfg.port, cfg.baud);
  } else if (cfg.port.empty() && !cfg.no_send) {
    log_msg("[serial_demo] WARNING: no --port given; running without serial output");
  }

  // Vision pipeline
  auto_aim::Detector detector(cfg.model_path);

  io::myCamera cam(cfg.video_source);
  if (!cfg.video_source.empty() && std::all_of(cfg.video_source.begin(), cfg.video_source.end(), [](unsigned char ch) {
        return std::isdigit(ch) != 0;
      })) {
    io::applySavedExposure(cam);
  }

  cv::Mat frame;
  std::chrono::steady_clock::time_point timestamp;
  int frame_count = 0;
  uint16_t serial_frame_counter = 0;

  const auto frame_interval = std::chrono::milliseconds(50);  // 20 Hz

  log_msg("[serial_demo] Starting...");
  log_msg("[serial_demo] source=" + cfg.video_source +
          " model=" + (detector.has_model() ? "loaded" : "missing") +
          " serial=" + (serial ? cfg.port : "(none)") +
          " no_send=" + (cfg.no_send ? "yes" : "no"));

  while (cam.read(frame, timestamp)) {
    if (frame.empty()) {
      continue;
    }

    // Detection + classification (gated by confidence > 0.8 and name != not_armor)
    const std::list<auto_aim::Armor> armors = detector.detect(frame);
    const auto_aim::Armor * target = best_armor(armors);

    // Build payload from the best armor
    frame::Payload payload{};
    payload.frame_counter = serial_frame_counter;
    payload.tracker_state = 0;
    payload.distance = 0;  // EX-04 PnP not implemented

    if (target != nullptr) {
      payload.target_present = 1;
      payload.target_x = static_cast<int16_t>(cvRound(target->center.x));
      payload.target_y = static_cast<int16_t>(cvRound(target->center.y));
      payload.digit = digit_from_armor_name(target->name);
      payload.tracker_state = 2;  // "tracking" (this pipeline has no Kalman tracker)
    } else {
      payload.target_present = 0;
      payload.target_x = 0;
      payload.target_y = 0;
      payload.digit = 255;  // unknown
    }
    payload.xor_checksum = 0;  // build() computes it

    // Build and send frame
    const std::array<uint8_t, frame::FRAME_LEN> frame_bytes = frame::build(payload);
    if (serial) {
      if (serial->send(frame_bytes.data(), frame_bytes.size())) {
        log_msg("[serial_demo] Frame " + std::to_string(serial_frame_counter) + " sent");
      } else {
        log_msg("[serial_demo] Frame " + std::to_string(serial_frame_counter) + " send FAILED");
      }
    }

    ++serial_frame_counter;
    ++frame_count;

    // Display
    for (const auto & armor : armors) {
      tools::draw_points(frame, armor.points, {0, 255, 0}, 2);
    }
    if (target != nullptr) {
      tools::draw_points(frame, target->points, {0, 255, 255}, 2);
      cv::circle(
        frame, cv::Point(cvRound(target->center.x), cvRound(target->center.y)), 4, {0, 255, 255},
        cv::FILLED, cv::LINE_AA);
    }
    cv::putText(
      frame, "serial_demo | FC:" + std::to_string(serial_frame_counter), cv::Point(10, 30),
      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::imshow("serial_demo", frame);

    const int key = cv::waitKey(1);
    if (key == 'q' || key == 'Q' || key == 27) {
      break;  // q or ESC
    }

    // 20 Hz timing
    std::this_thread::sleep_for(frame_interval);

    if (cfg.max_frames > 0 && frame_count >= cfg.max_frames) {
      break;
    }
  }

  log_msg("[serial_demo] Done. " + std::to_string(frame_count) + " frames processed.");
  cv::destroyAllWindows();

  return 0;
}
