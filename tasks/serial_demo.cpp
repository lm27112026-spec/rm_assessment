// Wave 3 Task 13: vision-to-serial integration demo.
// Reads frames via io::myCamera, runs the armor_vision pipeline
// (ArmorDetector + DigitRecognizer + Tracker), converts the tracker
// output into a frame::Payload, and sends it over serial at 20 Hz.
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include "communication/frame.hpp"
#include "communication/mySerial.hpp"
#include "io/myCamera.hpp"
#include "src/armor.hpp"
#include "src/detector.hpp"
#include "src/digit_recognizer.hpp"
#include "src/tracker.hpp"

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
  bool show_display = true;
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
            << "  --no-send         Vision only, no serial output (dry run)\n"
            << "  --no-display      Headless mode\n";
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
    } else if (arg == "--no-display") {
      cfg.show_display = false;
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

// Pick the recognized digit of the candidate closest to the tracked center.
// Returns 255 (unknown) when no candidate is near the tracked position.
int digit_for_tracked_target(
  const std::vector<rm_assessment::ArmorCandidate> & candidates,
  const std::vector<int> & digits,
  const cv::Point2f & tracked_center)
{
  int best_digit = 255;
  float best_distance = std::numeric_limits<float>::max();
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const float dx = candidates[i].center.x - tracked_center.x;
    const float dy = candidates[i].center.y - tracked_center.y;
    const float distance = dx * dx + dy * dy;
    if (distance < best_distance) {
      best_distance = distance;
      best_digit = digits[i];
    }
  }
  return best_digit;
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
  rm_assessment::ArmorDetector detector;
  rm_assessment::DigitRecognizer recognizer(cfg.model_path);
  rm_assessment::Tracker tracker;

  io::myCamera cam(cfg.video_source);

  cv::Mat frame;
  std::chrono::steady_clock::time_point timestamp;
  int frame_count = 0;
  uint16_t serial_frame_counter = 0;

  const auto frame_interval = std::chrono::milliseconds(50);  // 20 Hz

  log_msg("[serial_demo] Starting...");
  log_msg("[serial_demo] source=" + cfg.video_source +
          " model=" + (recognizer.has_model() ? "loaded" : "missing") +
          " serial=" + (serial ? cfg.port : "(none)") +
          " no_send=" + (cfg.no_send ? "yes" : "no"));

  while (cam.read(frame, timestamp)) {
    if (frame.empty()) {
      continue;
    }

    // Detection + digit recognition
    std::vector<rm_assessment::ArmorCandidate> candidates = detector.detect(frame);
    std::vector<int> digits(candidates.size(), 255);
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      const rm_assessment::RecognitionResult result = recognizer.recognize(candidates[i]);
      if (result.reliable && result.digit >= 0 && result.digit <= 9) {
        digits[i] = result.digit;
      }
    }

    // Tracking
    tracker.update(candidates, frame.size());

    // Build payload from tracker output
    frame::Payload payload{};
    payload.frame_counter = serial_frame_counter;
    payload.tracker_state = static_cast<uint8_t>(tracker.state_value());
    payload.distance = 0;  // EX-04 PnP not implemented

    if (tracker.has_target()) {
      const cv::Point2f center = tracker.tracked_center();
      payload.target_present = 1;
      payload.target_x = static_cast<int16_t>(cvRound(center.x));
      payload.target_y = static_cast<int16_t>(cvRound(center.y));
      payload.digit = static_cast<uint8_t>(digit_for_tracked_target(candidates, digits, center));
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
    if (cfg.show_display) {
      if (tracker.has_target()) {
        const cv::Rect2f box = tracker.tracked_box();
        cv::rectangle(frame, box, {0, 255, 255}, 2, cv::LINE_AA);
        cv::circle(frame, tracker.tracked_center(), 4, {0, 255, 255}, cv::FILLED, cv::LINE_AA);
      }
      cv::putText(frame, "serial_demo | tracker: " + tracker.state() + " | FC:" + std::to_string(serial_frame_counter),
                  cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
      cv::imshow("serial_demo", frame);

      const int key = cv::waitKey(1);
      if (key == 'q' || key == 'Q' || key == 27) {
        break;  // q or ESC
      }
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