#include <array>
#include <cstddef>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "DetectionTracker.hpp"
#include "camera_exposure.hpp"
#include "myCamera.hpp"
#include "MySerial/MySerial.hpp"
#include "yolov5.hpp"

namespace
{

namespace fs = std::filesystem;

struct DemoOptions
{
  std::string source = "0";
  std::string model_path = "models/yolov5/yolov5.xml";
  std::string device = "CPU";
  std::string port;
  int baud = 115200;
  std::size_t max_frames = 0U;
};

bool looks_like_integer(const std::string & value)
{
  if (value.empty()) return false;
  std::size_t index = 0;
  if (value[0] == '+' || value[0] == '-') {
    index = 1;
  }
  if (index >= value.size()) return false;
  return std::all_of(value.begin() + static_cast<std::ptrdiff_t>(index), value.end(), [](unsigned char ch) {
    return std::isdigit(ch) != 0;
  });
}

bool parse_size_t(const std::string & text, std::size_t & value)
{
  try {
    std::size_t consumed = 0;
    const unsigned long parsed = std::stoul(text, &consumed);
    if (consumed != text.size()) return false;
    value = static_cast<std::size_t>(parsed);
    return true;
  } catch (...) {
    return false;
  }
}

void print_usage()
{
  std::cout << "Usage: Serial_demo [--max-frames N] [--device DEVICE] [--port COMx] [--baud N] [source] [model_path]\n"
               "  --max-frames N: stop after processing N frames\n"
               "  --device DEVICE: OpenVINO device name (default CPU)\n"
               "  --port COMx: serial port for downlink coordinates\n"
               "  --baud N: serial baud rate (default 115200)\n"
               "  source: camera index or video file path (default 0)\n"
               "  model_path: YOLOv5 OpenVINO .xml path (default models/yolov5/yolov5.xml)\n";
}

DemoOptions parse_options(int argc, char ** argv)
{
  DemoOptions options;
  std::vector<std::string> positional;

  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "-h" || argument == "--help") {
      print_usage();
      std::exit(0);
    }
    if (argument == "--max-frames") {
      if (i + 1 >= argc) {
        std::cerr << "--max-frames requires a value\n";
        std::exit(1);
      }
      std::size_t max_frames = 0;
      if (!parse_size_t(argv[++i], max_frames)) {
        std::cerr << "--max-frames requires a non-negative integer\n";
        std::exit(1);
      }
      options.max_frames = max_frames;
      continue;
    }
    if (argument == "--device") {
      if (i + 1 >= argc) {
        std::cerr << "--device requires a value\n";
        std::exit(1);
      }
      options.device = argv[++i];
      continue;
    }
    if (argument == "--port") {
      if (i + 1 >= argc) {
        std::cerr << "--port requires a value\n";
        std::exit(1);
      }
      options.port = argv[++i];
      continue;
    }
    if (argument == "--baud") {
      if (i + 1 >= argc) {
        std::cerr << "--baud requires a value\n";
        std::exit(1);
      }
      options.baud = std::stoi(argv[++i]);
      continue;
    }
    positional.push_back(argument);
  }

  if (!positional.empty()) {
    options.source = positional[0];
  }
  if (positional.size() >= 2U) {
    options.model_path = positional[1];
  }

  return options;
}

std::string format_fps(double fps)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(1) << fps;
  return oss.str();
}

std::string source_label(const std::string & source)
{
  return looks_like_integer(source) ? ("camera " + source) : source;
}

void draw_detection(cv::Mat & frame, const rm_assessment::yolov5::Detection & detection)
{
  cv::rectangle(frame, detection.box, {0, 255, 0}, 2, cv::LINE_AA);
  std::ostringstream label;
  label << "color=" << detection.color_id << " " << std::fixed << std::setprecision(2) << detection.confidence;
  cv::putText(frame, label.str(), {cvRound(detection.box.x), std::max(18, cvRound(detection.box.y) - 6)},
    cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 1, cv::LINE_AA);
}

cv::Point2f detection_center(const rm_assessment::yolov5::Detection & detection)
{
  float sum_x = 0.0F;
  float sum_y = 0.0F;
  for (const auto & corner : detection.corners) {
    sum_x += corner.x;
    sum_y += corner.y;
  }
  return {sum_x / 4.0F, sum_y / 4.0F};
}

bool send_center_packet(MySerial & serial, const cv::Point2f & center)
{
  const int16_t x = static_cast<int16_t>(cvRound(center.x));
  const int16_t y = static_cast<int16_t>(cvRound(center.y));

  const std::array<uint8_t, 6> packet = {
    0xAA,
    static_cast<uint8_t>(x & 0xFF),
    static_cast<uint8_t>((x >> 8) & 0xFF),
    static_cast<uint8_t>(y & 0xFF),
    static_cast<uint8_t>((y >> 8) & 0xFF),
    0xBB,
  };
  return serial.send(packet.data(), packet.size());
}

}  // namespace

int main(int argc, char ** argv)
{
  const DemoOptions options = parse_options(argc, argv);

  if (!fs::exists(options.model_path)) {
    std::cerr << "Model not found: " << options.model_path << '\n';
    return 1;
  }

  rm_assessment::yolov5::YOLOV5Detector detector(options.model_path, options.device);
  rm_assessment::DetectionTracker tracker;

  io::myCamera camera(options.source);

  if (!camera.isOpened()) {
    std::cerr << "Failed to open source: " << options.source << '\n';
    return 1;
  }

  if (looks_like_integer(options.source)) {
    io::applySavedExposure(camera);
  }

  std::unique_ptr<MySerial> serial;
  if (!options.port.empty()) {
    serial = std::make_unique<MySerial>(options.port, options.baud);
    if (!serial->isOpen()) {
      std::cerr << "WARNING: failed to open serial port: " << options.port << '\n';
      serial.reset();
    }
  }

  std::cout << "Source: " << source_label(options.source) << '\n';
  std::cout << "Model: " << options.model_path << "\n";
  std::cout << "Device: " << options.device << '\n';
  std::cout << "Serial: " << (serial ? options.port : std::string("(disabled)")) << '\n';

  std::chrono::steady_clock::time_point last_tick = std::chrono::steady_clock::now();
  double fps = 0.0;
  std::size_t frame_index = 0U;
  std::size_t total_detections = 0U;
  std::size_t tracker_hits = 0U;

  const std::string window_name = "Serial_demo";
  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

  while (true) {
    if (options.max_frames > 0U && frame_index >= options.max_frames) {
      break;
    }

    cv::Mat frame;
    std::chrono::steady_clock::time_point timestamp;
    if (!camera.read(frame, timestamp) || frame.empty()) {
      break;
    }

    const auto detections = detector.detect(frame);
    total_detections += detections.size();

    const rm_assessment::yolov5::Detection * best_detection = nullptr;

    std::vector<cv::Rect2f> boxes;
    boxes.reserve(detections.size());
    for (const auto & detection : detections) {
      boxes.push_back(detection.box);
      draw_detection(frame, detection);
      if (best_detection == nullptr || detection.confidence > best_detection->confidence) {
        best_detection = &detection;
      }
    }

    const bool tracked = tracker.update(boxes, frame.size());
    if (tracked && tracker.has_target()) {
      ++tracker_hits;
    }

    if (tracker.has_target()) {
      cv::rectangle(frame, tracker.tracked_box(), {0, 255, 255}, 2, cv::LINE_AA);
      cv::putText(frame, "tracker " + tracker.state_str(), {12, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
        {0, 255, 255}, 2, cv::LINE_AA);
    }

    if (serial && best_detection != nullptr) {
      const cv::Point2f center = detection_center(*best_detection);
      (void)send_center_packet(*serial, center);
      cv::circle(frame, {cvRound(center.x), cvRound(center.y)}, 4, {0, 0, 255}, cv::FILLED, cv::LINE_AA);
    }

    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - last_tick).count();
    if (elapsed > 0.0) {
      fps = 1.0 / elapsed;
    }
    last_tick = now;

    std::ostringstream hud;
    hud << source_label(options.source) << " | model: loaded | fps: " << format_fps(fps)
        << " | frame: " << frame_index << " | detections: " << detections.size();
    cv::putText(frame, hud.str(), {12, 50}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 2, cv::LINE_AA);
    cv::imshow(window_name, frame);
    const int key = cv::waitKey(1);
    if (key == 27 || key == 'q') {
      break;
    }

    ++frame_index;
  }

  cv::destroyAllWindows();

  std::cout << "Frames processed: " << frame_index << '\n';
  std::cout << "Total detections: " << total_detections << '\n';
  std::cout << "Tracker hits: " << tracker_hits << '\n';
  return 0;
}
