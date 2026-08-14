//题目2
#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <list>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "armor.hpp"
#include "detector.hpp"
#include "img_tools.hpp"

namespace
{

namespace fs = std::filesystem;

struct DemoOptions
{
  std::string source;
  std::string model_path;
  std::size_t max_frames = 0U;
  int width = 0;   // 0 = keep camera native resolution
  int height = 0;  // 0 = keep camera native resolution
  int fps = 0;     // 0 = keep camera native fps
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

bool parse_int(const std::string & text, int & value)
{
  try {
    std::size_t consumed = 0;
    const int parsed = std::stoi(text, &consumed);
    if (consumed != text.size()) return false;
    value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

std::string join_path(const fs::path & base, const fs::path & child)
{
  if (child.is_absolute()) {
    return child.string();
  }
  return (base / child).lexically_normal().string();
}

DemoOptions parse_options(int argc, char ** argv)
{
  DemoOptions options;
  const fs::path repo_root = fs::current_path();
  options.source = "0";
  options.model_path = join_path(repo_root, fs::path("learning/assets/tiny_resnet.onnx"));

  std::vector<std::string> positional;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "-h" || argument == "--help") {
      std::cout << "Usage: armor_demo [--max-frames N] [--width W] [--height H] [--fps F] [source] [model_path]\n"
                   "  --max-frames N: stop after processing N frames\n"
                   "  --width W --height H --fps F: camera resolution/fps (lower res = faster capture)\n"
                   "  source: video file, camera index, or RTSP/HTTP URL\n"
                   "  model_path: optional OpenCV DNN ONNX digit classifier path\n";
      std::exit(0);
    }
    if (argument == "--max-frames") {
      if (i + 1 >= argc) {
        std::cerr << "--max-frames requires an integer value\n";
        std::exit(1);
      }
      int max_frames = 0;
      if (!parse_int(argv[++i], max_frames) || max_frames < 0) {
        std::cerr << "--max-frames requires a non-negative integer\n";
        std::exit(1);
      }
      options.max_frames = static_cast<std::size_t>(max_frames);
      continue;
    }
    if (argument == "--width" || argument == "--height") {
      if (i + 1 >= argc) {
        std::cerr << argument << " requires an integer value\n";
        std::exit(1);
      }
      int value = 0;
      if (!parse_int(argv[++i], value) || value < 0) {
        std::cerr << argument << " requires a non-negative integer\n";
        std::exit(1);
      }
      if (argument == "--width") {
        options.width = value;
      } else {
        options.height = value;
      }
      continue;
    }
    if (argument == "--fps") {
      if (i + 1 >= argc) {
        std::cerr << "--fps requires an integer value\n";
        std::exit(1);
      }
      int value = 0;
      if (!parse_int(argv[++i], value) || value < 0) {
        std::cerr << "--fps requires a non-negative integer\n";
        std::exit(1);
      }
      options.fps = value;
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

cv::Scalar color_for_armor(auto_aim::Color color)
{
  switch (color) {
    case auto_aim::red:
      return {0, 0, 255};
    case auto_aim::blue:
      return {255, 0, 0};
    case auto_aim::purple:
    default:
      return {0, 255, 255};
  }
}

void draw_armor(cv::Mat & frame, const auto_aim::Armor & armor)
{
  const cv::Scalar color = color_for_armor(armor.color);
  tools::draw_points(frame, armor.points, color, 2);

  std::ostringstream label;
  label << auto_aim::COLORS[armor.color] << ", " << auto_aim::ARMOR_NAMES[armor.name] << ", "
        << std::fixed << std::setprecision(2) << armor.confidence;
  tools::draw_text(
    frame, label.str(), cv::Point(cvRound(armor.left.top.x), cvRound(armor.left.top.y)), 0.6, color, 2);
}

std::string source_label(const std::string & source)
{
  if (looks_like_integer(source)) {
    return "camera " + source;
  }
  return source;
}

bool open_camera(cv::VideoCapture & capture, int preferred_index, int & opened_index)
{
  std::vector<int> indices = {preferred_index};
  for (int index = 0; index <= 5; ++index) {
    if (index != preferred_index) {
      indices.push_back(index);
    }
  }

#ifdef _WIN32
  // CAP_ANY (auto) is much faster than CAP_DSHOW on MinGW OpenCV
  // (measured ~23fps vs ~10fps on the same camera); keep DSHOW as fallback.
  const std::vector<int> backends = {cv::CAP_ANY, cv::CAP_DSHOW};
#else
  const std::vector<int> backends = {cv::CAP_ANY};
#endif

  for (const int index : indices) {
    for (const int backend : backends) {
      capture.release();
      if (capture.open(index, backend)) {
        opened_index = index;
        return true;
      }
    }
  }
  return false;
}

struct DemoState
{
  std::chrono::steady_clock::time_point last_tick = std::chrono::steady_clock::now();
  double fps = 0.0;
  std::size_t frame_index = 0U;
  std::size_t total_detections = 0U;
};

}  // namespace

int main(int argc, char ** argv)
{
  DemoOptions options = parse_options(argc, argv);

  auto_aim::Detector detector(options.model_path);

  cv::VideoCapture capture;
  if (looks_like_integer(options.source)) {
    const int preferred_index = std::stoi(options.source);
    int opened_index = preferred_index;
    if (open_camera(capture, preferred_index, opened_index)) {
      options.source = std::to_string(opened_index);
    }
  } else {
    capture.open(options.source);
  }

  if (!capture.isOpened()) {
    if (looks_like_integer(options.source)) {
      std::cerr << "No available camera was found (checked indices 0-5).\n"
                   "Check the camera privacy switch, Windows camera permissions, and Device Manager.\n";
    } else {
      std::cerr << "Failed to open source: " << options.source << '\n';
    }
    return 1;
  }

  // Lower camera resolution / set fps on request (high-res capture/decoding is
  // usually what makes camera FPS drop; the detector itself runs at 100+ FPS).
  if (options.width > 0 && options.height > 0) {
    capture.set(cv::CAP_PROP_FRAME_WIDTH, options.width);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, options.height);
  }
  if (options.fps > 0) {
    capture.set(cv::CAP_PROP_FPS, options.fps);
  }
  if (looks_like_integer(options.source)) {
    std::cout << "Camera: " << capture.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
              << capture.get(cv::CAP_PROP_FRAME_HEIGHT) << " @ "
              << capture.get(cv::CAP_PROP_FPS) << " fps\n";
  }

  std::cout << "Opened source: " << options.source << '\n';
  std::cout << "Digit model: " << options.model_path << " (" << (detector.has_model() ? "loaded" : "missing")
            << ")\n";

  DemoState state;
  std::array<std::size_t, 9> class_counts{};
  const std::string window_name = "armor_demo";
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);
  cv::resizeWindow(window_name, 1280, 720);
  cv::moveWindow(window_name, 40, 40);
  cv::setWindowProperty(window_name, cv::WND_PROP_TOPMOST, 1.0);

  // Run at maximum throughput: waitKey(1) only pumps the window message loop,
  // so the demo is limited by detection/IO speed rather than a per-frame sleep.
  const int frame_delay_ms = 1;

  for (;;) {
    if (options.max_frames > 0U && state.frame_index >= options.max_frames) {
      break;
    }

    cv::Mat frame;
    if (!capture.read(frame) || frame.empty()) {
      break;  // 视频播放完毕或摄像头断开：停止并输出统计，避免回绕 seek 失败导致死循环
    }

    const std::list<auto_aim::Armor> armors = detector.detect(frame);
    state.total_detections += armors.size();

    for (const auto & armor : armors) {
      ++class_counts[static_cast<std::size_t>(armor.name)];
      draw_armor(frame, armor);
    }

    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - state.last_tick).count();
    state.last_tick = now;
    if (elapsed > 0.0) {
      // Exponential moving average: smooths out per-frame jitter from camera
      // buffering (instantaneous fps otherwise spikes between blocked reads).
      const double instantaneous = 1.0 / elapsed;
      state.fps = (state.fps <= 0.0) ? instantaneous : (state.fps * 0.9 + instantaneous * 0.1);
    }

    std::ostringstream hud;
    hud << source_label(options.source) << " | model: " << (detector.has_model() ? "loaded" : "missing")
        << " | fps: " << format_fps(state.fps) << " | frame: " << state.frame_index
        << " | detections: " << armors.size();
    cv::putText(frame, hud.str(), {12, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255, 255, 255}, 2, cv::LINE_AA);
    cv::putText(frame, "q/Esc to exit", {12, 56}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 2, cv::LINE_AA);

    ++state.frame_index;

    cv::imshow(window_name, frame);
    const int key = cv::waitKey(frame_delay_ms);
    if (key == 27 || key == 'q' || key == 'Q') {
      break;
    }
  }

  std::cout << "frames=" << state.frame_index << " detections=" << state.total_detections << '\n';
  std::cout << "classes:";
  for (std::size_t i = 0; i < class_counts.size(); ++i) {
    std::cout << ' ' << auto_aim::ARMOR_NAMES[i] << '=' << class_counts[i];
  }
  std::cout << '\n';

  return 0;
}
