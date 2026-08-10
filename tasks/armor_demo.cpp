//题目2
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "src/armor.hpp"
#include "src/detector.hpp"
#include "src/digit_recognizer.hpp"
#include "src/tracker.hpp"

namespace
{

namespace fs = std::filesystem;

struct DemoOptions
{
  std::string source;
  std::string model_path;
  int brightness_threshold = 150;
  bool headless = false;
  bool has_max_frames = false;
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
      std::cout << "Usage: armor_demo [--headless] [--max-frames N] [source] [model_path] [brightness_threshold]\n"
                   "  --headless: disable GUI and print a summary on exit\n"
                   "  --max-frames N: stop after processing N frames in headless mode\n"
                   "  source: video file, camera index, or RTSP/HTTP URL\n"
                   "  model_path: optional OpenCV DNN ONNX digit classifier path\n"
                   "  brightness_threshold: optional integer, default 150\n";
      std::exit(0);
    }
    if (argument == "--headless") {
      options.headless = true;
      continue;
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
      options.has_max_frames = true;
      options.max_frames = static_cast<std::size_t>(max_frames);
      continue;
    }
    positional.push_back(argument);
  }

  if (!positional.empty()) {
    options.source = positional[0];
  }
  if (positional.size() >= 2U) {
    const bool second_is_threshold = positional.size() == 2U && looks_like_integer(positional[1]);
    if (second_is_threshold) {
      parse_int(positional[1], options.brightness_threshold);
    } else {
      options.model_path = positional[1];
      if (positional.size() >= 3U) {
        parse_int(positional[2], options.brightness_threshold);
      }
    }
  }

  if (options.brightness_threshold < 0) options.brightness_threshold = 0;
  if (options.brightness_threshold > 255) options.brightness_threshold = 255;
  return options;
}

std::string format_fps(double fps)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(1) << fps;
  return oss.str();
}

cv::Scalar color_for_armor(rm_assessment::ArmorColor color)
{
  switch (color) {
    case rm_assessment::ArmorColor::red:
      return {0, 0, 255};
    case rm_assessment::ArmorColor::blue:
      return {255, 0, 0};
    case rm_assessment::ArmorColor::unknown:
    default:
      return {0, 255, 255};
  }
}

void draw_candidate(cv::Mat & frame, const rm_assessment::ArmorCandidate & candidate)
{
  const cv::Scalar color = color_for_armor(candidate.color);
  std::vector<cv::Point> polygon;
  polygon.reserve(candidate.corners.size());
  for (const auto & corner : candidate.corners) {
    polygon.emplace_back(cvRound(corner.x), cvRound(corner.y));
  }

  if (polygon.size() == 4U) {
    const std::vector<std::vector<cv::Point>> contours = {polygon};
    cv::polylines(frame, contours, true, color, 2, cv::LINE_AA);
  }

  for (const auto & corner : polygon) {
    cv::circle(frame, corner, 3, color, cv::FILLED, cv::LINE_AA);
  }

  cv::circle(frame, cv::Point(cvRound(candidate.center.x), cvRound(candidate.center.y)), 4, color, cv::FILLED,
             cv::LINE_AA);
}

void draw_tracker(cv::Mat & frame, const rm_assessment::Tracker & tracker)
{
  if (!tracker.has_target()) return;

  const cv::Rect2f box = tracker.tracked_box();
  cv::rectangle(frame, box, {0, 255, 255}, 2, cv::LINE_AA);
  cv::circle(frame, tracker.tracked_center(), 4, {0, 255, 255}, cv::FILLED, cv::LINE_AA);

  const std::string label = "pred " + tracker.state() + " " + rm_assessment::to_string(tracker.tracked_color());
  cv::putText(frame, label, {cvRound(box.x), std::max(20, cvRound(box.y) - 8)}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
              {0, 255, 255}, 2, cv::LINE_AA);
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
  const std::vector<int> backends = {cv::CAP_MSMF};
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
  std::size_t total_candidate_detections = 0U;
  std::size_t frames_with_tracker_target = 0U;
  std::size_t reliable_recognitions = 0U;
};

void process_frame(cv::Mat & frame, const DemoOptions & options, rm_assessment::ArmorDetector & detector,
                   rm_assessment::DigitRecognizer & recognizer, rm_assessment::Tracker & tracker, DemoState & state,
                   bool draw_gui)
{
  std::vector<rm_assessment::ArmorCandidate> armors = detector.detect(frame);
  state.total_candidate_detections += armors.size();
  const bool tracked = tracker.update(armors, frame.size());
  if (tracked && tracker.has_target()) {
    ++state.frames_with_tracker_target;
  }

  for (auto & armor : armors) {
    const auto recognition = recognizer.recognize(armor);
    if (recognition.reliable) {
      ++state.reliable_recognitions;
    }

    if (draw_gui) {
      const cv::Scalar text_color = recognition.reliable ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255);
      draw_candidate(frame, armor);

      const cv::Rect text_box = armor.bounding_box;
      std::ostringstream label;
      label << recognition.label << " " << std::fixed << std::setprecision(2) << recognition.confidence;
      cv::putText(frame, label.str(), {text_box.x, std::max(18, text_box.y - 6)}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  text_color, 1, cv::LINE_AA);
    }
  }

  if (draw_gui) {
    draw_tracker(frame, tracker);
  }

  const auto now = std::chrono::steady_clock::now();
  const double elapsed = std::chrono::duration<double>(now - state.last_tick).count();
  if (elapsed > 0.0) {
    state.fps = 1.0 / elapsed;
  }
  state.last_tick = now;

  if (draw_gui) {
    std::ostringstream hud;
    hud << source_label(options.source) << " | model: " << (recognizer.has_model() ? "loaded" : "missing")
        << " | thresh: " << options.brightness_threshold << " | fps: " << format_fps(state.fps)
        << " | frame: " << state.frame_index << " | tracker: " << tracker.state();
    cv::putText(frame, hud.str(), {12, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255, 255, 255}, 2, cv::LINE_AA);
    cv::putText(frame, "q/Esc to exit", {12, 56}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 2, cv::LINE_AA);
  }

  ++state.frame_index;
}

}  // namespace

int main(int argc, char ** argv)
{
  DemoOptions options = parse_options(argc, argv);

  rm_assessment::ArmorDetector::Params detector_params;
  detector_params.brightness_threshold = options.brightness_threshold;
  rm_assessment::ArmorDetector detector(detector_params);
  rm_assessment::DigitRecognizer recognizer(options.model_path);
  rm_assessment::Tracker tracker;

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

  std::cout << "Opened source: " << options.source << '\n';
  std::cout << "Digit model: " << options.model_path << " (" << (recognizer.has_model() ? "loaded" : "missing") << ")\n";

  DemoState state;

  if (!options.headless) {
    const std::string window_name = "armor_demo";
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name, 1280, 720);
    cv::moveWindow(window_name, 40, 40);
    cv::setWindowProperty(window_name, cv::WND_PROP_TOPMOST, 1.0);

    int frame_delay_ms = 1;
    if (!looks_like_integer(options.source)) {
      const double source_fps = capture.get(cv::CAP_PROP_FPS);
      if (source_fps > 1.0 && source_fps <= 240.0) {
        frame_delay_ms = std::clamp(static_cast<int>(1000.0 / source_fps), 1, 1000);
      }
    }

    for (;;) {
      cv::Mat frame;
      if (!capture.read(frame) || frame.empty()) {
        if (!looks_like_integer(options.source)) {
          capture.set(cv::CAP_PROP_POS_FRAMES, 0.0);
          continue;
        }
        break;
      }

      process_frame(frame, options, detector, recognizer, tracker, state, true);

      cv::imshow(window_name, frame);
      const int key = cv::waitKey(frame_delay_ms);
      if (key == 27 || key == 'q' || key == 'Q') {
        break;
      }
    }
  } else {
    for (;;) {
      if (options.has_max_frames && state.frame_index >= options.max_frames) {
        break;
      }

      cv::Mat frame;
      if (!capture.read(frame) || frame.empty()) {
        break;
      }

      process_frame(frame, options, detector, recognizer, tracker, state, false);
    }

    std::cout << "frames=" << state.frame_index << " candidates=" << state.total_candidate_detections
              << " tracker_frames=" << state.frames_with_tracker_target
              << " reliable_recognitions=" << state.reliable_recognitions << '\n';
  }

  return 0;
}
