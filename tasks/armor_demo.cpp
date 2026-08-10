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
  std::string template_directory;
  int brightness_threshold = 150;
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
  options.source = join_path(repo_root, fs::path("learning/assets/demo/demo.avi"));
  options.template_directory = join_path(repo_root, fs::path("learning/assets/demo/templates"));

  std::vector<std::string> positional;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "-h" || argument == "--help") {
      std::cout << "Usage: armor_demo [source] [template_dir] [brightness_threshold]\n"
                   "  source: video file, camera index, or RTSP/HTTP URL\n"
                   "  template_dir: optional directory containing digit templates\n"
                   "  brightness_threshold: optional integer, default 150\n";
      std::exit(0);
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
      options.template_directory = positional[1];
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

}  // namespace

int main(int argc, char ** argv)
{
  const DemoOptions options = parse_options(argc, argv);

  rm_assessment::ArmorDetector::Params detector_params;
  detector_params.brightness_threshold = options.brightness_threshold;
  rm_assessment::ArmorDetector detector(detector_params);
  rm_assessment::DigitRecognizer recognizer(options.template_directory);
  rm_assessment::Tracker tracker;

  cv::VideoCapture capture;
  if (looks_like_integer(options.source)) {
    capture.open(std::stoi(options.source));
  } else {
    capture.open(options.source);
  }

  if (!capture.isOpened()) {
    std::cerr << "Failed to open source: " << options.source << '\n';
    return 1;
  }

  const std::string window_name = "armor_demo";
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);

  auto last_tick = std::chrono::steady_clock::now();
  double fps = 0.0;
  std::size_t frame_index = 0U;

  for (;;) {
    cv::Mat frame;
    if (!capture.read(frame) || frame.empty()) {
      if (!looks_like_integer(options.source)) {
        capture.set(cv::CAP_PROP_POS_FRAMES, 0.0);
        continue;
      }
      break;
    }

    std::vector<rm_assessment::ArmorCandidate> armors = detector.detect(frame);
    const bool tracked = tracker.update(armors, frame.size());

    for (auto & armor : armors) {
      const auto recognition = recognizer.recognize(armor);
      const cv::Scalar text_color = recognition.reliable ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255);
      draw_candidate(frame, armor);

      const cv::Rect text_box = armor.bounding_box;
      std::ostringstream label;
      label << recognition.label << " " << std::fixed << std::setprecision(2) << recognition.confidence;
      cv::putText(frame, label.str(), {text_box.x, std::max(18, text_box.y - 6)}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  text_color, 1, cv::LINE_AA);
    }

    draw_tracker(frame, tracker);

    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - last_tick).count();
    if (elapsed > 0.0) {
      fps = 1.0 / elapsed;
    }
    last_tick = now;

    std::ostringstream hud;
    hud << source_label(options.source) << " | templates: " << (recognizer.has_templates() ? "loaded" : "none")
        << " | thresh: " << options.brightness_threshold << " | fps: " << format_fps(fps)
        << " | frame: " << frame_index++ << " | tracker: " << tracker.state();
    cv::putText(frame, hud.str(), {12, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255, 255, 255}, 2, cv::LINE_AA);
    cv::putText(frame, "q/Esc to exit", {12, 56}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 2, cv::LINE_AA);

    cv::imshow(window_name, frame);
    const int key = cv::waitKey(1);
    if (key == 27 || key == 'q' || key == 'Q') {
      break;
    }
  }

  return 0;
}
