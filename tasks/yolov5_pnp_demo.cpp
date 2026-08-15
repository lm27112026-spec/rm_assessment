/**
 * @file yolov5_pnp_demo.cpp
 * @brief Standalone YOLOv5 armor demo that solves armor pose with cv::solvePnP.
 */
#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "yolov5.hpp"

namespace
{

namespace fs = std::filesystem;

constexpr double kLightbarHeight = 56e-3;
constexpr double kSmallArmorWidth = 135e-3;
constexpr double kBigArmorWidth = 230e-3;

const std::array<double, 9> kFallbackCameraMatrix = {
  1818.3669452465165, 0.0, 751.06226574703498,
  0.0, 1822.494494078506, 530.43671556112133,
  0.0, 0.0, 1.0};
const std::array<double, 5> kFallbackDistCoeffs = {
  -0.077944626599568856, 0.15447826031486889, -0.0025714394278524674,
  0.00083016311301273629, 0.0};

struct DemoOptions
{
  std::string source = "0";
  std::string model_path = "models/yolov5/yolov5.xml";
  std::string device = "CPU";
  std::string config_path = "calibration/camera_params.yaml";
  std::string camera_matrix_csv;
  std::string dist_coeffs_csv;
  bool big_armor = false;
  double axis_length = 0.05;
  double max_reproj_error = 40.0;
  std::size_t max_frames = 0U;
};

bool looks_like_integer(const std::string & value)
{
  if (value.empty()) return false;
  std::size_t index = 0;
  if (value[0] == '+' || value[0] == '-') index = 1;
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

bool parse_double(const std::string & text, double & value)
{
  try {
    std::size_t consumed = 0;
    const double parsed = std::stod(text, &consumed);
    if (consumed != text.size()) return false;
    value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

bool parse_csv_doubles(const std::string & text, std::vector<double> & values)
{
  values.clear();
  std::istringstream stream(text);
  std::string token;
  while (std::getline(stream, token, ',')) {
    const std::size_t first = token.find_first_not_of(" \t\r\n");
    const std::size_t last = token.find_last_not_of(" \t\r\n");
    if (first == std::string::npos) return false;
    const std::string trimmed = token.substr(first, last - first + 1);
    double value = 0.0;
    if (!parse_double(trimmed, value)) return false;
    values.push_back(value);
  }
  return !values.empty();
}

void print_usage()
{
  std::cout << "Usage: yolov5_pnp_demo [options] [source] [model_path]\n";
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
      if (i + 1 >= argc) std::exit(1);
      parse_size_t(argv[++i], options.max_frames);
      continue;
    }
    if (argument == "--device") {
      if (i + 1 >= argc) std::exit(1);
      options.device = argv[++i];
      continue;
    }
    if (argument == "--config") {
      if (i + 1 >= argc) std::exit(1);
      options.config_path = argv[++i];
      continue;
    }
    if (argument == "--camera-matrix") {
      if (i + 1 >= argc) std::exit(1);
      options.camera_matrix_csv = argv[++i];
      continue;
    }
    if (argument == "--dist-coeffs") {
      if (i + 1 >= argc) std::exit(1);
      options.dist_coeffs_csv = argv[++i];
      continue;
    }
    if (argument == "--armor-size") {
      if (i + 1 >= argc) std::exit(1);
      std::string value = argv[++i];
      options.big_armor = (value == "big" || value == "BIG");
      continue;
    }
    if (argument == "--axis-length") {
      if (i + 1 >= argc) std::exit(1);
      parse_double(argv[++i], options.axis_length);
      continue;
    }
    if (argument == "--max-reproj-error") {
      if (i + 1 >= argc) std::exit(1);
      parse_double(argv[++i], options.max_reproj_error);
      continue;
    }
    positional.push_back(argument);
  }

  if (!positional.empty()) options.source = positional[0];
  if (positional.size() >= 2U) options.model_path = positional[1];

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

bool open_camera(cv::VideoCapture & capture, int preferred_index, int & opened_index)
{
  std::vector<int> indices = {preferred_index};
  for (int index = 0; index <= 5; ++index) {
    if (index != preferred_index) indices.push_back(index);
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

struct Intrinsics
{
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  std::string source_label;
};

bool parse_yaml_array_line(const std::string & line, const std::string & key, std::vector<double> & values)
{
  const std::size_t first = line.find_first_not_of(" \t");
  if (first == std::string::npos) return false;
  const std::string text = line.substr(first);
  if (text.rfind(key, 0) != 0) return false;
  const std::size_t colon = text.find(':');
  if (colon == std::string::npos) return false;
  const std::size_t open = text.find('[', colon);
  const std::size_t close = text.find(']', open);
  if (open == std::string::npos || close == std::string::npos || close <= open) return false;
  return parse_csv_doubles(text.substr(open + 1, close - open - 1), values);
}

bool load_intrinsics_from_config(const std::string & path, cv::Mat & camera_matrix, cv::Mat & dist_coeffs)
{
  std::ifstream stream(path);
  if (!stream.is_open()) return false;

  std::vector<double> camera_data;
  std::vector<double> dist_data;
  std::string line;
  while (std::getline(stream, line)) {
    std::vector<double> values;
    if (parse_yaml_array_line(line, "camera_matrix", values)) {
      camera_data = std::move(values);
    } else if (parse_yaml_array_line(line, "distort_coeffs", values)) {
      dist_data = std::move(values);
    }
  }
  if (camera_data.size() != 9U) return false;
  if (dist_data.size() != 4U && dist_data.size() != 5U) return false;

  camera_matrix = cv::Mat(3, 3, CV_64F);
  dist_coeffs = cv::Mat(1, 5, CV_64F, cv::Scalar(0.0));
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      camera_matrix.at<double>(row, col) = camera_data[static_cast<std::size_t>(row * 3 + col)];
    }
  }
  for (std::size_t i = 0; i < dist_data.size(); ++i) {
    dist_coeffs.at<double>(0, static_cast<int>(i)) = dist_data[i];
  }
  return true;
}

Intrinsics resolve_intrinsics(const DemoOptions & options)
{
  Intrinsics result;
  if (load_intrinsics_from_config(options.config_path, result.camera_matrix, result.dist_coeffs)) {
    result.source_label = "config " + options.config_path;
    return result;
  }
  result.camera_matrix = cv::Mat(3, 3, CV_64F);
  result.dist_coeffs = cv::Mat(1, 5, CV_64F, cv::Scalar(0.0));
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      result.camera_matrix.at<double>(row, col) =
        kFallbackCameraMatrix[static_cast<std::size_t>(row * 3 + col)];
    }
  }
  for (std::size_t i = 0; i < kFallbackDistCoeffs.size(); ++i) {
    result.dist_coeffs.at<double>(0, static_cast<int>(i)) = kFallbackDistCoeffs[i];
  }
  result.source_label = "built-in fallback";
  return result;
}

struct PoseResult
{
  bool solved = false;
  bool valid = false;
  cv::Vec3d rvec{};
  cv::Vec3d tvec{};
  double reproj_error = 0.0;
  double distance_3d = 0.0;
  double distance_horizontal = 0.0;
  double distance_z = 0.0;
  double yaw_deg = 0.0;
  double pitch_deg = 0.0;
  double roll_deg = 0.0;
};

std::array<cv::Point2f, 4> order_corners_tl_tr_br_bl(const std::array<cv::Point2f, 4> & corners)
{
  std::vector<cv::Point2f> points(corners.begin(), corners.end());
  std::sort(points.begin(), points.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    if (a.y != b.y) return a.y < b.y;
    return a.x < b.x;
  });
  const cv::Point2f & top_left = (points[0].x <= points[1].x) ? points[0] : points[1];
  const cv::Point2f & top_right = (points[0].x <= points[1].x) ? points[1] : points[0];
  const cv::Point2f & bottom_right = (points[2].x <= points[3].x) ? points[3] : points[2];
  const cv::Point2f & bottom_left = (points[2].x <= points[3].x) ? points[2] : points[3];
  return {top_left, top_right, bottom_right, bottom_left};
}

PoseResult solve_armor_pose(
  const std::array<cv::Point2f, 4> & corners_in, double armor_width, double lightbar_height,
  const cv::Mat & camera_matrix, const cv::Mat & dist_coeffs, double max_reproj_error)
{
  PoseResult result;
  const std::array<cv::Point2f, 4> corners = order_corners_tl_tr_br_bl(corners_in);

  const double half_width = armor_width / 2.0;
  const double half_height = lightbar_height / 2.0;
  const std::vector<cv::Point3f> object_points = {
    cv::Point3f(-half_width, -half_height, 0.0),
    cv::Point3f(half_width, -half_height, 0.0),
    cv::Point3f(half_width, half_height, 0.0),
    cv::Point3f(-half_width, half_height, 0.0)};
  const std::vector<cv::Point2f> image_points(corners.begin(), corners.end());

  if (!cv::solvePnP(
        object_points, image_points, camera_matrix, dist_coeffs, result.rvec, result.tvec, false,
        cv::SOLVEPNP_IPPE)) {
    return result;
  }
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(result.rvec[i]) || !std::isfinite(result.tvec[i])) return result;
  }
  if (result.tvec[2] <= 0.0) return result;
  result.solved = true;

  std::vector<cv::Point2f> projected;
  cv::projectPoints(object_points, result.rvec, result.tvec, camera_matrix, dist_coeffs, projected);
  double error = 0.0;
  for (std::size_t i = 0; i < image_points.size(); ++i) {
    error += cv::norm(image_points[i] - projected[i]);
  }
  result.reproj_error = error / static_cast<double>(image_points.size());
  if (result.reproj_error > max_reproj_error) return result;

  const double x = result.tvec[0];
  const double y = result.tvec[1];
  const double z = result.tvec[2];
  result.distance_3d = std::sqrt(x * x + y * y + z * z);
  result.distance_horizontal = std::sqrt(x * x + z * z);
  result.distance_z = z;

  cv::Mat rmat;
  cv::Rodrigues(result.rvec, rmat);
  const double r00 = rmat.at<double>(0, 0);
  const double r10 = rmat.at<double>(1, 0);
  const double r20 = rmat.at<double>(2, 0);
  const double r21 = rmat.at<double>(2, 1);
  const double r22 = rmat.at<double>(2, 2);
  result.yaw_deg = std::atan2(r10, r00) * 180.0 / CV_PI;
  result.pitch_deg = std::asin(std::max(-1.0, std::min(1.0, -r20))) * 180.0 / CV_PI;
  result.roll_deg = std::atan2(r21, r22) * 180.0 / CV_PI;

  result.valid = true;
  return result;
}

void draw_pose_target(
  cv::Mat & frame, const rm_assessment::yolov5::Detection & detection,
  const std::array<cv::Point2f, 4> & ordered_corners, const PoseResult & pose,
  double axis_length, const cv::Mat & camera_matrix, const cv::Mat & dist_coeffs)
{
  // 1. 绘制绿色检测框
  cv::rectangle(frame, detection.box, {0, 255, 0}, 2, cv::LINE_AA);

  // 2. 绘制 4 个角点 (TL, TR, BR, BL)
  const char * corner_names[] = {"TL", "TR", "BR", "BL"};
  for (std::size_t i = 0; i < ordered_corners.size(); ++i) {
    cv::circle(frame, ordered_corners[i], 4, {0, 255, 255}, 1, cv::LINE_AA);
    cv::putText(
      frame, corner_names[i], ordered_corners[i] + cv::Point2f(5.0F, -5.0F),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, {0, 255, 255}, 1, cv::LINE_AA);
  }

  // 3. 绘制中心点
  cv::Point2f center(0.0F, 0.0F);
  for (const auto & corner : ordered_corners) {
    center += corner;
  }
  center *= 0.25F;
  cv::circle(frame, center, 3, {0, 255, 0}, -1, cv::LINE_AA);

  // 4. 当位姿解算成功时：绘制坐标轴，并在【左上方 y=76, y=102】绘制距离和姿态信息
  if (pose.valid) {
    // 绘制中心坐标轴
    cv::drawFrameAxes(frame, camera_matrix, dist_coeffs, pose.rvec, pose.tvec, axis_length, 2);

    // 【第 2 行：左上方绿色距离信息 (y = 76)】
    std::ostringstream dist_text;
    dist_text << "d3D=" << std::fixed << std::setprecision(2) << pose.distance_3d << "m"
              << "  horiz=" << pose.distance_horizontal << "m"
              << "  Z=" << pose.distance_z << "m";
    cv::putText(frame, dist_text.str(), {12, 76}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 0, 0}, 2, cv::LINE_AA);
    cv::putText(frame, dist_text.str(), {12, 76}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 255, 0}, 1, cv::LINE_AA);

    // 【第 3 行：左上方绿色姿态信息 (y = 102)】
    std::ostringstream ypr_text;
    ypr_text << "YPR(cam)=" << std::fixed << std::setprecision(1)
             << pose.yaw_deg << "/" << pose.pitch_deg << "/" << pose.roll_deg << "deg";
    cv::putText(frame, ypr_text.str(), {12, 102}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 0, 0}, 2, cv::LINE_AA);
    cv::putText(frame, ypr_text.str(), {12, 102}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 255, 0}, 1, cv::LINE_AA);
  } else {
    const int line_y = std::max(18, cvRound(detection.box.y) - 6);
    std::ostringstream label;
    label << "pose rejected";
    if (pose.solved) {
      label << " (err=" << std::fixed << std::setprecision(1) << pose.reproj_error << "px)";
    } else {
      label << " (solve failed)";
    }
    cv::putText(
      frame, label.str(), {cvRound(detection.box.x), line_y}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
      {0, 0, 255}, 1, cv::LINE_AA);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const DemoOptions options = parse_options(argc, argv);

    if (!fs::exists(options.model_path)) {
      std::cerr << "Model not found: " << options.model_path << '\n';
      return 1;
    }

    rm_assessment::yolov5::YOLOV5Detector detector(options.model_path, options.device);
    const Intrinsics intrinsics = resolve_intrinsics(options);
    const double armor_width = options.big_armor ? kBigArmorWidth : kSmallArmorWidth;

    cv::VideoCapture capture;
    if (looks_like_integer(options.source)) {
      const int preferred_index = std::stoi(options.source);
      int opened_index = preferred_index;
      if (open_camera(capture, preferred_index, opened_index)) {
        std::cout << "Opened camera " << opened_index << '\n';
      }
    } else {
      capture.open(options.source);
    }

    if (!capture.isOpened()) {
      std::cerr << "Failed to open source: " << options.source << '\n';
      return 1;
    }

    std::cout << "Source: " << source_label(options.source) << '\n';
    std::cout << "Model: " << options.model_path << '\n';
    std::cout << "Device: " << options.device << '\n';

    std::chrono::steady_clock::time_point last_tick = std::chrono::steady_clock::now();
    double fps = 0.0;
    std::size_t frame_index = 0U;
    std::size_t total_detections = 0U;
    std::size_t pose_solved = 0U;
    std::size_t pose_rejected = 0U;

    const std::string window_name = "yolov5_pnp_demo";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    while (true) {
      if (options.max_frames > 0U && frame_index >= options.max_frames) {
        break;
      }

      cv::Mat frame;
      if (!capture.read(frame) || frame.empty()) {
        break;
      }

      const auto detections = detector.detect(frame);
      total_detections += detections.size();

      for (const auto & detection : detections) {
        const std::array<cv::Point2f, 4> ordered = order_corners_tl_tr_br_bl(detection.corners);
        const PoseResult pose = solve_armor_pose(
          detection.corners, armor_width, kLightbarHeight, intrinsics.camera_matrix,
          intrinsics.dist_coeffs, options.max_reproj_error);
        if (pose.valid) {
          ++pose_solved;
        } else {
          ++pose_rejected;
        }
        draw_pose_target(
          frame, detection, ordered, pose, options.axis_length, intrinsics.camera_matrix,
          intrinsics.dist_coeffs);
      }

      const auto now = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now - last_tick).count();
      if (elapsed > 0.0) {
        fps = 1.0 / elapsed;
      }
      last_tick = now;

      // 【第 1 行：白色系统状态 (y = 50)】
      std::ostringstream hud;
      hud << source_label(options.source) << " | fps: " << format_fps(fps)
          << " | frame: " << frame_index << " | detections: " << detections.size()
          << " | pose: " << pose_solved << " | rejected: " << pose_rejected;
      
      // 黑色描边 + 白色字体
      cv::putText(frame, hud.str(), {12, 50}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 0, 0}, 2, cv::LINE_AA);
      cv::putText(frame, hud.str(), {12, 50}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 1, cv::LINE_AA);

      cv::imshow(window_name, frame);
      const int key = cv::waitKey(1);
      if (key == 27 || key == 'q') {
        break;
      }

      ++frame_index;
    }

    cv::destroyAllWindows();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "yolov5_pnp_demo failed: " << error.what() << '\n';
    return 1;
  }
}