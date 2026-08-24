#include "myCamera.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <sstream>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "armor.hpp"
#include "detector.hpp"
#include "img_tools.hpp"

namespace
{
struct ExposureState
{
  io::myCamera * camera = nullptr;
  double min_exposure = -13.0;
  double scale = 100.0;
  double current_exposure = -6.0;
};

double slider_to_exposure(int slider, const ExposureState & state)
{
  return state.min_exposure + static_cast<double>(slider) / state.scale;
}

int exposure_to_slider(double exposure, const ExposureState & state)
{
  return static_cast<int>(std::lround((exposure - state.min_exposure) * state.scale));
}

void on_exposure_changed(int slider, void * userdata)
{
  auto * state = static_cast<ExposureState *>(userdata);
  if (state == nullptr || state->camera == nullptr) {
    return;
  }

  state->current_exposure = slider_to_exposure(slider, *state);
  if (!state->camera->setExposure(state->current_exposure)) {
    std::cerr << "warning: backend rejected exposure " << state->current_exposure << '\n';
  }
}

bool save_exposure(const std::string & output_path, double exposure)
{
  const std::filesystem::path path(output_path);
  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path());
  }

  std::ofstream out(path);
  if (!out.is_open()) {
    return false;
  }

  out << "# Auto-saved by exposure_tune\n";
  out << "# Value is requested through io::myCamera; most camera backends quantize it.\n";
  out << std::setprecision(10) << "exposure: " << exposure << '\n';
  return true;
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

double parse_double_arg(char ** argv, int argc, int index, double fallback)
{
  if (argc <= index) {
    return fallback;
  }

  try {
    return std::stod(argv[index]);
  } catch (const std::exception &) {
    return fallback;
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  const std::string source = (argc > 1) ? argv[1] : std::string("0");
  const std::string output_path = (argc > 2) ? argv[2] : std::string("calibration/camera_exposure.yaml");
  const std::string model_path = (argc > 6) ? argv[6] : std::string("learning/assets/tiny_resnet.onnx");

  ExposureState state;
  state.min_exposure = parse_double_arg(argv, argc, 3, -13.0);
  const double max_exposure = parse_double_arg(argv, argc, 4, 0.0);
  state.scale = parse_double_arg(argv, argc, 5, 100.0);
  if (state.scale <= 0.0 || max_exposure <= state.min_exposure) {
    std::cerr << "usage: exposure_tune [source] [output_yaml] [min_exposure] [max_exposure] [scale] [model_path]\n";
    return 1;
  }

  io::myCamera camera(source);
  state.camera = &camera;
  auto_aim::Detector detector(model_path);

  cv::Mat image;
  std::chrono::steady_clock::time_point timestamp;
  if (!camera.read(image, timestamp) || image.empty()) {
    std::cerr << "failed to read from camera source: " << source << '\n';
    return 1;
  }

  const std::string window_name = "Exposure Tune";
  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

  state.current_exposure = camera.getExposure();
  if (state.current_exposure < state.min_exposure || state.current_exposure > max_exposure) {
    state.current_exposure = state.min_exposure;
  }

  const int slider_max = exposure_to_slider(max_exposure, state);
  int slider = exposure_to_slider(state.current_exposure, state);
  cv::createTrackbar("Exposure", window_name, &slider, slider_max, on_exposure_changed, &state);
  on_exposure_changed(slider, &state);

  std::cout << "Source: " << source << '\n';
  std::cout << "Digit model: " << model_path << " (" << (detector.has_model() ? "loaded" : "missing") << ")\n";
  std::cout << "Exposure range: " << state.min_exposure << " to " << max_exposure << '\n';
  std::cout << "Controls: drag the Exposure trackbar, press q or ESC to save and exit.\n";

  while (true) {
    if (!camera.read(image, timestamp) || image.empty()) {
      std::cerr << "read failed\n";
      break;
    }

    const std::list<auto_aim::Armor> armors = detector.detect(image);
    for (const auto & armor : armors) {
      draw_armor(image, armor);
    }

    state.current_exposure = slider_to_exposure(cv::getTrackbarPos("Exposure", window_name), state);
    std::ostringstream exposure_text;
    exposure_text << "Exposure: " << std::fixed << std::setprecision(2) << state.current_exposure
                  << " | armors: " << armors.size() << " | model: " << (detector.has_model() ? "loaded" : "missing");
    cv::putText(image, exposure_text.str(), {20, 32}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 255}, 2,
      cv::LINE_AA);
    cv::putText(image, "q/ESC: save and exit", {20, 64}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 255}, 2,
      cv::LINE_AA);
    cv::imshow(window_name, image);

    const int key = cv::waitKey(1);
    if (key == 27 || key == 'q' || key == 'Q') {
      break;
    }
  }

  cv::destroyAllWindows();

  if (!save_exposure(output_path, state.current_exposure)) {
    std::cerr << "failed to save exposure to " << output_path << '\n';
    return 1;
  }

  std::cout << "Saved exposure " << state.current_exposure << " to " << output_path << '\n';
  return 0;
}
