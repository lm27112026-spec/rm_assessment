#include <exception>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "detector.hpp"
#include "digit_recognizer.hpp"
#include "tracker.hpp"

namespace
{
using rm_assessment::ArmorCandidate;
using rm_assessment::ArmorColor;
using rm_assessment::ArmorDetector;
using rm_assessment::DigitRecognizer;
using rm_assessment::Tracker;

class TestFailure : public std::runtime_error
{
public:
  explicit TestFailure(const std::string & message) : std::runtime_error(message) {}
};

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw TestFailure(message);
  }
}

cv::Mat make_digit_roi(int digit)
{
  cv::Mat roi(96, 160, CV_8UC3, cv::Scalar(0, 0, 0));
  const std::string text = std::to_string(digit);
  constexpr int font = cv::FONT_HERSHEY_SIMPLEX;
  constexpr double scale = 2.4;
  constexpr int thickness = 6;
  int baseline = 0;
  const cv::Size text_size = cv::getTextSize(text, font, scale, thickness, &baseline);
  const cv::Point origin(
    (roi.cols - text_size.width) / 2, (roi.rows + text_size.height) / 2 - baseline);
  cv::putText(roi, text, origin, font, scale, cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);
  return roi;
}

std::filesystem::path default_model_path()
{
  return std::filesystem::current_path() / "learning" / "assets" / "tiny_resnet.onnx";
}

void draw_lightbar(cv::Mat & image, const cv::Point2f & center)
{
  const cv::Size2f size(14.0F, 76.0F);
  const cv::RotatedRect rect(center, size, 0.0F);
  cv::Point2f points[4];
  rect.points(points);
  std::vector<cv::Point> polygon;
  polygon.reserve(4);
  for (const auto & point : points) {
    polygon.emplace_back(cv::Point(cvRound(point.x), cvRound(point.y)));
  }
  cv::fillConvexPoly(image, polygon, cv::Scalar(255, 80, 20), cv::LINE_AA);
}

ArmorDetector::Params detector_params()
{
  ArmorDetector::Params params;
  params.brightness_threshold = 70;
  params.color_difference_threshold = 25;
  params.morph_kernel_size = 3;
  params.morph_iterations = 1;
  params.min_contour_area = 20.0;
  params.min_lightbar_length = 20.0;
  params.min_lightbar_ratio = 2.0;
  params.max_lightbar_ratio = 12.0;
  params.min_armor_ratio = 0.8;
  params.max_armor_ratio = 4.0;
  params.max_side_ratio = 1.25;
  params.max_rectangular_error_deg = 10.0;
  params.max_vertical_center_delta_ratio = 0.2;
  params.min_lightbar_gap_ratio = 0.8;
  params.max_lightbar_gap_ratio = 3.0;
  return params;
}

void test_detector_finds_synthetic_blue_lightbars()
{
  cv::Mat image(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
  draw_lightbar(image, {120.0F, 120.0F});
  draw_lightbar(image, {200.0F, 120.0F});

  const ArmorDetector detector(detector_params());
  const std::vector<ArmorCandidate> candidates = detector.detect(image);

  std::ostringstream detail;
  detail << "ArmorDetector returned " << candidates.size()
         << " candidates for two synthetic blue lightbars";
  require(!candidates.empty(), detail.str());
  require(candidates.front().color == ArmorColor::blue, "Detector candidate color is not blue");
  require(!candidates.front().normalized_roi.empty(), "Detector candidate normalized ROI is empty");
}

void test_digit_recognizer_missing_model_is_safe()
{
  const DigitRecognizer recognizer("learning/assets/does_not_exist.onnx");
  require(!recognizer.has_model(), "DigitRecognizer reported a missing model as loaded");

  const rm_assessment::RecognitionResult result = recognizer.recognize(make_digit_roi(3));
  require(!result.reliable, "Missing-model recognition should not be reliable");
  require(result.label == "Unknown", "Missing-model recognition label should be Unknown");
  require(result.digit == -1, "Missing-model recognition digit should be -1");
  require(result.confidence == 0.0, "Missing-model recognition confidence should be 0");
}

void test_digit_recognizer_real_model_output_contract()
{
  const auto model_path = default_model_path();
  require(std::filesystem::exists(model_path), "Expected test model is missing: " + model_path.string());

  const DigitRecognizer recognizer(model_path.string());
  require(recognizer.has_model(), "DigitRecognizer did not load tiny_resnet.onnx");

  const rm_assessment::RecognitionResult result = recognizer.recognize(make_digit_roi(3));
  std::ostringstream detail;
  detail << "DigitRecognizer result label=" << result.label << " digit=" << result.digit
         << " confidence=" << result.confidence << " reliable=" << result.reliable;
  require(result.confidence >= 0.0 && result.confidence <= 1.0, detail.str());
  if (result.reliable) {
    require(result.digit >= 1 && result.digit <= 5, detail.str());
    require(result.label == std::to_string(result.digit), detail.str());
  } else {
    require(result.digit == -1, detail.str());
    require(result.label == "Unknown", detail.str());
  }
}

ArmorCandidate make_candidate(const cv::Point2f & center, ArmorColor color = ArmorColor::blue)
{
  ArmorCandidate candidate;
  candidate.color = color;
  candidate.center = center;
  candidate.bounding_box = cv::Rect(
    cvRound(center.x - 30.0F), cvRound(center.y - 20.0F), 60, 40);
  candidate.corners = {
    cv::Point2f(center.x - 30.0F, center.y - 20.0F),
    cv::Point2f(center.x + 30.0F, center.y - 20.0F),
    cv::Point2f(center.x + 30.0F, center.y + 20.0F),
    cv::Point2f(center.x - 30.0F, center.y + 20.0F)};
  return candidate;
}

std::vector<ArmorCandidate> single_candidate(float x)
{
  return {make_candidate({x, 120.0F})};
}

void require_state(const Tracker & tracker, const std::string & expected, const std::string & step)
{
  const std::string actual = tracker.state();
  require(actual == expected, step + ": expected state " + expected + ", got " + actual);
}

void test_tracker_state_transitions()
{
  Tracker::Params params;
  params.min_detect_count = 2;
  params.max_temp_lost_count = 1;
  params.max_match_distance = 90.0;
  params.min_box_size = {10.0F, 10.0F};

  Tracker tracker(params);
  require_state(tracker, "lost", "initial");

  require(tracker.update(single_candidate(100.0F), {320, 240}), "Tracker rejected first candidate");
  require_state(tracker, "detecting", "after first detection");

  require(tracker.update(single_candidate(104.0F), {320, 240}), "Tracker rejected second candidate");
  require_state(tracker, "tracking", "after second detection");

  require(tracker.update({}, {320, 240}), "Tracker did not keep target during first missing frame");
  require_state(tracker, "temp_lost", "after first missing frame");

  require(!tracker.update({}, {320, 240}), "Tracker should report false after temp lost threshold");
  require_state(tracker, "lost", "after temp lost threshold");
}

void run_test(const std::string & name, void (*test)())
{
  try {
    test();
    std::cout << "[PASS] " << name << '\n';
  } catch (const std::exception & error) {
    std::cerr << "[FAIL] " << name << ": " << error.what() << '\n';
    throw;
  }
}

}  // namespace

int main()
{
  try {
    run_test("ArmorDetector synthetic blue lightbars", test_detector_finds_synthetic_blue_lightbars);
    run_test("DigitRecognizer missing model safe", test_digit_recognizer_missing_model_is_safe);
    run_test("DigitRecognizer real model output contract", test_digit_recognizer_real_model_output_contract);
    run_test("Tracker state transitions", test_tracker_state_transitions);
  } catch (const std::exception &) {
    return 1;
  }
  return 0;
}
