/**
 * @file digit_recognizer.cpp
 * @brief OpenCV DNN ONNX digit recognizer implementation.
 */
#include "digit_recognizer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>

#include <opencv2/imgproc.hpp>

namespace rm_assessment
{
namespace
{

constexpr int kInputSize = 32;
constexpr int kExpectedClassCount = 9;
constexpr int kKnownDigitClasses = 5;
constexpr std::array<const char *, kKnownDigitClasses> kDigitLabels = {"1", "2", "3", "4", "5"};

cv::Mat to_gray(const cv::Mat & input)
{
  if (input.empty()) {
    return {};
  }
  if (input.channels() == 1) {
    return input.clone();
  }
  if (input.channels() == 3) {
    cv::Mat gray;
    cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
    return gray;
  }
  if (input.channels() == 4) {
    cv::Mat gray;
    cv::cvtColor(input, gray, cv::COLOR_BGRA2GRAY);
    return gray;
  }
  return {};
}

}  // namespace

DigitRecognizer::DigitRecognizer(const std::string & model_path)
{
  set_model_path(model_path);
}

void DigitRecognizer::set_model_path(const std::string & model_path)
{
  model_path_ = model_path;
  model_loaded_ = load_model(model_path_);
}

void DigitRecognizer::set_template_directory(const std::string & model_path)
{
  set_model_path(model_path);
}

bool DigitRecognizer::has_model() const noexcept
{
  return model_loaded_;
}

bool DigitRecognizer::has_templates() const noexcept
{
  return has_model();
}

RecognitionResult DigitRecognizer::recognize(const ArmorCandidate & candidate) const
{
  return recognize(candidate.normalized_roi);
}

RecognitionResult DigitRecognizer::recognize(const cv::Mat & normalized_roi) const
{
  if (!model_loaded_ || normalized_roi.empty()) {
    return unknown_result();
  }

  try {
    const cv::Mat input = preprocess_roi(normalized_roi);
    if (input.empty()) {
      return unknown_result();
    }

    cv::Mat blob = cv::dnn::blobFromImage(input, 1.0 / 255.0, cv::Size(), cv::Scalar());
    net_.setInput(blob);
    return result_from_logits(net_.forward());
  } catch (const cv::Exception &) {
    return unknown_result();
  } catch (const std::exception &) {
    return unknown_result();
  }
}

bool DigitRecognizer::load_model(const std::string & model_path) noexcept
{
  net_ = cv::dnn::Net();
  if (model_path.empty()) {
    return false;
  }

  try {
    net_ = cv::dnn::readNetFromONNX(model_path);
    return !net_.empty();
  } catch (const cv::Exception &) {
    net_ = cv::dnn::Net();
    return false;
  } catch (const std::exception &) {
    net_ = cv::dnn::Net();
    return false;
  }
}

RecognitionResult DigitRecognizer::unknown_result() noexcept
{
  return {};
}

cv::Mat DigitRecognizer::preprocess_roi(const cv::Mat & input)
{
  const cv::Mat gray = to_gray(input);
  if (gray.empty() || gray.cols <= 0 || gray.rows <= 0) {
    return {};
  }

  cv::Mat canvas(kInputSize, kInputSize, CV_8UC1, cv::Scalar(0));
  const double x_scale = static_cast<double>(kInputSize) / static_cast<double>(gray.cols);
  const double y_scale = static_cast<double>(kInputSize) / static_cast<double>(gray.rows);
  const double scale = std::min(x_scale, y_scale);
  const int width = static_cast<int>(gray.cols * scale);
  const int height = static_cast<int>(gray.rows * scale);
  if (width <= 0 || height <= 0) {
    return {};
  }

  cv::resize(gray, canvas(cv::Rect(0, 0, width, height)), cv::Size(width, height));
  return canvas;
}

RecognitionResult DigitRecognizer::result_from_logits(const cv::Mat & logits)
{
  cv::Mat flat = logits.reshape(1, 1);
  if (flat.empty() || flat.total() != kExpectedClassCount || flat.depth() != CV_32F) {
    return unknown_result();
  }

  float max_logit = -std::numeric_limits<float>::infinity();
  for (int i = 0; i < flat.cols; ++i) {
    max_logit = std::max(max_logit, flat.at<float>(0, i));
  }
  if (!std::isfinite(max_logit)) {
    return unknown_result();
  }

  std::array<double, kExpectedClassCount> probabilities{};
  double sum = 0.0;
  for (int i = 0; i < flat.cols; ++i) {
    const double value = std::exp(static_cast<double>(flat.at<float>(0, i) - max_logit));
    probabilities[static_cast<std::size_t>(i)] = value;
    sum += value;
  }
  if (sum <= 0.0 || !std::isfinite(sum)) {
    return unknown_result();
  }

  const auto best = std::max_element(probabilities.begin(), probabilities.end());
  const int class_id = static_cast<int>(std::distance(probabilities.begin(), best));
  if (class_id < 0 || class_id >= kKnownDigitClasses) {
    return unknown_result();
  }

  RecognitionResult result;
  result.digit = class_id + 1;
  result.label = kDigitLabels[static_cast<std::size_t>(class_id)];
  result.confidence = *best / sum;
  result.reliable = true;
  return result;
}

}  // namespace rm_assessment
