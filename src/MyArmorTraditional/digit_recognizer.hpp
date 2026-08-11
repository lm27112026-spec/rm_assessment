/**
 * @file digit_recognizer.hpp
 * @brief Digit recognizer backed by an OpenCV DNN ONNX classifier.
 */
#ifndef RM_ASSESSMENT_MY_ARMOR_TRADITIONAL_DIGIT_RECOGNIZER_HPP_
#define RM_ASSESSMENT_MY_ARMOR_TRADITIONAL_DIGIT_RECOGNIZER_HPP_

#include <string>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include "armor.hpp"

namespace rm_assessment
{

struct RecognitionResult
{
  std::string label{"Unknown"};
  double confidence{0.0};
  int digit{-1};
  bool reliable{false};
};

class DigitRecognizer
{
public:
  explicit DigitRecognizer(const std::string & model_path = {});

  void set_model_path(const std::string & model_path);
  void set_template_directory(const std::string & model_path);

  bool has_model() const noexcept;
  bool has_templates() const noexcept;

  RecognitionResult recognize(const ArmorCandidate & candidate) const;
  RecognitionResult recognize(const cv::Mat & normalized_roi) const;

private:
  std::string model_path_;
  mutable cv::dnn::Net net_;
  bool model_loaded_{false};

  bool load_model(const std::string & model_path) noexcept;

  static RecognitionResult unknown_result() noexcept;
  static cv::Mat preprocess_roi(const cv::Mat & input);
  static RecognitionResult result_from_logits(const cv::Mat & logits);
};

}  // namespace rm_assessment

#endif  // RM_ASSESSMENT_MY_ARMOR_TRADITIONAL_DIGIT_RECOGNIZER_HPP_
