#ifndef RM_ASSESSMENT_SRC_DIGIT_RECOGNIZER_HPP_
#define RM_ASSESSMENT_SRC_DIGIT_RECOGNIZER_HPP_

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "src/armor.hpp"

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
  // Template directory behavior:
  // - The directory is optional. If it is empty or unreadable, the recognizer stays usable
  //   and will return Unknown instead of failing.
  // - When present, the recognizer looks for label files named 1.png ... 5.png at the top
  //   level of that directory. Common image suffixes such as .png, .jpg, .jpeg, and .bmp
  //   are supported.
  // - Missing, empty, or too-small template images are skipped safely.
  explicit DigitRecognizer(const std::string & template_directory = {});

  void set_template_directory(const std::string & template_directory);

  bool has_templates() const noexcept;

  RecognitionResult recognize(const ArmorCandidate & candidate) const;
  RecognitionResult recognize(const cv::Mat & normalized_roi) const;

private:
  struct TemplateSample
  {
    int digit{-1};
    cv::Mat binary;
    std::vector<cv::Point> contour;
  };

  std::string template_directory_;
  std::vector<TemplateSample> templates_;

  bool load_templates_from_directory(const std::string & template_directory);

  static RecognitionResult unknown_result() noexcept;
  static cv::Mat preprocess_roi(const cv::Mat & input);
  static double score_binary_image(const cv::Mat & binary);
  static bool extract_primary_contour(
    const cv::Mat & binary, std::vector<cv::Point> & contour, cv::Rect & bounding_box);
  static cv::Mat center_digit_on_canvas(const cv::Mat & binary, const cv::Rect & bounding_box);
  static double pixel_similarity(const cv::Mat & lhs, const cv::Mat & rhs);
  static double contour_similarity(const std::vector<cv::Point> & lhs, const std::vector<cv::Point> & rhs);
};

}  // namespace rm_assessment

#endif  // RM_ASSESSMENT_SRC_DIGIT_RECOGNIZER_HPP_
