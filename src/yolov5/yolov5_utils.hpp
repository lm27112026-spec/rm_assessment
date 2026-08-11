/**
 * @file yolov5_utils.hpp
 * @brief Reusable YOLOv5 preprocessing and postprocessing helpers.
 */
#ifndef RM_ASSESSMENT_SRC_YOLOV5_YOLOV5_UTILS_HPP_
#define RM_ASSESSMENT_SRC_YOLOV5_YOLOV5_UTILS_HPP_

#include <array>
#include <cmath>
#include <vector>

#include <opencv2/core.hpp>

namespace rm_assessment
{
namespace yolov5
{

struct Detection
{
  cv::Rect2f box;
  float confidence = 0.0F;
  int class_id = 0;
  std::array<cv::Point2f, 4> corners{};
  int color_id = 0;
  int digit_id = 0;
};

inline constexpr int kScoreColumn = 8;
inline constexpr int kColorStartCol = 9;
inline constexpr int kColorCount = 4;
inline constexpr int kDigitStartCol = 13;
inline constexpr int kDigitCount = 9;
inline constexpr int kOutputCols = 22;

inline float sigmoid(float x)
{
  if (x > 0.0F)
    return 1.0F / (1.0F + std::exp(-x));
  else
    return std::exp(x) / (1.0F + std::exp(x));
}

cv::Mat letterbox(
  const cv::Mat & src, const cv::Size & target_size, cv::Size & scale, cv::Point & pad);

std::vector<Detection> nms(const std::vector<Detection> & detections, float iou_threshold);

std::vector<Detection> decode_outputs(
  const cv::Mat & output,
  float confidence_threshold,
  const cv::Size & input_size,
  const cv::Size & original_size,
  const cv::Size & scale,
  const cv::Point & pad);

}  // namespace yolov5
}  // namespace rm_assessment

#endif  // RM_ASSESSMENT_SRC_YOLOV5_YOLOV5_UTILS_HPP_
