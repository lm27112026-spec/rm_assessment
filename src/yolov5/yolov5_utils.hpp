/**
 * @file yolov5_utils.hpp
 * @brief Reusable YOLOv5 preprocessing and postprocessing helpers.
 */
#ifndef RM_ASSESSMENT_SRC_YOLOV5_YOLOV5_UTILS_HPP_
#define RM_ASSESSMENT_SRC_YOLOV5_YOLOV5_UTILS_HPP_

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
};

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
