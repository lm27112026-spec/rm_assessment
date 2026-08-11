#include "src/yolov5/yolov5_utils.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace rm_assessment::yolov5
{

namespace
{
cv::Rect2f clamp_box(const cv::Rect2f & box, const cv::Size & bounds)
{
  const float left = std::clamp(box.x, 0.0F, static_cast<float>(bounds.width));
  const float top = std::clamp(box.y, 0.0F, static_cast<float>(bounds.height));
  const float right = std::clamp(box.x + box.width, 0.0F, static_cast<float>(bounds.width));
  const float bottom = std::clamp(box.y + box.height, 0.0F, static_cast<float>(bounds.height));
  return {left, top, right - left, bottom - top};
}

float iou(const Detection & lhs, const Detection & rhs)
{
  const float inter_left = std::max(lhs.box.x, rhs.box.x);
  const float inter_top = std::max(lhs.box.y, rhs.box.y);
  const float inter_right = std::min(lhs.box.x + lhs.box.width, rhs.box.x + rhs.box.width);
  const float inter_bottom = std::min(lhs.box.y + lhs.box.height, rhs.box.y + rhs.box.height);

  const float inter_w = std::max(0.0F, inter_right - inter_left);
  const float inter_h = std::max(0.0F, inter_bottom - inter_top);
  const float inter_area = inter_w * inter_h;

  const float lhs_area = lhs.box.width * lhs.box.height;
  const float rhs_area = rhs.box.width * rhs.box.height;
  const float union_area = lhs_area + rhs_area - inter_area;
  if (union_area <= 0.0F) {
    return 0.0F;
  }
  return inter_area / union_area;
}
}  // namespace

cv::Mat letterbox(
  const cv::Mat & src, const cv::Size & target_size, cv::Size & scale, cv::Point & pad)
{
  if (src.empty() || target_size.width <= 0 || target_size.height <= 0) {
    scale = cv::Size();
    pad = cv::Point();
    return {};
  }

  const double ratio = std::min(
    static_cast<double>(target_size.width) / static_cast<double>(src.cols),
    static_cast<double>(target_size.height) / static_cast<double>(src.rows));

  const int resized_width = static_cast<int>(std::round(src.cols * ratio));
  const int resized_height = static_cast<int>(std::round(src.rows * ratio));
  scale = cv::Size(resized_width, resized_height);

  cv::Mat resized;
  cv::resize(src, resized, scale, 0.0, 0.0, cv::INTER_LINEAR);

  const int pad_x = (target_size.width - resized_width) / 2;
  const int pad_y = (target_size.height - resized_height) / 2;
  pad = cv::Point(pad_x, pad_y);

  cv::Mat output(target_size, src.type(), cv::Scalar::all(114));
  resized.copyTo(output(cv::Rect(pad_x, pad_y, resized_width, resized_height)));
  return output;
}

std::vector<Detection> nms(const std::vector<Detection> & detections, float iou_threshold)
{
  std::vector<Detection> sorted = detections;
  std::sort(sorted.begin(), sorted.end(), [](const Detection & a, const Detection & b) {
    return a.confidence > b.confidence;
  });

  std::vector<Detection> result;
  for (const auto & det : sorted) {
    bool suppressed = false;
    for (const auto & kept : result) {
      if (iou(det, kept) > iou_threshold) {
        suppressed = true;
        break;
      }
    }
    if (!suppressed) {
      result.push_back(det);
    }
  }
  return result;
}

std::vector<Detection> decode_outputs(
  const cv::Mat & output,
  float confidence_threshold,
  const cv::Size & input_size,
  const cv::Size & original_size,
  const cv::Size & scale,
  const cv::Point & pad)
{
  std::vector<Detection> detections;
  if (output.empty() || output.cols < 6 || output.type() != CV_32F) {
    return detections;
  }

  if (input_size.width <= 0 || input_size.height <= 0 || original_size.width <= 0 ||
    original_size.height <= 0 || scale.width <= 0 || scale.height <= 0)
  {
    return detections;
  }

  const float gain_x = static_cast<float>(scale.width) / static_cast<float>(original_size.width);
  const float gain_y = static_cast<float>(scale.height) / static_cast<float>(original_size.height);
  if (gain_x <= 0.0F || gain_y <= 0.0F) {
    return detections;
  }

  for (int i = 0; i < output.rows; ++i) {
    const float x = output.at<float>(i, 0);
    const float y = output.at<float>(i, 1);
    const float w = output.at<float>(i, 2);
    const float h = output.at<float>(i, 3);
    const float objectness = output.at<float>(i, 4);
    if (w <= 0.0F || h <= 0.0F || objectness <= 0.0F) {
      continue;
    }

    int class_id = 0;
    float best_class_probability = output.at<float>(i, 5);
    for (int class_col = 6; class_col < output.cols; ++class_col) {
      const float probability = output.at<float>(i, class_col);
      if (probability > best_class_probability) {
        best_class_probability = probability;
        class_id = class_col - 5;
      }
    }

    if (best_class_probability <= 0.0F) {
      continue;
    }

    const float confidence = objectness * best_class_probability;

    if (confidence < confidence_threshold) {
      continue;
    }

    const float left = (x - w * 0.5F - static_cast<float>(pad.x)) / gain_x;
    const float top = (y - h * 0.5F - static_cast<float>(pad.y)) / gain_y;
    const float width = w / gain_x;
    const float height = h / gain_y;
    const cv::Rect2f box = clamp_box(cv::Rect2f(left, top, width, height), original_size);
    if (box.width <= 0.0F || box.height <= 0.0F) {
      continue;
    }

    Detection det;
    det.box = box;
    det.confidence = confidence;
    det.class_id = class_id;
    detections.push_back(det);
  }

  return detections;
}

}  // namespace rm_assessment::yolov5
