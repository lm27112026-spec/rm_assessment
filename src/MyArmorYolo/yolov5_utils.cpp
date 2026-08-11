#include "yolov5_utils.hpp"

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
  if (output.empty() || output.cols < kOutputCols || output.type() != CV_32F) {
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
    // Skip rows with NaN/Inf in any corner coordinate (cols 0-7).
    bool invalid_coord = false;
    for (int col = 0; col < 8; ++col) {
      const float corner_value = output.at<float>(i, col);
      if (std::isnan(corner_value) || std::isinf(corner_value)) {
        invalid_coord = true;
        break;
      }
    }
    if (invalid_coord) {
      continue;
    }

    // col 8 is the score logit; reject NaN/Inf before sigmoid and filter by confidence.
    const float raw_score = output.at<float>(i, kScoreColumn);
    if (std::isnan(raw_score) || std::isinf(raw_score)) {
      continue;
    }
    const float score = sigmoid(raw_score);
    if (score < confidence_threshold) {
      continue;
    }

    // Model emits corners as [TL, BL, BR, TR]; reorder to [TL, TR, BR, BL].
    // Map each corner from letterbox space back to original image space:
    // original = (model - pad) * original_size / scale.
    const cv::Point2f model_corners[4] = {
      {output.at<float>(i, 0), output.at<float>(i, 1)},  // TL
      {output.at<float>(i, 2), output.at<float>(i, 3)},  // BL
      {output.at<float>(i, 4), output.at<float>(i, 5)},  // BR
      {output.at<float>(i, 6), output.at<float>(i, 7)},  // TR
    };
    const int corners_order[4] = {0, 3, 2, 1};  // TL, TR, BR, BL
    Detection det;
    for (int corner = 0; corner < 4; ++corner) {
      const cv::Point2f & model_pt = model_corners[corners_order[corner]];
      det.corners[corner] = {
        (model_pt.x - static_cast<float>(pad.x)) * static_cast<float>(original_size.width) /
          static_cast<float>(scale.width),
        (model_pt.y - static_cast<float>(pad.y)) * static_cast<float>(original_size.height) /
          static_cast<float>(scale.height)};
    }

    // Axis-aligned box from the min/max of the reordered corners.
    float min_x = det.corners[0].x;
    float max_x = det.corners[0].x;
    float min_y = det.corners[0].y;
    float max_y = det.corners[0].y;
    for (int corner = 1; corner < 4; ++corner) {
      min_x = std::min(min_x, det.corners[corner].x);
      max_x = std::max(max_x, det.corners[corner].x);
      min_y = std::min(min_y, det.corners[corner].y);
      max_y = std::max(max_y, det.corners[corner].y);
    }
    const cv::Rect2f box = clamp_box(
      cv::Rect2f(min_x, min_y, max_x - min_x, max_y - min_y), original_size);
    // min/max guarantees non-negative extents; only reject impossible negatives.
    // Zero-area boxes (collapsed corners) are still valid detections.
    if (box.width < 0.0F || box.height < 0.0F) {
      continue;
    }

    // Color one-hot (cols 9-12): argmax index -> color_id (0-3).
    int color_id = 0;
    float best_color = output.at<float>(i, kColorStartCol);
    for (int col = kColorStartCol + 1; col < kColorStartCol + kColorCount; ++col) {
      const float value = output.at<float>(i, col);
      if (value > best_color) {
        best_color = value;
        color_id = col - kColorStartCol;
      }
    }

    // Digit one-hot (cols 13-21): argmax index -> digit_id (0-8).
    int digit_id = 0;
    float best_digit = output.at<float>(i, kDigitStartCol);
    for (int col = kDigitStartCol + 1; col < kDigitStartCol + kDigitCount; ++col) {
      const float value = output.at<float>(i, col);
      if (value > best_digit) {
        best_digit = value;
        digit_id = col - kDigitStartCol;
      }
    }

    det.box = box;
    det.confidence = score;
    det.color_id = color_id;
    det.digit_id = digit_id;
    detections.push_back(det);
  }

  return detections;
}

}  // namespace rm_assessment::yolov5
