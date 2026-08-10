#include "src/armor.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace rm_assessment
{

namespace
{
double normalized_right_angle_error(double angle)
{
  return std::abs(std::remainder(angle - CV_PI / 2.0, CV_PI));
}
}  // namespace

std::string to_string(const ArmorColor color)
{
  switch (color) {
    case ArmorColor::red:
      return "red";
    case ArmorColor::blue:
      return "blue";
    case ArmorColor::unknown:
      return "unknown";
  }
  return "unknown";
}

Lightbar::Lightbar(const cv::RotatedRect & rect, const std::size_t lightbar_id)
: id(lightbar_id), rotated_rect(rect)
{
  std::array<cv::Point2f, 4> corners{};
  rect.points(corners.data());
  std::sort(corners.begin(), corners.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.y < b.y;
  });

  center = rect.center;
  top = (corners[0] + corners[1]) * 0.5F;
  bottom = (corners[2] + corners[3]) * 0.5F;
  top_to_bottom = bottom - top;
  points = {top, bottom};

  width = cv::norm(corners[0] - corners[1]);
  length = cv::norm(top_to_bottom);
  if (width > 1e-6) {
    ratio = length / width;
  }
  angle = std::atan2(static_cast<double>(top_to_bottom.y), static_cast<double>(top_to_bottom.x));
  angle_error = normalized_right_angle_error(angle);
}

ArmorCandidate::ArmorCandidate(const Lightbar & left_lightbar, const Lightbar & right_lightbar)
: color(left_lightbar.color), left(left_lightbar), right(right_lightbar)
{
  center = (left.center + right.center) * 0.5F;
  corners = {left.top, right.top, right.bottom, left.bottom};

  const cv::Point2f left_to_right = right.center - left.center;
  const double center_distance = cv::norm(left_to_right);
  const double max_length = std::max(left.length, right.length);
  const double min_length = std::min(left.length, right.length);

  if (max_length > 1e-6) {
    ratio = center_distance / max_length;
  }
  if (min_length > 1e-6) {
    side_ratio = max_length / min_length;
  }

  const double roll = std::atan2(static_cast<double>(left_to_right.y), static_cast<double>(left_to_right.x));
  const double left_error = normalized_right_angle_error(left.angle - roll);
  const double right_error = normalized_right_angle_error(right.angle - roll);
  rectangular_error = std::max(left_error, right_error);
}

}  // namespace rm_assessment
