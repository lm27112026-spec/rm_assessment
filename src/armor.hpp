#ifndef RM_ASSESSMENT_SRC_ARMOR_HPP_
#define RM_ASSESSMENT_SRC_ARMOR_HPP_

#include <array>
#include <cstddef>
#include <string>

#include <opencv2/core.hpp>

namespace rm_assessment
{

enum class ArmorColor
{
  red,
  blue,
  unknown
};

std::string to_string(ArmorColor color);

struct Lightbar
{
  std::size_t id = 0U;
  ArmorColor color = ArmorColor::unknown;
  cv::Point2f center{};
  cv::Point2f top{};
  cv::Point2f bottom{};
  cv::Point2f top_to_bottom{};
  std::array<cv::Point2f, 2> points{};
  double angle = 0.0;
  double angle_error = 0.0;
  double length = 0.0;
  double width = 0.0;
  double ratio = 0.0;
  cv::RotatedRect rotated_rect{};

  Lightbar() = default;
  Lightbar(const cv::RotatedRect & rect, std::size_t lightbar_id);
};

struct ArmorCandidate
{
  ArmorColor color = ArmorColor::unknown;
  Lightbar left{};
  Lightbar right{};
  cv::Point2f center{};
  std::array<cv::Point2f, 4> corners{};  // top-left, top-right, bottom-right, bottom-left
  cv::Rect bounding_box{};
  cv::Mat normalized_roi{};
  double ratio = 0.0;
  double side_ratio = 0.0;
  double rectangular_error = 0.0;
  bool duplicated = false;

  ArmorCandidate() = default;
  ArmorCandidate(const Lightbar & left_lightbar, const Lightbar & right_lightbar);
};

}  // namespace rm_assessment

#endif  // RM_ASSESSMENT_SRC_ARMOR_HPP_
