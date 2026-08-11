/**
 * @file armor.cpp
 * @brief 装甲板基础数据结构的实现
 *
 * 【功能】
 * 实现 armor.hpp 中声明的数据结构构造函数与辅助函数，
 * 为灯条与装甲板候选计算后续检测/识别所需的全部几何特征。
 *
 * 【方法】
 *  - normalized_right_angle_error（匿名命名空间）：计算给定角度与竖直方向
 *    （π/2）之间的归一化角度误差，用于判断灯条是否接近竖直；
 *  - to_string(ArmorColor)：颜色枚举转字符串；
 *  - Lightbar 构造函数：从最小外接矩形提取灯条几何特征；
 *  - ArmorCandidate 构造函数：从左右灯条构造装甲板候选并计算几何特征。
 *
 * 【实现方式】
 *  - Lightbar：通过 rect.points() 取出外接矩形 4 个角点并按 y 坐标排序，
 *    上半两点均值作为 top、下半两点均值作为 bottom，进而得到
 *    中心、宽度（top 两点距离）、长度（top 到 bottom 距离）、
 *    长宽比、倾斜角（atan2）与角度误差；
 *  - ArmorCandidate：center 取左右灯条中心的中点；corners 按
 *    左上/右上/右下/左下排列；ratio = 灯条中心间距 / 较长灯条长度；
 *    side_ratio = 较长 / 较短灯条长度；rectangular_error 取左右灯条
 *    与两灯条连线的角度误差中的最大值，衡量四边形是否接近矩形。
 */
#include "armor.hpp"

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
