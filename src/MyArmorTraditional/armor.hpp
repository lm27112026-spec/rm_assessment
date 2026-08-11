/**
 * @file armor.hpp
 * @brief 装甲板基础数据结构定义
 *
 * 【功能】
 * 定义装甲板视觉系统的基础公共类型与数据结构：
 *  - ArmorColor 枚举：表示装甲板颜色（红 / 蓝 / 未知）；
 *  - Lightbar 结构体：表示装甲板两侧的发光灯条，携带其颜色与几何信息
 *    （中心、上下端点、倾斜角、长度、宽度、长宽比、角度误差、最小外接矩形等）；
 *  - ArmorCandidate 结构体：表示由一对左右灯条构成的装甲板候选，
 *    携带中心、四角点、包围盒、归一化 ROI、灯条间距比、侧边比、
 *    矩形度误差以及去重标志等信息。
 *
 * 【方法】
 *  - to_string(ArmorColor)：将颜色枚举转换为可读字符串；
 *  - Lightbar 构造函数：由 cv::RotatedRect 与灯条 id 构造灯条并计算几何特征；
 *  - ArmorCandidate 构造函数：由左右两条灯条构造装甲板候选并计算几何特征。
 *
 * 【实现方式】
 *  - Lightbar：保存最小外接矩形，并在构造函数内通过角点排序派生
 *    上下端点、宽度（上边两端点距离）、长度（上下端点距离）、长宽比、
 *    倾斜角（atan2）与归一化角度误差；
 *  - ArmorCandidate：组合左右灯条的几何信息，计算中心中点、按
 *    左上/右上/右下/左下顺序排列的角点、灯条间距比、长宽比以及矩形度误差；
 *  - 全部几何量均在构造函数内完成，本文件仅作声明，实现见 armor.cpp。
 */
#ifndef RM_ASSESSMENT_MY_ARMOR_TRADITIONAL_ARMOR_HPP_
#define RM_ASSESSMENT_MY_ARMOR_TRADITIONAL_ARMOR_HPP_

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

#endif  // RM_ASSESSMENT_MY_ARMOR_TRADITIONAL_ARMOR_HPP_
