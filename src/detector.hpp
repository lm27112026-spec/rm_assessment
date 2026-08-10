/**
 * @file detector.hpp
 * @brief 装甲板检测器类声明
 *
 * 【功能】
 * 声明装甲板检测器 ArmorDetector，负责从一帧 BGR 图像中检测出所有装甲板候选
 * （ArmorCandidate 列表）。检测流程为：构建二值化掩码 → 提取灯条 →
 * 灯条两两配对 → 几何校验 → 生成归一化 ROI → 去除重复候选。
 * 内部 Params 结构体集中封装了全部可调参数：
 *  - 预处理参数：亮度阈值、颜色差阈值、形态学核大小与迭代次数、最小轮廓面积；
 *  - 灯条几何约束：最大角度误差、长宽比范围、最小长度；
 *  - 装甲几何约束：长宽比范围、最大侧边比、最大矩形度误差、
 *    垂直中心差比例、灯条间距比例范围；
 *  - 归一化 ROI 尺寸。
 *
 * 【方法】
 *  - detect(bgr_image)：检测主入口，返回装甲板候选列表；
 *  - build_binary_mask：构建“颜色差 + 亮度”联合二值化掩码；
 *  - find_lightbars：从二值化掩码中提取并通过几何/颜色筛选灯条；
 *  - pair_lightbars：将同色灯条两两配对，校验几何后生成装甲候选；
 *  - check_lightbar_geometry / check_armor_geometry：灯条/装甲几何约束校验；
 *  - get_color：统计轮廓内红蓝通道差异判定灯条颜色；
 *  - fill_image_products：计算候选角点、包围盒与透视归一化 ROI；
 *  - remove_duplicates：去除共享灯条或大面积重叠的重复候选。
 *
 * 【实现方式】
 *  - 典型流水线：二值化 → findContours 轮廓检测 → 灯条过滤 →
 *    配对与几何过滤 → 去重，步骤间通过私有成员函数解耦；
 *  - detect 为 const 方法，检测过程不修改检测器状态，可被多线程安全调用；
 *  - 所有阈值均从 Params 读取，便于集中调参，实现见 detector.cpp。
 */
#ifndef RM_ASSESSMENT_SRC_DETECTOR_HPP_
#define RM_ASSESSMENT_SRC_DETECTOR_HPP_

#include <vector>

#include <opencv2/core.hpp>

#include "src/armor.hpp"

namespace rm_assessment
{

class ArmorDetector
{
public:
  struct Params
  {
    int brightness_threshold = 150;
    int color_difference_threshold = 50;
    int morph_kernel_size = 3;
    int morph_iterations = 1;
    double min_contour_area = 8.0;

    double max_lightbar_angle_error_deg = 35.0;
    double min_lightbar_ratio = 1.5;
    double max_lightbar_ratio = 25.0;
    double min_lightbar_length = 6.0;

    double min_armor_ratio = 0.8;
    double max_armor_ratio = 5.5;
    double max_side_ratio = 2.0;
    double max_rectangular_error_deg = 35.0;
    double max_vertical_center_delta_ratio = 0.65;
    double min_lightbar_gap_ratio = 0.35;
    double max_lightbar_gap_ratio = 6.5;

    cv::Size normalized_roi_size{120, 64};
  };

  ArmorDetector();
  explicit ArmorDetector(const Params & params);

  const Params & params() const;
  void set_params(const Params & params);

  std::vector<ArmorCandidate> detect(const cv::Mat & bgr_image) const;

private:
  Params params_{};

  cv::Mat build_binary_mask(const cv::Mat & bgr_image) const;
  std::vector<Lightbar> find_lightbars(const cv::Mat & bgr_image, const cv::Mat & binary_mask) const;
  std::vector<ArmorCandidate> pair_lightbars(
    const cv::Mat & bgr_image, const std::vector<Lightbar> & lightbars) const;

  bool check_lightbar_geometry(const Lightbar & lightbar) const;
  bool check_armor_geometry(const ArmorCandidate & armor) const;
  ArmorColor get_color(const cv::Mat & bgr_image, const std::vector<cv::Point> & contour) const;
  bool fill_image_products(const cv::Mat & bgr_image, ArmorCandidate & armor) const;
  void remove_duplicates(std::vector<ArmorCandidate> & armors) const;
};

}  // namespace rm_assessment

#endif  // RM_ASSESSMENT_SRC_DETECTOR_HPP_
