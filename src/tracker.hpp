/**
 * @file tracker.hpp
 * @brief 装甲板目标追踪器类声明（卡尔曼滤波 + 状态机）
 *
 * 【功能】
 * 声明基于卡尔曼滤波的装甲板目标追踪器 Tracker：对检测器输出的
 * 装甲候选序列进行持续跟踪，通过状态机与滤波平滑输出稳定的目标
 * 位置/尺寸/颜色，并在短暂丢失时用预测结果维持输出。
 *  - State 状态机：lost（丢失）→ detecting（检测中）→ tracking（跟踪中），
 *    跟踪中丢失目标进入 temp_lost（临时丢失），超限后回到 lost；
 *  - Params 封装追踪参数：最小连续检测帧数、最大临时丢失帧数、
 *    最大匹配距离、颜色失配惩罚、尺寸权重、最小框尺寸。
 *
 * 【方法】
 *  - update(candidates, frame_size)：每帧接收检测候选，驱动状态机更新；
 *  - reset：重置追踪器到初始状态；
 *  - state_value / state / has_target：状态查询；
 *  - tracked_box / tracked_center / tracked_size / tracked_color：
 *    当前跟踪目标的框、中心、尺寸与颜色查询；
 *  - bootstrap：用种子候选初始化卡尔曼滤波器与跟踪信息；
 *  - apply_prediction / apply_measurement：执行卡尔曼预测与测量修正；
 *  - choose_seed_candidate / choose_match_candidate：初始种子选择与
 *    每帧匹配候选选择；
 *  - candidate_score / color_compatible：候选匹配评分与颜色兼容判断。
 *
 * 【实现方式】
 *  - 8 维状态（中心 x/y、速度 vx/vy、宽高 w/h、宽高变化率 vw/vh）
 *    恒速模型卡尔曼滤波，4 维测量（x, y, w, h）；
 *  - 状态机：连续检测到 min_detect_count 帧才进入 tracking；
 *    丢失后进入 temp_lost 并以预测框持续输出，超过 max_temp_lost_count
 *    帧则回到 lost。实现见 tracker.cpp。
 */
#ifndef RM_ASSESSMENT_SRC_TRACKER_HPP_
#define RM_ASSESSMENT_SRC_TRACKER_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

#include "src/armor.hpp"

namespace rm_assessment
{

class Tracker
{
public:
  enum class State
  {
    lost,
    detecting,
    tracking,
    temp_lost,
  };

  struct Params
  {
    int min_detect_count = 2;
    int max_temp_lost_count = 8;
    double max_match_distance = 80.0;
    double color_mismatch_penalty = 250.0;
    double size_weight = 0.08;
    cv::Size2f min_box_size{10.0F, 10.0F};
  };

  Tracker();
  explicit Tracker(const Params & params);

  void reset();
  bool update(const std::vector<ArmorCandidate> & candidates, const cv::Size & frame_size = {});

  State state_value() const noexcept;
  std::string state() const;
  bool has_target() const noexcept;

  const cv::Rect2f & tracked_box() const noexcept;
  cv::Point2f tracked_center() const noexcept;
  cv::Size2f tracked_size() const noexcept;
  ArmorColor tracked_color() const noexcept;

private:
  Params params_{};
  State state_ = State::lost;
  bool initialized_ = false;
  int detect_streak_ = 0;
  int temp_lost_streak_ = 0;

  cv::KalmanFilter kalman_;
  cv::Mat measurement_;
  cv::Rect2f tracked_box_{};
  cv::Point2f tracked_center_{};
  cv::Size2f tracked_size_{};
  ArmorColor tracked_color_ = ArmorColor::unknown;
  ArmorCandidate tracked_candidate_{};

  void configure_filter();
  void bootstrap(const ArmorCandidate & candidate);
  void apply_prediction();
  void apply_measurement(const ArmorCandidate & candidate);

  const ArmorCandidate * choose_seed_candidate(
    const std::vector<ArmorCandidate> & candidates, const cv::Size & frame_size) const;
  const ArmorCandidate * choose_match_candidate(const std::vector<ArmorCandidate> & candidates) const;

  double candidate_score(
    const ArmorCandidate & candidate, const cv::Point2f & reference_center,
    const cv::Size2f & reference_size) const;
  bool color_compatible(ArmorColor candidate_color) const noexcept;

  static cv::Point2f candidate_center(const ArmorCandidate & candidate) noexcept;
  static cv::Size2f candidate_size(const ArmorCandidate & candidate) noexcept;
  static cv::Rect2f build_box(const cv::Point2f & center, const cv::Size2f & size) noexcept;
  static double box_area(const cv::Size2f & size) noexcept;
  static std::string state_to_string(State state);
};

}  // namespace rm_assessment

#endif  // RM_ASSESSMENT_SRC_TRACKER_HPP_
