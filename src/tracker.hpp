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
