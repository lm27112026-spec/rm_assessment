/**
 * @file tracker.cpp
 * @brief 装甲板目标追踪器的实现（卡尔曼滤波 + 状态机）
 *
 * 【功能】
 * 实现 Tracker 的完整追踪逻辑：结合 8 维恒速模型卡尔曼滤波与
 * lost/detecting/tracking/temp_lost 状态机，对检测候选进行初始化、
 * 匹配与持续跟踪，输出平滑的目标框/中心/尺寸/颜色。
 *
 * 【方法】
 *  - configure_filter：初始化卡尔曼滤波器——8 维状态、4 维测量、
 *    恒速转移矩阵（位置+速度、尺寸+尺寸变化率）、测量矩阵
 *    （观测 x/y/w/h），并设置过程/测量噪声协方差；
 *  - update：状态机主逻辑，按“无候选 / 未初始化 / lost / detecting /
 *    tracking / temp_lost”各分支处理；
 *  - bootstrap：用种子候选填充卡尔曼状态（位置、宽高，速度置零）
 *    并初始化跟踪信息，进入 detecting 状态；
 *  - apply_prediction：调用 kalman.predict() 更新预测框，用于临时丢失期输出；
 *  - apply_measurement：调用 kalman.correct() 用匹配候选修正状态并更新跟踪信息；
 *  - choose_seed_candidate：无目标时按“靠近画面中心（或最大面积）、
 *    已知颜色减分”的评分选取初始候选；
 *  - choose_match_candidate：对颜色兼容的候选按 candidate_score 取最低分，
 *    且分数须小于门限（max_match_distance + 0.25×√目标面积）才接受；
 *  - candidate_score：中心距离 + 尺寸差异×权重 + 颜色失配惩罚。
 *
 * 【实现方式】
 *  - 恒速模型：状态 [x, y, vx, vy, w, h, vw, vh]，测量 [x, y, w, h]；
 *  - 状态机：detecting 状态连续匹配 min_detect_count（默认 2）帧
 *    才升级为 tracking；tracking/temp_lost 中丢帧进入 temp_lost，
 *    累计超过 max_temp_lost_count（默认 8）帧则 reset 回到 lost；
 *  - 尺寸取 max(测量值, min_box_size) 防止退化；
 *  - 颜色兼容：跟踪颜色未知、候选颜色未知或二者相同均视为兼容。
 */
#include "src/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rm_assessment
{

namespace
{
constexpr int kStateDim = 8;
constexpr int kMeasureDim = 4;

float positive_dimension(float value, float minimum)
{
  return std::max(value, minimum);
}

}  // namespace

Tracker::Tracker() : Tracker(Params{}) {}

Tracker::Tracker(const Params & params) : params_{params}, measurement_{cv::Mat::zeros(kMeasureDim, 1, CV_32F)}
{
  configure_filter();
  reset();
}

void Tracker::configure_filter()
{
  kalman_.init(kStateDim, kMeasureDim, 0, CV_32F);
  kalman_.transitionMatrix = (cv::Mat_<float>(kStateDim, kStateDim) <<
                                1.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
                                0.0F, 1.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
                                0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
                                0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
                                0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.0F,
                                0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 1.0F,
                                0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
                                0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F);
  kalman_.measurementMatrix = (cv::Mat_<float>(kMeasureDim, kStateDim) <<
                                 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
                                 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
                                 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F,
                                 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F);

  cv::setIdentity(kalman_.processNoiseCov, cv::Scalar::all(1e-2));
  cv::setIdentity(kalman_.measurementNoiseCov, cv::Scalar::all(1e-1));
  cv::setIdentity(kalman_.errorCovPost, cv::Scalar::all(1.0));
  kalman_.statePost = cv::Mat::zeros(kStateDim, 1, CV_32F);
  kalman_.statePre = cv::Mat::zeros(kStateDim, 1, CV_32F);
}

void Tracker::reset()
{
  state_ = State::lost;
  initialized_ = false;
  detect_streak_ = 0;
  temp_lost_streak_ = 0;
  tracked_box_ = {};
  tracked_center_ = {};
  tracked_size_ = {};
  tracked_color_ = ArmorColor::unknown;
  tracked_candidate_ = {};
  cv::setIdentity(kalman_.errorCovPost, cv::Scalar::all(1.0));
  kalman_.statePost = cv::Mat::zeros(kStateDim, 1, CV_32F);
  kalman_.statePre = cv::Mat::zeros(kStateDim, 1, CV_32F);
}

bool Tracker::update(const std::vector<ArmorCandidate> & candidates, const cv::Size & frame_size)
{
  if (candidates.empty()) {
    if (!initialized_) {
      reset();
      return false;
    }

    if (state_ == State::detecting) {
      reset();
      return false;
    }

    apply_prediction();

    if (state_ == State::tracking) {
      state_ = State::temp_lost;
      temp_lost_streak_ = 1;
      return true;
    }

    if (state_ == State::temp_lost) {
      ++temp_lost_streak_;
      if (temp_lost_streak_ > params_.max_temp_lost_count) {
        reset();
        return false;
      }
      return true;
    }

    return false;
  }

  if (!initialized_ || state_ == State::lost) {
    const auto * seed = choose_seed_candidate(candidates, frame_size);
    if (seed == nullptr) {
      reset();
      return false;
    }

    bootstrap(*seed);
    state_ = State::detecting;
    return true;
  }

  apply_prediction();
  const auto * match = choose_match_candidate(candidates);

  switch (state_) {
    case State::detecting:
      if (match != nullptr) {
        apply_measurement(*match);
        ++detect_streak_;
        if (detect_streak_ >= params_.min_detect_count) {
          state_ = State::tracking;
          temp_lost_streak_ = 0;
        }
        return true;
      }
      reset();
      return false;

    case State::tracking:
      if (match != nullptr) {
        apply_measurement(*match);
        temp_lost_streak_ = 0;
        return true;
      }

      state_ = State::temp_lost;
      temp_lost_streak_ = 1;
      return true;

    case State::temp_lost:
      if (match != nullptr) {
        apply_measurement(*match);
        state_ = State::tracking;
        temp_lost_streak_ = 0;
        return true;
      }

      ++temp_lost_streak_;
      if (temp_lost_streak_ > params_.max_temp_lost_count) {
        reset();
        return false;
      }
      return true;

    case State::lost:
    default:
      break;
  }

  return false;
}

Tracker::State Tracker::state_value() const noexcept { return state_; }

std::string Tracker::state() const { return state_to_string(state_); }

bool Tracker::has_target() const noexcept { return state_ != State::lost && initialized_; }

const cv::Rect2f & Tracker::tracked_box() const noexcept { return tracked_box_; }

cv::Point2f Tracker::tracked_center() const noexcept { return tracked_center_; }

cv::Size2f Tracker::tracked_size() const noexcept { return tracked_size_; }

ArmorColor Tracker::tracked_color() const noexcept { return tracked_color_; }

void Tracker::bootstrap(const ArmorCandidate & candidate)
{
  const auto center = candidate_center(candidate);
  const auto size = candidate_size(candidate);
  const auto width = positive_dimension(size.width, params_.min_box_size.width);
  const auto height = positive_dimension(size.height, params_.min_box_size.height);

  measurement_.at<float>(0) = center.x;
  measurement_.at<float>(1) = center.y;
  measurement_.at<float>(2) = width;
  measurement_.at<float>(3) = height;

  kalman_.statePost.at<float>(0) = center.x;
  kalman_.statePost.at<float>(1) = center.y;
  kalman_.statePost.at<float>(2) = 0.0F;
  kalman_.statePost.at<float>(3) = 0.0F;
  kalman_.statePost.at<float>(4) = width;
  kalman_.statePost.at<float>(5) = height;
  kalman_.statePost.at<float>(6) = 0.0F;
  kalman_.statePost.at<float>(7) = 0.0F;
  kalman_.statePre = kalman_.statePost.clone();

  tracked_center_ = center;
  tracked_size_ = {width, height};
  tracked_box_ = build_box(center, tracked_size_);
  tracked_color_ = candidate.color;
  tracked_candidate_ = candidate;
  initialized_ = true;
  detect_streak_ = 1;
  temp_lost_streak_ = 0;
}

void Tracker::apply_prediction()
{
  const cv::Mat prediction = kalman_.predict();

  const auto center = cv::Point2f{prediction.at<float>(0), prediction.at<float>(1)};
  const auto size = cv::Size2f{
    positive_dimension(prediction.at<float>(4), params_.min_box_size.width),
    positive_dimension(prediction.at<float>(5), params_.min_box_size.height)};

  tracked_center_ = center;
  tracked_size_ = size;
  tracked_box_ = build_box(center, size);
}

void Tracker::apply_measurement(const ArmorCandidate & candidate)
{
  const auto center = candidate_center(candidate);
  const auto size = candidate_size(candidate);
  const auto width = positive_dimension(size.width, params_.min_box_size.width);
  const auto height = positive_dimension(size.height, params_.min_box_size.height);

  measurement_.at<float>(0) = center.x;
  measurement_.at<float>(1) = center.y;
  measurement_.at<float>(2) = width;
  measurement_.at<float>(3) = height;

  const cv::Mat corrected = kalman_.correct(measurement_);
  tracked_center_ = {corrected.at<float>(0), corrected.at<float>(1)};
  tracked_size_ = {
    positive_dimension(corrected.at<float>(4), params_.min_box_size.width),
    positive_dimension(corrected.at<float>(5), params_.min_box_size.height)};
  tracked_box_ = build_box(tracked_center_, tracked_size_);
  tracked_color_ = candidate.color;
  tracked_candidate_ = candidate;
}

const ArmorCandidate * Tracker::choose_seed_candidate(
  const std::vector<ArmorCandidate> & candidates, const cv::Size & frame_size) const
{
  if (candidates.empty()) return nullptr;

  const bool use_frame_center = frame_size.width > 0 && frame_size.height > 0;
  const cv::Point2f reference_center{
    use_frame_center ? static_cast<float>(frame_size.width) * 0.5F : 0.0F,
    use_frame_center ? static_cast<float>(frame_size.height) * 0.5F : 0.0F};

  const ArmorCandidate * best = &candidates.front();
  double best_score = std::numeric_limits<double>::infinity();

  for (const auto & candidate : candidates) {
    double score = 0.0;
    if (use_frame_center) {
      score = cv::norm(candidate_center(candidate) - reference_center);
    } else {
      score = -box_area(candidate_size(candidate));
    }

    if (candidate.color != ArmorColor::unknown) {
      score -= 5.0;
    }

    if (score < best_score) {
      best_score = score;
      best = &candidate;
    }
  }

  return best;
}

const ArmorCandidate * Tracker::choose_match_candidate(const std::vector<ArmorCandidate> & candidates) const
{
  if (candidates.empty()) return nullptr;

  const auto reference_center = tracked_center_;
  const auto reference_size = tracked_size_;

  const ArmorCandidate * best = nullptr;
  double best_score = std::numeric_limits<double>::infinity();

  for (const auto & candidate : candidates) {
    if (!color_compatible(candidate.color)) continue;

    const double score = candidate_score(candidate, reference_center, reference_size);
    if (score < best_score) {
      best_score = score;
      best = &candidate;
    }
  }

  const double gate = params_.max_match_distance + 0.25 * std::sqrt(box_area(reference_size));
  if (best != nullptr && best_score <= gate) return best;
  return nullptr;
}

double Tracker::candidate_score(
  const ArmorCandidate & candidate, const cv::Point2f & reference_center,
  const cv::Size2f & reference_size) const
{
  const auto center = candidate_center(candidate);
  const auto size = candidate_size(candidate);

  const double center_distance = cv::norm(center - reference_center);
  const double size_delta = std::hypot(
    static_cast<double>(size.width - reference_size.width),
    static_cast<double>(size.height - reference_size.height));
  double score = center_distance + params_.size_weight * size_delta;

  if (tracked_color_ != ArmorColor::unknown && candidate.color != ArmorColor::unknown &&
      candidate.color != tracked_color_) {
    score += params_.color_mismatch_penalty;
  }

  return score;
}

bool Tracker::color_compatible(ArmorColor candidate_color) const noexcept
{
  return tracked_color_ == ArmorColor::unknown || candidate_color == ArmorColor::unknown ||
         candidate_color == tracked_color_;
}

cv::Point2f Tracker::candidate_center(const ArmorCandidate & candidate) noexcept
{
  return candidate.center;
}

cv::Size2f Tracker::candidate_size(const ArmorCandidate & candidate) noexcept
{
  const auto width = static_cast<float>(std::max(candidate.bounding_box.width, 0));
  const auto height = static_cast<float>(std::max(candidate.bounding_box.height, 0));
  return {width, height};
}

cv::Rect2f Tracker::build_box(const cv::Point2f & center, const cv::Size2f & size) noexcept
{
  return {
    center.x - size.width * 0.5F, center.y - size.height * 0.5F, size.width, size.height};
}

double Tracker::box_area(const cv::Size2f & size) noexcept
{
  return std::max(0.0F, size.width) * std::max(0.0F, size.height);
}

std::string Tracker::state_to_string(State state)
{
  switch (state) {
    case State::lost:
      return "lost";
    case State::detecting:
      return "detecting";
    case State::tracking:
      return "tracking";
    case State::temp_lost:
      return "temp_lost";
  }

  return "lost";
}

}  // namespace rm_assessment
