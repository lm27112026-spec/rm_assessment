#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

namespace rm_assessment
{

// Lightweight constant-velocity Kalman tracker over detection boxes.
// Standalone replacement for the removed armor-module Tracker, used by the
// YOLO follow demo (题目3). Tracks the box center via a 4-state Kalman filter;
// the box size is carried from the last matched detection.
class DetectionTracker
{
public:
  enum class State
  {
    lost,
    tracking,
    temp_lost,
  };

  struct Params
  {
    double max_match_distance = 120.0;  // association gate radius in pixels
    int max_temp_lost = 8;              // frames to keep predicting after a miss
    cv::Size2f min_size{10.0F, 10.0F};
  };

  DetectionTracker() : DetectionTracker(Params{}) {}
  explicit DetectionTracker(const Params & params)
  : params_(params), measurement_(cv::Mat::zeros(2, 1, CV_32F))
  {
    configure();
  }

  // Returns true while a target is held (tracking or temp_lost).
  bool update(const std::vector<cv::Rect2f> & boxes, const cv::Size & frame_size = {})
  {
    if (boxes.empty()) {
      if (!initialized_) {
        reset();
        return false;
      }
      predict();
      if (state_ == State::tracking) {
        state_ = State::temp_lost;
        temp_lost_streak_ = 1;
        return true;
      }
      if (state_ == State::temp_lost) {
        if (++temp_lost_streak_ > params_.max_temp_lost) {
          reset();
          return false;
        }
        return true;
      }
      return false;
    }

    if (!initialized_ || state_ == State::lost) {
      const cv::Rect2f * seed = choose_seed(boxes, frame_size);
      if (seed == nullptr) {
        reset();
        return false;
      }
      bootstrap(*seed);
      state_ = State::tracking;
      return true;
    }

    predict();
    const cv::Rect2f * match = choose_match(boxes);
    if (match != nullptr) {
      correct(*match);
      state_ = State::tracking;
      temp_lost_streak_ = 0;
    } else if (state_ == State::tracking) {
      state_ = State::temp_lost;
      temp_lost_streak_ = 1;
    } else if (state_ == State::temp_lost) {
      if (++temp_lost_streak_ > params_.max_temp_lost) {
        reset();
        return false;
      }
    }
    return has_target();
  }

  State state() const noexcept { return state_; }

  std::string state_str() const
  {
    switch (state_) {
      case State::tracking:
        return "tracking";
      case State::temp_lost:
        return "temp_lost";
      case State::lost:
      default:
        return "lost";
    }
  }

  bool has_target() const noexcept { return initialized_ && state_ != State::lost; }
  const cv::Rect2f & tracked_box() const noexcept { return tracked_box_; }
  cv::Point2f tracked_center() const noexcept { return tracked_center_; }

private:
  static cv::Point2f box_center(const cv::Rect2f & box)
  {
    return {box.x + box.width * 0.5F, box.y + box.height * 0.5F};
  }

  void configure()
  {
    kalman_.init(4, 2, 0, CV_32F);
    kalman_.transitionMatrix = (cv::Mat_<float>(4, 4) <<
      1.0F, 0.0F, 1.0F, 0.0F,
      0.0F, 1.0F, 0.0F, 1.0F,
      0.0F, 0.0F, 1.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 1.0F);
    kalman_.measurementMatrix = (cv::Mat_<float>(2, 4) <<
      1.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 1.0F, 0.0F, 0.0F);
    cv::setIdentity(kalman_.processNoiseCov, cv::Scalar::all(1e-2));
    cv::setIdentity(kalman_.measurementNoiseCov, cv::Scalar::all(1e-1));
    cv::setIdentity(kalman_.errorCovPost, cv::Scalar::all(1.0));
    kalman_.statePost = cv::Mat::zeros(4, 1, CV_32F);
    kalman_.statePre = cv::Mat::zeros(4, 1, CV_32F);
  }

  void reset()
  {
    state_ = State::lost;
    initialized_ = false;
    temp_lost_streak_ = 0;
    tracked_box_ = {};
    tracked_center_ = {};
    tracked_size_ = {};
    cv::setIdentity(kalman_.errorCovPost, cv::Scalar::all(1.0));
    kalman_.statePost = cv::Mat::zeros(4, 1, CV_32F);
    kalman_.statePre = cv::Mat::zeros(4, 1, CV_32F);
  }

  void bootstrap(const cv::Rect2f & box)
  {
    const cv::Point2f center = box_center(box);
    kalman_.statePost.at<float>(0) = center.x;
    kalman_.statePost.at<float>(1) = center.y;
    kalman_.statePost.at<float>(2) = 0.0F;
    kalman_.statePost.at<float>(3) = 0.0F;
    kalman_.statePre = kalman_.statePost.clone();

    tracked_center_ = center;
    tracked_size_ = {std::max(box.width, params_.min_size.width), std::max(box.height, params_.min_size.height)};
    tracked_box_ = box;
    initialized_ = true;
    temp_lost_streak_ = 0;
  }

  void predict()
  {
    const cv::Mat pred = kalman_.predict();
    tracked_center_ = {pred.at<float>(0), pred.at<float>(1)};
    tracked_box_ = cv::Rect2f(
      tracked_center_.x - tracked_size_.width * 0.5F, tracked_center_.y - tracked_size_.height * 0.5F,
      tracked_size_.width, tracked_size_.height);
  }

  void correct(const cv::Rect2f & box)
  {
    const cv::Point2f center = box_center(box);
    measurement_.at<float>(0) = center.x;
    measurement_.at<float>(1) = center.y;
    const cv::Mat corrected = kalman_.correct(measurement_);
    tracked_center_ = {corrected.at<float>(0), corrected.at<float>(1)};
    tracked_size_ = {std::max(box.width, params_.min_size.width), std::max(box.height, params_.min_size.height)};
    tracked_box_ = cv::Rect2f(
      tracked_center_.x - tracked_size_.width * 0.5F, tracked_center_.y - tracked_size_.height * 0.5F,
      tracked_size_.width, tracked_size_.height);
  }

  const cv::Rect2f * choose_seed(const std::vector<cv::Rect2f> & boxes, const cv::Size & frame_size) const
  {
    if (boxes.empty()) {
      return nullptr;
    }
    const bool use_center = frame_size.width > 0 && frame_size.height > 0;
    const cv::Point2f ref(
      static_cast<float>(frame_size.width) * 0.5F, static_cast<float>(frame_size.height) * 0.5F);

    const cv::Rect2f * best = nullptr;
    double best_score = std::numeric_limits<double>::infinity();
    for (const auto & box : boxes) {
      const double score = use_center ? cv::norm(box_center(box) - ref) : -static_cast<double>(box.area());
      if (score < best_score) {
        best_score = score;
        best = &box;
      }
    }
    return best;
  }

  const cv::Rect2f * choose_match(const std::vector<cv::Rect2f> & boxes) const
  {
    const cv::Rect2f * best = nullptr;
    double best_dist = std::numeric_limits<double>::infinity();
    for (const auto & box : boxes) {
      const double dist = cv::norm(box_center(box) - tracked_center_);
      if (dist < best_dist) {
        best_dist = dist;
        best = &box;
      }
    }
    if (best != nullptr && best_dist <= params_.max_match_distance) {
      return best;
    }
    return nullptr;
  }

  Params params_;
  State state_ = State::lost;
  bool initialized_ = false;
  int temp_lost_streak_ = 0;

  cv::KalmanFilter kalman_;
  cv::Mat measurement_;
  cv::Rect2f tracked_box_{};
  cv::Point2f tracked_center_{};
  cv::Size2f tracked_size_{};
};

}  // namespace rm_assessment
