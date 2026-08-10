#include "src/detector.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>

#include <opencv2/imgproc.hpp>

namespace rm_assessment
{

namespace
{
double deg_to_rad(const double degree)
{
  return degree * CV_PI / 180.0;
}

cv::Rect image_rect(const cv::Mat & image)
{
  return {0, 0, image.cols, image.rows};
}

cv::Rect safe_bounding_rect(const std::array<cv::Point2f, 4> & corners, const cv::Size & image_size)
{
  std::vector<cv::Point2f> points(corners.begin(), corners.end());
  cv::Rect rect = cv::boundingRect(points);
  rect &= cv::Rect(0, 0, image_size.width, image_size.height);
  return rect;
}

double polygon_area(const std::array<cv::Point2f, 4> & corners)
{
  std::vector<cv::Point2f> points(corners.begin(), corners.end());
  return std::abs(cv::contourArea(points));
}

double intersection_over_min_area(const ArmorCandidate & first, const ArmorCandidate & second)
{
  const cv::Rect intersection = first.bounding_box & second.bounding_box;
  if (intersection.empty()) {
    return 0.0;
  }
  const double intersection_area = static_cast<double>(intersection.area());
  const double min_area = static_cast<double>(std::min(first.bounding_box.area(), second.bounding_box.area()));
  if (min_area <= 0.0) {
    return 0.0;
  }
  return intersection_area / min_area;
}
}  // namespace

ArmorDetector::ArmorDetector() = default;

ArmorDetector::ArmorDetector(const Params & params) : params_(params) {}

const ArmorDetector::Params & ArmorDetector::params() const
{
  return params_;
}

void ArmorDetector::set_params(const Params & params)
{
  params_ = params;
}

std::vector<ArmorCandidate> ArmorDetector::detect(const cv::Mat & bgr_image) const
{
  if (bgr_image.empty() || bgr_image.cols <= 0 || bgr_image.rows <= 0 || bgr_image.channels() != 3) {
    return {};
  }

  const cv::Mat binary_mask = build_binary_mask(bgr_image);
  if (binary_mask.empty()) {
    return {};
  }

  std::vector<Lightbar> lightbars = find_lightbars(bgr_image, binary_mask);
  std::sort(lightbars.begin(), lightbars.end(), [](const Lightbar & a, const Lightbar & b) {
    return a.center.x < b.center.x;
  });

  std::vector<ArmorCandidate> armors = pair_lightbars(bgr_image, lightbars);
  remove_duplicates(armors);
  armors.erase(
    std::remove_if(armors.begin(), armors.end(), [](const ArmorCandidate & armor) {
      return armor.duplicated;
    }),
    armors.end());
  return armors;
}

cv::Mat ArmorDetector::build_binary_mask(const cv::Mat & bgr_image) const
{
  cv::Mat channels[3];
  cv::split(bgr_image, channels);

  cv::Mat red_minus_blue;
  cv::Mat blue_minus_red;
  cv::subtract(channels[2], channels[0], red_minus_blue);
  cv::subtract(channels[0], channels[2], blue_minus_red);

  cv::Mat red_mask;
  cv::Mat blue_mask;
  cv::threshold(
    red_minus_blue, red_mask, params_.color_difference_threshold, 255, cv::THRESH_BINARY);
  cv::threshold(
    blue_minus_red, blue_mask, params_.color_difference_threshold, 255, cv::THRESH_BINARY);

  cv::Mat color_mask;
  cv::bitwise_or(red_mask, blue_mask, color_mask);

  cv::Mat gray;
  cv::cvtColor(bgr_image, gray, cv::COLOR_BGR2GRAY);
  cv::Mat bright_mask;
  cv::threshold(gray, bright_mask, params_.brightness_threshold, 255, cv::THRESH_BINARY);

  cv::Mat binary_mask;
  cv::bitwise_and(color_mask, bright_mask, binary_mask);

  const int kernel_size = std::max(1, params_.morph_kernel_size | 1);
  const int iterations = std::max(0, params_.morph_iterations);
  if (iterations > 0) {
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {kernel_size, kernel_size});
    cv::morphologyEx(binary_mask, binary_mask, cv::MORPH_CLOSE, kernel, {-1, -1}, iterations);
    cv::morphologyEx(binary_mask, binary_mask, cv::MORPH_OPEN, kernel, {-1, -1}, iterations);
  }
  return binary_mask;
}

std::vector<Lightbar> ArmorDetector::find_lightbars(
  const cv::Mat & bgr_image, const cv::Mat & binary_mask) const
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<Lightbar> lightbars;
  lightbars.reserve(contours.size());
  std::size_t id = 0U;
  const cv::Rect bounds = image_rect(bgr_image);

  for (const std::vector<cv::Point> & contour : contours) {
    if (contour.size() < 2U || cv::contourArea(contour) < params_.min_contour_area) {
      continue;
    }

    const cv::RotatedRect rect = cv::minAreaRect(contour);
    Lightbar lightbar(rect, id);
    if (!check_lightbar_geometry(lightbar)) {
      continue;
    }

    const cv::Rect bar_box = rect.boundingRect() & bounds;
    if (bar_box.empty()) {
      continue;
    }

    lightbar.color = get_color(bgr_image, contour);
    if (lightbar.color == ArmorColor::unknown) {
      continue;
    }

    lightbars.push_back(lightbar);
    ++id;
  }

  return lightbars;
}

std::vector<ArmorCandidate> ArmorDetector::pair_lightbars(
  const cv::Mat & bgr_image, const std::vector<Lightbar> & lightbars) const
{
  std::vector<ArmorCandidate> armors;
  for (auto left = lightbars.begin(); left != lightbars.end(); ++left) {
    for (auto right = std::next(left); right != lightbars.end(); ++right) {
      if (left->color != right->color) {
        continue;
      }

      const double max_length = std::max(left->length, right->length);
      const double horizontal_gap = static_cast<double>(right->center.x - left->center.x);
      if (max_length <= 1e-6 || horizontal_gap <= max_length * params_.min_lightbar_gap_ratio ||
        horizontal_gap >= max_length * params_.max_lightbar_gap_ratio) {
        continue;
      }

      const double vertical_delta = std::abs(static_cast<double>(left->center.y - right->center.y));
      if (vertical_delta > max_length * params_.max_vertical_center_delta_ratio) {
        continue;
      }

      ArmorCandidate armor(*left, *right);
      if (!check_armor_geometry(armor) || !fill_image_products(bgr_image, armor)) {
        continue;
      }
      armors.push_back(armor);
    }
  }
  return armors;
}

bool ArmorDetector::check_lightbar_geometry(const Lightbar & lightbar) const
{
  const bool has_size = lightbar.length >= params_.min_lightbar_length && lightbar.width > 1e-6;
  const bool angle_ok = lightbar.angle_error <= deg_to_rad(params_.max_lightbar_angle_error_deg);
  const bool ratio_ok = lightbar.ratio >= params_.min_lightbar_ratio &&
    lightbar.ratio <= params_.max_lightbar_ratio;
  return has_size && angle_ok && ratio_ok;
}

bool ArmorDetector::check_armor_geometry(const ArmorCandidate & armor) const
{
  const bool ratio_ok = armor.ratio >= params_.min_armor_ratio && armor.ratio <= params_.max_armor_ratio;
  const bool side_ratio_ok = armor.side_ratio <= params_.max_side_ratio;
  const bool rectangular_ok = armor.rectangular_error <= deg_to_rad(params_.max_rectangular_error_deg);
  return ratio_ok && side_ratio_ok && rectangular_ok;
}

ArmorColor ArmorDetector::get_color(
  const cv::Mat & bgr_image, const std::vector<cv::Point> & contour) const
{
  double red_sum = 0.0;
  double blue_sum = 0.0;
  int valid_points = 0;
  const cv::Rect bounds = image_rect(bgr_image);

  for (const cv::Point & point : contour) {
    if (!bounds.contains(point)) {
      continue;
    }
    const cv::Vec3b pixel = bgr_image.at<cv::Vec3b>(point);
    blue_sum += static_cast<double>(pixel[0]);
    red_sum += static_cast<double>(pixel[2]);
    ++valid_points;
  }

  if (valid_points == 0) {
    return ArmorColor::unknown;
  }
  if (red_sum > blue_sum + params_.color_difference_threshold * valid_points * 0.25) {
    return ArmorColor::red;
  }
  if (blue_sum > red_sum + params_.color_difference_threshold * valid_points * 0.25) {
    return ArmorColor::blue;
  }
  return ArmorColor::unknown;
}

bool ArmorDetector::fill_image_products(const cv::Mat & bgr_image, ArmorCandidate & armor) const
{
  const cv::Point2f left_extension = armor.left.top_to_bottom * 0.625F;
  const cv::Point2f right_extension = armor.right.top_to_bottom * 0.625F;
  armor.corners = {
    armor.left.center - left_extension, armor.right.center - right_extension,
    armor.right.center + right_extension, armor.left.center + left_extension};
  armor.center = std::accumulate(
                   armor.corners.begin(), armor.corners.end(), cv::Point2f(0.0F, 0.0F)) *
    0.25F;

  if (polygon_area(armor.corners) < params_.min_contour_area) {
    return false;
  }

  armor.bounding_box = safe_bounding_rect(armor.corners, bgr_image.size());
  if (armor.bounding_box.empty()) {
    return false;
  }

  const cv::Size roi_size(
    std::max(1, params_.normalized_roi_size.width), std::max(1, params_.normalized_roi_size.height));
  const std::array<cv::Point2f, 4> dst = {
    cv::Point2f(0.0F, 0.0F), cv::Point2f(static_cast<float>(roi_size.width - 1), 0.0F),
    cv::Point2f(static_cast<float>(roi_size.width - 1), static_cast<float>(roi_size.height - 1)),
    cv::Point2f(0.0F, static_cast<float>(roi_size.height - 1))};

  const cv::Mat transform = cv::getPerspectiveTransform(armor.corners.data(), dst.data());
  if (transform.empty()) {
    return false;
  }
  cv::warpPerspective(
    bgr_image, armor.normalized_roi, transform, roi_size, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
  return !armor.normalized_roi.empty();
}

void ArmorDetector::remove_duplicates(std::vector<ArmorCandidate> & armors) const
{
  for (auto first = armors.begin(); first != armors.end(); ++first) {
    for (auto second = std::next(first); second != armors.end(); ++second) {
      const bool share_lightbar = first->left.id == second->left.id || first->left.id == second->right.id ||
        first->right.id == second->left.id || first->right.id == second->right.id;
      const bool overlap = intersection_over_min_area(*first, *second) > 0.6;
      if (!share_lightbar && !overlap) {
        continue;
      }

      const double first_score = first->bounding_box.area() / (1.0 + first->rectangular_error + first->side_ratio);
      const double second_score = second->bounding_box.area() / (1.0 + second->rectangular_error + second->side_ratio);
      if (first_score >= second_score) {
        second->duplicated = true;
      } else {
        first->duplicated = true;
      }
    }
  }
}

}  // namespace rm_assessment
