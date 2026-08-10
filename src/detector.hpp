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
