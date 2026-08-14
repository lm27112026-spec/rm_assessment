#ifndef AUTO_AIM__DETECTOR_HPP
#define AUTO_AIM__DETECTOR_HPP

#include <list>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include "armor.hpp"

namespace auto_aim
{
class Detector
{
public:
  Detector();
  explicit Detector(const std::string & model_path);

  std::list<Armor> detect(const cv::Mat & bgr_img);

  bool has_model() const noexcept;

private:
  bool check_geometry(const Lightbar & lightbar);
  bool check_geometry(const Armor & armor);
  bool check_name(const Armor & armor);

  Color get_color(const cv::Mat & bgr_img, const std::vector<cv::Point> & contour);
  cv::Mat get_pattern(const cv::Mat & bgr_img, const Armor & armor);

  void classify(Armor & armor);

  cv::dnn::Net net_;
  bool has_model_ = false;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__DETECTOR_HPP
