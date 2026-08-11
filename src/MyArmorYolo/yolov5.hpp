#pragma once

#include <string>
#include <vector>

#include <openvino/openvino.hpp>
#include <opencv2/core.hpp>

#include "yolov5_utils.hpp"

namespace rm_assessment::yolov5
{

class YOLOV5Detector
{
public:
  explicit YOLOV5Detector(const std::string & model_path, std::string device = "CPU");
  std::vector<Detection> detect(const cv::Mat & image);

private:
  std::string model_path_;
  std::string device_;
  ov::Core core_;
  ov::CompiledModel compiled_model_;
  float confidence_threshold_ = 0.7F;
  float nms_threshold_ = 0.3F;
};

}  // namespace rm_assessment::yolov5
