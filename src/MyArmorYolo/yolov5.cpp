#include "yolov5.hpp"

#include <algorithm>
#include <filesystem>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace rm_assessment::yolov5
{

namespace
{
cv::Mat prepare_input(const cv::Mat & image, cv::Size & scale, cv::Point & pad)
{
  return letterbox(image, cv::Size(640, 640), scale, pad);
}
}  // namespace

YOLOV5Detector::YOLOV5Detector(const std::string & model_path, std::string device)
: model_path_(model_path), device_(std::move(device))
{
  const std::filesystem::path model_file = std::filesystem::absolute(model_path_);
  const std::filesystem::path weights_file = std::filesystem::path(model_file).replace_extension(".bin");
  if (!std::filesystem::exists(model_file)) {
    throw std::runtime_error("YOLOv5 model XML not found: " + model_file.string());
  }
  if (!std::filesystem::exists(weights_file)) {
    throw std::runtime_error("YOLOv5 model weights not found: " + weights_file.string());
  }
  const std::string model_path_string = model_file.string();
  const auto model = core_.read_model(model_path_string);
  compiled_model_ = core_.compile_model(model, device_);
}

std::vector<Detection> YOLOV5Detector::detect(const cv::Mat & image)
{
  if (image.empty()) {
    return {};
  }

  cv::Size scale;
  cv::Point pad;
  const cv::Mat resized = prepare_input(image, scale, pad);
  if (resized.empty()) {
    return {};
  }

  cv::Mat rgb_input;
  cv::cvtColor(resized, rgb_input, cv::COLOR_BGR2RGB);

  cv::Mat float_input;
  rgb_input.convertTo(float_input, CV_32F, 1.0 / 255.0);
  std::vector<cv::Mat> channels;
  cv::split(float_input, channels);

  std::vector<float> blob(static_cast<std::size_t>(3 * 640 * 640));
  for (int c = 0; c < 3; ++c) {
    const cv::Mat flat = channels[c].reshape(1, 1);
    const float * src_ptr = flat.ptr<float>();
    std::copy(src_ptr, src_ptr + 640 * 640, blob.begin() + static_cast<std::size_t>(c * 640 * 640));
  }

  ov::Tensor input_tensor(ov::element::f32, ov::Shape{1, 3, 640, 640}, blob.data());

  auto infer_request = compiled_model_.create_infer_request();
  infer_request.set_input_tensor(input_tensor);
  infer_request.infer();

  const auto output_tensor = infer_request.get_output_tensor();
  const auto output_shape = output_tensor.get_shape();
  if (output_shape.size() != 3 || output_shape[0] == 0) {
    return {};
  }

  const auto * data = output_tensor.data<float>();
  cv::Mat output(static_cast<int>(output_shape[1]), static_cast<int>(output_shape[2]), CV_32F,
    const_cast<float *>(data));

  auto decoded = decode_outputs(
    output, confidence_threshold_, cv::Size(640, 640), image.size(), scale, pad);
  decoded = nms(decoded, nms_threshold_);
  return decoded;
}

}  // namespace rm_assessment::yolov5
