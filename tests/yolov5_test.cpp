#include "src/yolov5/yolov5_utils.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <filesystem>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>
#include <openvino/openvino.hpp>

namespace
{
class TestFailure : public std::runtime_error
{
public:
  explicit TestFailure(const std::string & message) : std::runtime_error(message) {}
};

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw TestFailure(message);
  }
}
}  // namespace

int main()
{
  try {
    const cv::Mat src(100, 200, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Size scale;
    cv::Point pad;
    const cv::Mat letterboxed = rm_assessment::yolov5::letterbox(src, cv::Size(640, 640), scale, pad);
    require(!letterboxed.empty(), "letterbox should return an image");
    require(letterboxed.rows == 640 && letterboxed.cols == 640, "letterbox output size mismatch");
    require(scale.width == 640 && scale.height == 320, "letterbox scale mismatch");
    require(pad.x == 0 && pad.y == 160, "letterbox padding mismatch");

    std::vector<rm_assessment::yolov5::Detection> detections = {
      {cv::Rect2f(0.0F, 0.0F, 20.0F, 20.0F), 0.9F, 1},
      {cv::Rect2f(2.0F, 2.0F, 20.0F, 20.0F), 0.8F, 1},
      {cv::Rect2f(100.0F, 100.0F, 10.0F, 10.0F), 0.7F, 2},
    };
    const auto kept = rm_assessment::yolov5::nms(detections, 0.5F);
    require(kept.size() == 2, "nms should suppress overlapping detections");
    require(std::fabs(kept.front().confidence - 0.9F) < 1e-6F, "nms should keep highest-confidence box first");

    cv::Mat decoded_rows(4, 22, CV_32F, cv::Scalar(0.01F));
    decoded_rows.at<float>(0, 0) = 320.0F;
    decoded_rows.at<float>(0, 1) = 320.0F;
    decoded_rows.at<float>(0, 2) = 100.0F;
    decoded_rows.at<float>(0, 3) = 80.0F;
    decoded_rows.at<float>(0, 4) = 0.8F;
    decoded_rows.at<float>(0, 8) = 0.9F;
    decoded_rows.at<float>(1, 0) = 100.0F;
    decoded_rows.at<float>(1, 1) = 100.0F;
    decoded_rows.at<float>(1, 2) = 20.0F;
    decoded_rows.at<float>(1, 3) = 20.0F;
    decoded_rows.at<float>(1, 4) = 0.20F;
    decoded_rows.at<float>(1, 6) = 0.9F;
    decoded_rows.at<float>(2, 0) = 10.0F;
    decoded_rows.at<float>(2, 1) = 170.0F;
    decoded_rows.at<float>(2, 2) = 40.0F;
    decoded_rows.at<float>(2, 3) = 40.0F;
    decoded_rows.at<float>(2, 4) = 0.9F;
    decoded_rows.at<float>(2, 10) = 0.8F;
    decoded_rows.at<float>(3, 0) = 300.0F;
    decoded_rows.at<float>(3, 1) = 300.0F;
    decoded_rows.at<float>(3, 2) = 0.0F;
    decoded_rows.at<float>(3, 3) = 40.0F;
    decoded_rows.at<float>(3, 4) = 0.9F;
    decoded_rows.at<float>(3, 7) = 0.9F;

    const auto decoded = rm_assessment::yolov5::decode_outputs(
      decoded_rows, 0.3F, cv::Size(640, 640), cv::Size(1280, 1280), scale, pad);
    require(decoded.size() == 2, "decode_outputs should filter low-confidence and non-positive rows");
    require(decoded.front().class_id == 3, "decode_outputs class id mismatch");
    require(std::fabs(decoded.front().confidence - 0.72F) < 1e-6F,
      "decode_outputs confidence should combine objectness and class probability");
    require(std::fabs(decoded.front().box.x - 540.0F) < 1e-3F, "decode_outputs left mismatch");
    require(std::fabs(decoded.front().box.y - 480.0F) < 1e-3F, "decode_outputs top mismatch");
    require(std::fabs(decoded.front().box.width - 200.0F) < 1e-3F, "decode_outputs width mismatch");
    require(std::fabs(decoded.front().box.height - 320.0F) < 1e-3F, "decode_outputs height mismatch");
    require(decoded.back().box.x == 0.0F && decoded.back().box.y == 0.0F,
      "decode_outputs should clamp boxes to original bounds");
    require(decoded.back().box.width > 0.0F && decoded.back().box.height > 0.0F,
      "decode_outputs should keep positive clamped boxes");

    const std::filesystem::path model_path = std::filesystem::absolute(
      std::filesystem::path("models") / "yolov5" / "yolov5.xml");
    const std::filesystem::path weights_path = std::filesystem::absolute(
      std::filesystem::path("models") / "yolov5" / "yolov5.bin");
    const std::string model_path_string = model_path.string();
    const std::string weights_path_string = weights_path.string();
    std::cout << "yolov5_test: model path: " << model_path_string << '\n';
    std::cout << "yolov5_test: weights path: " << weights_path_string << '\n';
    require(std::filesystem::exists(model_path), "YOLOv5 model XML must exist");
    require(std::filesystem::exists(weights_path), "YOLOv5 model BIN must exist");

    ov::Core core;
    const auto model = core.read_model(model_path_string, weights_path_string);
    require(model->inputs().size() == 1, "YOLOv5 model should expose exactly one input");
    require(model->outputs().size() == 1, "YOLOv5 model should expose exactly one output");

    const auto input = model->input(0);
    const auto output = model->output(0);
    require(input.get_any_name() == "images", "YOLOv5 input name must be 'images'");
    require(output.get_any_name() == "output", "YOLOv5 output name must be 'output'");
    require(input.get_element_type() == ov::element::f32, "YOLOv5 input type must be float32");
    require(output.get_element_type() == ov::element::f32, "YOLOv5 output type must be float32");

    const auto input_shape = input.get_shape();
    const auto output_shape = output.get_shape();
    require(input_shape.size() == 4, "YOLOv5 input rank mismatch");
    require(input_shape[0] == 1 && input_shape[1] == 3 && input_shape[2] == 640 && input_shape[3] == 640,
      "YOLOv5 input shape mismatch");
    require(output_shape.size() == 3, "YOLOv5 output rank mismatch");
    require(output_shape[0] == 1 && output_shape[1] == 25200 && output_shape[2] == 22,
      "YOLOv5 output shape mismatch");

    std::cout << "yolov5_test: all checks passed\n";
    return 0;
  } catch (const std::exception & ex) {
    std::cerr << "yolov5_test failed: " << ex.what() << '\n';
    return 1;
  }
}
