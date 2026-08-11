#include "yolov5_utils.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <filesystem>
#include <limits>
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

    // ===== armor model decode contract tests =====
    // Model output contract: [1,25200,22]; cols 0-7 = 4 corners (TL:0-1, BL:2-3, BR:4-5, TR:6-7),
    // col 8 = score logit (sigmoid), cols 9-12 = color one-hot, cols 13-21 = digit one-hot.
    constexpr float kThreshold = 0.7F;
    const cv::Size kInputSize(640, 640);
    const cv::Size kOriginalSize(1280, 1280);

    // 1. Score logit 0 -> sigmoid(0) = 0.5 < 0.7 -> no detections.
    cv::Mat low_score_row(1, rm_assessment::yolov5::kOutputCols, CV_32F, cv::Scalar(0.01F));
    low_score_row.at<float>(0, rm_assessment::yolov5::kScoreColumn) = 0.0F;
    const auto low_score_dets = rm_assessment::yolov5::decode_outputs(
      low_score_row, kThreshold, kInputSize, kOriginalSize, scale, pad);
    require(low_score_dets.empty(), "sigmoid(0)=0.5 should be filtered below the 0.7 threshold");

    // 2. Score logit 2.2 -> sigmoid(2.2) ~= 0.9002 -> one detection at ~0.9 confidence.
    cv::Mat high_score_row(1, rm_assessment::yolov5::kOutputCols, CV_32F, cv::Scalar(0.01F));
    high_score_row.at<float>(0, rm_assessment::yolov5::kScoreColumn) = 2.2F;
    const auto high_score_dets = rm_assessment::yolov5::decode_outputs(
      high_score_row, kThreshold, kInputSize, kOriginalSize, scale, pad);
    require(high_score_dets.size() == 1, "high-score armor row should produce one detection");
    require(std::fabs(high_score_dets.front().confidence - rm_assessment::yolov5::sigmoid(2.2F)) < 0.05F,
      "confidence should equal sigmoid of the score logit");

    // 3. Corner order: model emits [TL, BL, BR, TR]; Detection.corners must be [TL, TR, BR, BL].
    //    Use identity mapping (scale == original size, zero padding) so corners map 1:1.
    cv::Mat corner_row(1, rm_assessment::yolov5::kOutputCols, CV_32F, cv::Scalar(0.01F));
    corner_row.at<float>(0, 0) = 10.0F;  // TL.x
    corner_row.at<float>(0, 1) = 20.0F;  // TL.y
    corner_row.at<float>(0, 2) = 30.0F;  // BL.x
    corner_row.at<float>(0, 3) = 40.0F;  // BL.y
    corner_row.at<float>(0, 4) = 50.0F;  // BR.x
    corner_row.at<float>(0, 5) = 60.0F;  // BR.y
    corner_row.at<float>(0, 6) = 70.0F;  // TR.x
    corner_row.at<float>(0, 7) = 80.0F;  // TR.y
    corner_row.at<float>(0, rm_assessment::yolov5::kScoreColumn) = 2.2F;
    const auto corner_dets = rm_assessment::yolov5::decode_outputs(
      corner_row, kThreshold, kInputSize, cv::Size(640, 320), cv::Size(640, 320), cv::Point(0, 0));
    require(corner_dets.size() == 1, "corner-order row should produce one detection");
    const auto & corners = corner_dets.front().corners;
    require(corners[0].x == 10.0F && corners[0].y == 20.0F, "corners[0] should be the TL point");
    require(corners[1].x == 70.0F && corners[1].y == 80.0F, "corners[1] should be the TR point");
    require(corners[2].x == 50.0F && corners[2].y == 60.0F, "corners[2] should be the BR point");
    require(corners[3].x == 30.0F && corners[3].y == 40.0F, "corners[3] should be the BL point");

    // 4. color_id = argmax of cols 9-12 (col 9 = 0.9 wins -> color_id 0).
    cv::Mat color_row(1, rm_assessment::yolov5::kOutputCols, CV_32F, cv::Scalar(0.01F));
    color_row.at<float>(0, rm_assessment::yolov5::kScoreColumn) = 2.2F;
    color_row.at<float>(0, rm_assessment::yolov5::kColorStartCol) = 0.9F;
    const auto color_dets = rm_assessment::yolov5::decode_outputs(
      color_row, kThreshold, kInputSize, kOriginalSize, scale, pad);
    require(color_dets.size() == 1, "color row should produce one detection");
    require(color_dets.front().color_id == 0, "color_id should be argmax of cols 9-12");

    // 5. digit_id = argmax of cols 13-21 (col 15 = 0.9 -> digit index 15-13 = 2).
    cv::Mat digit_row(1, rm_assessment::yolov5::kOutputCols, CV_32F, cv::Scalar(0.01F));
    digit_row.at<float>(0, rm_assessment::yolov5::kScoreColumn) = 2.2F;
    digit_row.at<float>(0, rm_assessment::yolov5::kDigitStartCol + 2) = 0.9F;
    const auto digit_dets = rm_assessment::yolov5::decode_outputs(
      digit_row, kThreshold, kInputSize, kOriginalSize, scale, pad);
    require(digit_dets.size() == 1, "digit row should produce one detection");
    require(digit_dets.front().digit_id == 2, "digit_id should be argmax of cols 13-21");

    // 6. Output with fewer than 22 columns must yield no detections.
    cv::Mat short_row(1, rm_assessment::yolov5::kOutputCols - 1, CV_32F, cv::Scalar(0.01F));
    short_row.at<float>(0, rm_assessment::yolov5::kScoreColumn) = 2.2F;
    const auto short_dets = rm_assessment::yolov5::decode_outputs(
      short_row, kThreshold, kInputSize, kOriginalSize, scale, pad);
    require(short_dets.empty(), "output with fewer than 22 columns should yield no detections");

    // 7. NaN score logit must be skipped (guarded before sigmoid).
    cv::Mat nan_score_row(1, rm_assessment::yolov5::kOutputCols, CV_32F, cv::Scalar(0.01F));
    nan_score_row.at<float>(0, rm_assessment::yolov5::kScoreColumn) =
      std::numeric_limits<float>::quiet_NaN();
    const auto nan_score_dets = rm_assessment::yolov5::decode_outputs(
      nan_score_row, kThreshold, kInputSize, kOriginalSize, scale, pad);
    require(nan_score_dets.empty(), "NaN score logit row should yield no detections");

    // 8. NaN corner coordinate must be skipped.
    cv::Mat nan_corner_row(1, rm_assessment::yolov5::kOutputCols, CV_32F, cv::Scalar(0.01F));
    nan_corner_row.at<float>(0, 3) = std::numeric_limits<float>::quiet_NaN();
    nan_corner_row.at<float>(0, rm_assessment::yolov5::kScoreColumn) = 2.2F;
    const auto nan_corner_dets = rm_assessment::yolov5::decode_outputs(
      nan_corner_row, kThreshold, kInputSize, kOriginalSize, scale, pad);
    require(nan_corner_dets.empty(), "NaN corner row should yield no detections");

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
    const auto model = core.read_model(model_path_string);
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
