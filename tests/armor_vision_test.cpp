#include <exception>
#include <filesystem>
#include <iostream>
#include <list>
#include <string>

#include <opencv2/core.hpp>

#include "armor.hpp"
#include "detector.hpp"

namespace
{
using auto_aim::Armor;
using auto_aim::Detector;

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

std::filesystem::path default_model_path()
{
  return std::filesystem::current_path() / "learning" / "assets" / "tiny_resnet.onnx";
}

void test_detector_missing_model_is_safe()
{
  Detector detector("learning/assets/does_not_exist.onnx");
  require(!detector.has_model(), "Detector reported a missing model as loaded");

  cv::Mat empty;
  const std::list<Armor> armors = detector.detect(empty);
  require(armors.empty(), "Detector returned armors for an empty image");

  cv::Mat black(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
  const std::list<Armor> armors_black = detector.detect(black);
  require(armors_black.empty(), "Detector returned armors for a black image without a model");
}

void test_detector_real_model_loads()
{
  const auto model_path = default_model_path();
  require(std::filesystem::exists(model_path), "Expected test model is missing: " + model_path.string());

  Detector detector(model_path.string());
  require(detector.has_model(), "Detector did not load tiny_resnet.onnx");
}

void test_detector_blank_image_no_detections()
{
  const auto model_path = default_model_path();
  if (!std::filesystem::exists(model_path)) {
    std::cout << "[SKIP] model missing, skipping blank-image test\n";
    return;
  }

  Detector detector(model_path.string());
  require(detector.has_model(), "Detector did not load tiny_resnet.onnx");

  cv::Mat black(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
  const std::list<Armor> armors = detector.detect(black);
  require(armors.empty(), "Detector produced false positives on a blank image");
}

void run_test(const std::string & name, void (*test)())
{
  try {
    test();
    std::cout << "[PASS] " << name << '\n';
  } catch (const std::exception & error) {
    std::cerr << "[FAIL] " << name << ": " << error.what() << '\n';
    throw;
  }
}

}  // namespace

int main()
{
  try {
    run_test("Detector missing model safe", test_detector_missing_model_is_safe);
    run_test("Detector real model loads", test_detector_real_model_loads);
    run_test("Detector blank image no detections", test_detector_blank_image_no_detections);
  } catch (const std::exception &) {
    return 1;
  }
  return 0;
}
