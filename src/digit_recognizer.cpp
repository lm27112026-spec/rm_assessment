/**
 * @file digit_recognizer.cpp
 * @brief 数字识别器的实现（模板匹配）
 *
 * 【功能】
 * 实现 DigitRecognizer 的完整识别流程：加载 1~5 号数字模板，
 * 对待识别 ROI 进行与模板一致的预处理后计算加权相似度，
 * 通过置信度阈值与“与次高分差距”双重判定输出可靠识别结果。
 *
 * 【方法】
 *  - recognize：预处理 → 提取最大轮廓 → 居中到标准画布 →
 *    与所有模板逐一计算相似度 → 排序 → 置信度与差距判定；
 *  - load_templates_from_directory：遍历数字 1~5 与四种图片后缀，
 *    读取并预处理有效模板存入 templates_；
 *  - preprocess_roi：转灰度 → 高斯模糊 → OTSU 正/反阈值双路二值化 →
 *    形态学清理 → 按质量得分择优选择正/反二值图；
 *  - score_binary_image：前景占比越接近 18% 且填充率越高得分越高，
 *    前景过少/过多或轮廓过小直接判为无效；
 *  - center_digit_on_canvas：按轮廓包围盒加 12% 内边距裁剪，
 *    等比缩放到 64×64 画布并居中（INTER_NEAREST 保持像素二值性）；
 *  - pixel_similarity：bitwise_xor 统计不匹配像素占比，返回匹配率；
 *  - contour_similarity：cv::matchShapes（CONTOURS_MATCH_I1）计算
 *    轮廓距离 d，再映射为 1/(1+d) 的相似度；
 *  - 其余辅助：to_gray / morph_clean / largest_contour（匿名命名空间）。
 *
 * 【实现方式】
 *  - 相似度 = 0.55 × 轮廓相似度（形状主导）+ 0.45 × 像素相似度；
 *  - 结果要求：最高置信度 ≥ 0.55 且与次高分差距 ≥ 0.05，否则返回 Unknown；
 *  - 预处理对模板与候选完全一致，保证匹配时输入分布对齐。
 */
#include "src/digit_recognizer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <system_error>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace rm_assessment
{
namespace
{

constexpr int kCanonicalWidth = 64;
constexpr int kCanonicalHeight = 64;
constexpr int kMinTemplateDimension = 8;
constexpr double kMinAcceptedConfidence = 0.55;
constexpr double kMinConfidenceMargin = 0.05;
constexpr std::array<const char *, 4> kTemplateExtensions = {".png", ".jpg", ".jpeg", ".bmp"};

cv::Mat to_gray(const cv::Mat & input)
{
  if (input.empty()) {
    return {};
  }
  if (input.channels() == 1) {
    return input.clone();
  }
  cv::Mat gray;
  if (input.channels() == 3) {
    cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
    return gray;
  }
  if (input.channels() == 4) {
    cv::cvtColor(input, gray, cv::COLOR_BGRA2GRAY);
    return gray;
  }
  return {};
}

cv::Mat morph_clean(const cv::Mat & binary)
{
  if (binary.empty()) {
    return {};
  }

  cv::Mat cleaned = binary.clone();
  const int kernel_size = std::max(3, static_cast<int>(std::round(std::min(binary.rows, binary.cols) * 0.06)));
  const int odd_kernel_size = kernel_size % 2 == 0 ? kernel_size + 1 : kernel_size;
  const cv::Mat kernel = cv::getStructuringElement(
    cv::MORPH_RECT, cv::Size(odd_kernel_size, odd_kernel_size));

  cv::morphologyEx(cleaned, cleaned, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(cleaned, cleaned, cv::MORPH_CLOSE, kernel);
  return cleaned;
}

std::vector<cv::Point> largest_contour(const cv::Mat & binary, cv::Rect & bounding_box)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.empty()) {
    bounding_box = {};
    return {};
  }

  auto best_it = std::max_element(
    contours.begin(), contours.end(), [](const auto & lhs, const auto & rhs) {
      return cv::contourArea(lhs) < cv::contourArea(rhs);
    });
  if (best_it == contours.end()) {
    bounding_box = {};
    return {};
  }

  bounding_box = cv::boundingRect(*best_it);
  return *best_it;
}

}  // namespace

DigitRecognizer::DigitRecognizer(const std::string & template_directory)
{
  set_template_directory(template_directory);
}

void DigitRecognizer::set_template_directory(const std::string & template_directory)
{
  template_directory_ = template_directory;
  templates_.clear();

  if (!template_directory_.empty()) {
    load_templates_from_directory(template_directory_);
  }
}

bool DigitRecognizer::has_templates() const noexcept
{
  return !templates_.empty();
}

RecognitionResult DigitRecognizer::recognize(const ArmorCandidate & candidate) const
{
  return recognize(candidate.normalized_roi);
}

RecognitionResult DigitRecognizer::recognize(const cv::Mat & normalized_roi) const
{
  if (normalized_roi.empty() || templates_.empty()) {
    return unknown_result();
  }

  cv::Mat candidate_binary = preprocess_roi(normalized_roi);
  if (candidate_binary.empty()) {
    return unknown_result();
  }

  std::vector<cv::Point> candidate_contour;
  cv::Rect candidate_box;
  candidate_contour = largest_contour(candidate_binary, candidate_box);
  if (candidate_contour.empty() || candidate_box.empty()) {
    return unknown_result();
  }

  const cv::Mat candidate_canvas = center_digit_on_canvas(candidate_binary, candidate_box);
  if (candidate_canvas.empty()) {
    return unknown_result();
  }

  struct ScoredTemplate
  {
    int digit{-1};
    double confidence{0.0};
  };

  std::vector<ScoredTemplate> scored_templates;
  scored_templates.reserve(templates_.size());

  for (const auto & sample : templates_) {
    if (sample.binary.empty() || sample.contour.empty()) {
      continue;
    }

    const double pixel_score = pixel_similarity(candidate_canvas, sample.binary);
    const double shape_score = contour_similarity(candidate_contour, sample.contour);
    const double confidence = 0.55 * shape_score + 0.45 * pixel_score;
    scored_templates.push_back({sample.digit, confidence});
  }

  if (scored_templates.empty()) {
    return unknown_result();
  }

  std::sort(
    scored_templates.begin(), scored_templates.end(),
    [](const ScoredTemplate & lhs, const ScoredTemplate & rhs) {
      return lhs.confidence > rhs.confidence;
    });

  const ScoredTemplate & best = scored_templates.front();
  const double second_best = scored_templates.size() > 1 ? scored_templates[1].confidence : 0.0;
  if (best.confidence < kMinAcceptedConfidence || best.confidence - second_best < kMinConfidenceMargin) {
    return unknown_result();
  }

  RecognitionResult result;
  result.digit = best.digit;
  result.label = std::to_string(best.digit);
  result.confidence = best.confidence;
  result.reliable = true;
  return result;
}

bool DigitRecognizer::load_templates_from_directory(const std::string & template_directory)
{
  namespace fs = std::filesystem;

  const fs::path root(template_directory);
  std::error_code error;
  if (template_directory.empty() || !fs::exists(root, error) || error ||
      !fs::is_directory(root, error) || error) {
    return false;
  }

  bool loaded_any = false;
  for (int digit = 1; digit <= 5; ++digit) {
    cv::Mat template_image;
    for (const char * extension : kTemplateExtensions) {
      const fs::path candidate_path = root / (std::to_string(digit) + extension);
      error.clear();
      if (!fs::exists(candidate_path, error) || error ||
          !fs::is_regular_file(candidate_path, error) || error) {
        continue;
      }

      template_image = cv::imread(candidate_path.string(), cv::IMREAD_UNCHANGED);
      if (!template_image.empty()) {
        break;
      }
    }

    if (template_image.empty() || template_image.cols < kMinTemplateDimension ||
        template_image.rows < kMinTemplateDimension) {
      continue;
    }

    cv::Mat binary = preprocess_roi(template_image);
    if (binary.empty()) {
      continue;
    }

    std::vector<cv::Point> contour;
    cv::Rect bounding_box;
    contour = largest_contour(binary, bounding_box);
    if (contour.empty() || bounding_box.empty()) {
      continue;
    }

    cv::Mat canvas = center_digit_on_canvas(binary, bounding_box);
    if (canvas.empty()) {
      continue;
    }

    templates_.push_back(TemplateSample{digit, canvas, contour});
    loaded_any = true;
  }

  return loaded_any;
}

RecognitionResult DigitRecognizer::unknown_result() noexcept
{
  return {};
}

cv::Mat DigitRecognizer::preprocess_roi(const cv::Mat & input)
{
  cv::Mat gray = to_gray(input);
  if (gray.empty() || gray.cols < kMinTemplateDimension || gray.rows < kMinTemplateDimension) {
    return {};
  }

  cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0.0);

  cv::Mat threshold_normal;
  cv::Mat threshold_inverse;
  cv::threshold(gray, threshold_normal, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  cv::threshold(gray, threshold_inverse, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

  threshold_normal = morph_clean(threshold_normal);
  threshold_inverse = morph_clean(threshold_inverse);

  const double normal_score = score_binary_image(threshold_normal);
  const double inverse_score = score_binary_image(threshold_inverse);

  if (normal_score < 0.0 && inverse_score < 0.0) {
    return {};
  }

  return inverse_score > normal_score ? threshold_inverse : threshold_normal;
}

double DigitRecognizer::score_binary_image(const cv::Mat & binary)
{
  if (binary.empty() || binary.type() != CV_8UC1) {
    return -1.0;
  }

  const double foreground_ratio = static_cast<double>(cv::countNonZero(binary)) /
    static_cast<double>(binary.total());
  if (foreground_ratio < 0.01 || foreground_ratio > 0.85) {
    return -1.0;
  }

  cv::Rect bounding_box;
  const std::vector<cv::Point> contour = largest_contour(binary, bounding_box);
  if (contour.empty() || bounding_box.empty() || bounding_box.area() <= 0) {
    return -1.0;
  }

  const double contour_area = cv::contourArea(contour);
  if (contour_area < 8.0) {
    return -1.0;
  }

  const double fill_ratio = contour_area / static_cast<double>(bounding_box.area());
  const double foreground_score = 1.0 - std::min(1.0, std::abs(foreground_ratio - 0.18) / 0.18);
  const double fill_score = std::clamp((fill_ratio - 0.05) / 0.8, 0.0, 1.0);
  return 0.6 * foreground_score + 0.4 * fill_score;
}

bool DigitRecognizer::extract_primary_contour(
  const cv::Mat & binary, std::vector<cv::Point> & contour, cv::Rect & bounding_box)
{
  contour = largest_contour(binary, bounding_box);
  return !contour.empty() && !bounding_box.empty();
}

cv::Mat DigitRecognizer::center_digit_on_canvas(const cv::Mat & binary, const cv::Rect & bounding_box)
{
  if (binary.empty() || bounding_box.empty()) {
    return {};
  }

  const int padding = std::max(2, static_cast<int>(std::round(0.12 * std::max(bounding_box.width, bounding_box.height))));
  const int left = std::max(0, bounding_box.x - padding);
  const int top = std::max(0, bounding_box.y - padding);
  const int right = std::min(binary.cols, bounding_box.x + bounding_box.width + padding);
  const int bottom = std::min(binary.rows, bounding_box.y + bounding_box.height + padding);
  const cv::Rect expanded(left, top, std::max(0, right - left), std::max(0, bottom - top));
  if (expanded.empty()) {
    return {};
  }

  const cv::Mat crop = binary(expanded);
  if (crop.empty()) {
    return {};
  }

  cv::Mat canvas(cv::Size(kCanonicalWidth, kCanonicalHeight), CV_8UC1, cv::Scalar(0));
  const double scale_x = static_cast<double>(kCanonicalWidth - 4) / std::max(1, crop.cols);
  const double scale_y = static_cast<double>(kCanonicalHeight - 4) / std::max(1, crop.rows);
  const double scale = std::min(scale_x, scale_y);
  if (scale <= 0.0) {
    return {};
  }

  const int resized_width = std::max(1, static_cast<int>(std::round(crop.cols * scale)));
  const int resized_height = std::max(1, static_cast<int>(std::round(crop.rows * scale)));
  cv::Mat resized;
  cv::resize(crop, resized, cv::Size(resized_width, resized_height), 0.0, 0.0, cv::INTER_NEAREST);
  if (resized.empty()) {
    return {};
  }

  const int offset_x = (kCanonicalWidth - resized.cols) / 2;
  const int offset_y = (kCanonicalHeight - resized.rows) / 2;
  const cv::Rect place(offset_x, offset_y, resized.cols, resized.rows);
  if (place.x < 0 || place.y < 0 || place.x + place.width > canvas.cols ||
      place.y + place.height > canvas.rows) {
    return {};
  }

  resized.copyTo(canvas(place));
  return canvas;
}

double DigitRecognizer::pixel_similarity(const cv::Mat & lhs, const cv::Mat & rhs)
{
  if (lhs.empty() || rhs.empty() || lhs.size() != rhs.size() || lhs.type() != CV_8UC1 ||
      rhs.type() != CV_8UC1) {
    return 0.0;
  }

  cv::Mat difference;
  cv::bitwise_xor(lhs, rhs, difference);
  const double mismatch_ratio = static_cast<double>(cv::countNonZero(difference)) /
    static_cast<double>(difference.total());
  return std::clamp(1.0 - mismatch_ratio, 0.0, 1.0);
}

double DigitRecognizer::contour_similarity(
  const std::vector<cv::Point> & lhs, const std::vector<cv::Point> & rhs)
{
  if (lhs.empty() || rhs.empty()) {
    return 0.0;
  }

  const double distance = cv::matchShapes(lhs, rhs, cv::CONTOURS_MATCH_I1, 0.0);
  if (!std::isfinite(distance) || distance < 0.0) {
    return 0.0;
  }
  return 1.0 / (1.0 + distance);
}

}  // namespace rm_assessment
