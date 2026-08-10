/**
 * @file digit_recognizer.hpp
 * @brief 数字识别器类声明（模板匹配）
 *
 * 【功能】
 * 声明基于模板匹配的数字识别器 DigitRecognizer，用于识别装甲板上的数字
 * （本项目范围为 1~5）。识别结果由 RecognitionResult 结构体表示：
 *  - label：数字标签字符串；
 *  - confidence：匹配置信度；
 *  - digit：识别出的数字（-1 表示未知）；
 *  - reliable：结果是否可靠（满足置信度阈值与差距要求）。
 *
 * 模板目录为可选配置：
 *  - 目录为空或不可读时识别器仍可用，此时返回 Unknown 而不报错；
 *  - 目录存在时从该目录顶层读取 1.png ~ 5.png（支持 .png/.jpg/.jpeg/.bmp）；
 *  - 缺失、为空或尺寸过小的模板会被安全跳过。
 *
 * 【方法】
 *  - recognize(candidate / normalized_roi)：识别入口（接受装甲候选或其 ROI）；
 *  - set_template_directory / has_templates：模板目录设置与模板是否存在查询；
 *  - load_templates_from_directory：从目录加载并预处理模板；
 *  - preprocess_roi：ROI 预处理（灰度化、模糊、OTSU 二值化、形态学清理）；
 *  - score_binary_image：对二值图质量打分（前景占比、填充率）；
 *  - extract_primary_contour：提取最大前景轮廓；
 *  - center_digit_on_canvas：将数字裁剪、等比缩放并居中到 64×64 画布；
 *  - pixel_similarity / contour_similarity：像素级与轮廓级相似度计算。
 *
 * 【实现方式】
 *  - 模板加载后统一预处理为二值图并提取轮廓存储；
 *  - 识别时对候选 ROI 做同样的预处理，再与所有模板计算
 *    加权相似度（0.55 × 轮廓相似度 + 0.45 × 像素相似度）；
 *  - 取置信度最高且满足“最低阈值 + 与次高分的差距”双重条件的模板作为结果，
 *    否则返回 Unknown。实现见 digit_recognizer.cpp。
 */
#ifndef RM_ASSESSMENT_SRC_DIGIT_RECOGNIZER_HPP_
#define RM_ASSESSMENT_SRC_DIGIT_RECOGNIZER_HPP_

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "src/armor.hpp"

namespace rm_assessment
{

struct RecognitionResult
{
  std::string label{"Unknown"};
  double confidence{0.0};
  int digit{-1};
  bool reliable{false};
};

class DigitRecognizer
{
public:
  // Template directory behavior:
  // - The directory is optional. If it is empty or unreadable, the recognizer stays usable
  //   and will return Unknown instead of failing.
  // - When present, the recognizer looks for label files named 1.png ... 5.png at the top
  //   level of that directory. Common image suffixes such as .png, .jpg, .jpeg, and .bmp
  //   are supported.
  // - Missing, empty, or too-small template images are skipped safely.
  explicit DigitRecognizer(const std::string & template_directory = {});

  void set_template_directory(const std::string & template_directory);

  bool has_templates() const noexcept;

  RecognitionResult recognize(const ArmorCandidate & candidate) const;
  RecognitionResult recognize(const cv::Mat & normalized_roi) const;

private:
  struct TemplateSample
  {
    int digit{-1};
    cv::Mat binary;
    std::vector<cv::Point> contour;
  };

  std::string template_directory_;
  std::vector<TemplateSample> templates_;

  bool load_templates_from_directory(const std::string & template_directory);

  static RecognitionResult unknown_result() noexcept;
  static cv::Mat preprocess_roi(const cv::Mat & input);
  static double score_binary_image(const cv::Mat & binary);
  static bool extract_primary_contour(
    const cv::Mat & binary, std::vector<cv::Point> & contour, cv::Rect & bounding_box);
  static cv::Mat center_digit_on_canvas(const cv::Mat & binary, const cv::Rect & bounding_box);
  static double pixel_similarity(const cv::Mat & lhs, const cv::Mat & rhs);
  static double contour_similarity(const std::vector<cv::Point> & lhs, const std::vector<cv::Point> & rhs);
};

}  // namespace rm_assessment

#endif  // RM_ASSESSMENT_SRC_DIGIT_RECOGNIZER_HPP_
