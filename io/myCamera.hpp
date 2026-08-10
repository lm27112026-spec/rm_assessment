#ifndef IO__MY_CAMERA_HPP
#define IO__MY_CAMERA_HPP

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#if !defined(MY_CAMERA_FORCE_FAKE) && __has_include(<opencv2/opencv.hpp>)
#define MY_CAMERA_HAS_OPENCV 1
#include <opencv2/opencv.hpp>
#endif

#ifndef MY_CAMERA_HAS_OPENCV
namespace cv
{
class Mat
{
public:
  Mat() = default;

  void create(int rows, int cols, int channels)
  {
    this->rows = rows;
    this->cols = cols;
    channels_ = channels;
    data_.assign(static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols) * static_cast<std::size_t>(channels_), 0);
  }

  bool empty() const
  {
    return data_.empty();
  }

  int channels() const
  {
    return channels_;
  }

  std::uint8_t * ptr(int row = 0)
  {
    return data_.data() + static_cast<std::size_t>(row) * static_cast<std::size_t>(cols) * static_cast<std::size_t>(channels_);
  }

  const std::uint8_t * ptr(int row = 0) const
  {
    return data_.data() + static_cast<std::size_t>(row) * static_cast<std::size_t>(cols) * static_cast<std::size_t>(channels_);
  }

  int rows = 0;
  int cols = 0;

private:
  int channels_ = 0;
  std::vector<std::uint8_t> data_;
};

class VideoCapture
{
public:
  VideoCapture() = default;
  explicit VideoCapture(const std::string &)
  {
    opened_ = true;
  }

  explicit VideoCapture(int)
  {
    opened_ = true;
  }

  bool open(const std::string &)
  {
    opened_ = true;
    return true;
  }

  bool open(int)
  {
    opened_ = true;
    return true;
  }

  bool isOpened() const
  {
    return opened_;
  }

  void release()
  {
    opened_ = false;
  }

  bool read(Mat &)
  {
    return opened_;
  }

private:
  bool opened_ = false;
};
}  // namespace cv
#endif

namespace io
{
class myCamera
{
public:
  explicit myCamera(const std::string & source = "0");
  ~myCamera();

  bool read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

private:
  std::string source_;
  cv::VideoCapture capture_;
  bool opened_;
  std::size_t frame_index_;
  int width_;
  int height_;
};

}  // namespace io

#endif  // IO__MY_CAMERA_HPP
