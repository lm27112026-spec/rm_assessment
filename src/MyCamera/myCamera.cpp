#include "myCamera.hpp"

#include <algorithm>
#include <cctype>

namespace
{
bool is_number(const std::string & value)
{
  return !value.empty() && std::all_of(value.begin(), value.end(), [](unsigned char ch) { return std::isdigit(ch) != 0; });
}
}  // namespace

namespace io
{
myCamera::myCamera(const std::string & source)
  : source_(source),
    capture_(),
    opened_(false),
    frame_index_(0),
    width_(640),
    height_(480),
    exposure_(0.0)
{
#ifdef MY_CAMERA_HAS_OPENCV
  if (source_.empty()) {
    capture_.open(0);
  } else if (is_number(source_)) {
    capture_.open(std::stoi(source_));
  } else {
    capture_.open(source_);
  }
  if (capture_.isOpened()) {
    capture_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  }
  opened_ = capture_.isOpened();
#else
  opened_ = true;
#endif
}

myCamera::~myCamera()
{
#ifdef MY_CAMERA_HAS_OPENCV
  if (capture_.isOpened()) {
    capture_.release();
  }
#endif
}

bool myCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  timestamp = std::chrono::steady_clock::now();

#ifdef MY_CAMERA_HAS_OPENCV
  if (!capture_.isOpened()) {
    return false;
  }

  return capture_.read(img);
#else
  img.create(height_, width_, 3);

  for (int row = 0; row < height_; ++row) {
    std::uint8_t * line = img.ptr(row);
    for (int col = 0; col < width_; ++col) {
      const std::size_t offset = static_cast<std::size_t>(col) * 3U;
      line[offset + 0] = static_cast<std::uint8_t>((col + static_cast<int>(frame_index_)) % 256);
      line[offset + 1] = static_cast<std::uint8_t>((row + static_cast<int>(frame_index_)) % 256);
      line[offset + 2] = static_cast<std::uint8_t>((col + row + static_cast<int>(frame_index_)) % 256);
    }
  }

  ++frame_index_;
  return true;
#endif
}

bool myCamera::setExposure(double exposure)
{
  exposure_ = exposure;

#ifdef MY_CAMERA_HAS_OPENCV
  if (!capture_.isOpened()) {
    return false;
  }

  capture_.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
  return capture_.set(cv::CAP_PROP_EXPOSURE, exposure);
#else
  return true;
#endif
}

double myCamera::getExposure() const
{
#ifdef MY_CAMERA_HAS_OPENCV
  if (capture_.isOpened()) {
    const double exposure = capture_.get(cv::CAP_PROP_EXPOSURE);
    if (exposure != 0.0) {
      return exposure;
    }
  }
#endif

  return exposure_;
}

}  // namespace io
