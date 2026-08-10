#include "io/myCamera.hpp"

#include <chrono>
#include <iostream>
#include <string>

#include <opencv2/highgui.hpp>

int main(int argc, char ** argv)
{
  const std::string source = (argc > 1) ? argv[1] : std::string("0");

  io::myCamera camera(source);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  const std::string window_name = "USB Camera";
  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

  while (true) {
    if (!camera.read(img, timestamp)) {
      std::cerr << "read failed\n";
      break;
    }

    cv::imshow(window_name, img);

    int key = cv::waitKey(1);
    if (key == 27 || key == 'q') {
      break;
    }
  }

  cv::destroyAllWindows();
  return 0;
}
