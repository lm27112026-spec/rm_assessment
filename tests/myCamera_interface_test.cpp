#include "myCamera.hpp"

#include <chrono>
#include <string>
#include <type_traits>

int main()
{
  static_assert(std::is_constructible<io::myCamera, const std::string &>::value,
    "myCamera must be constructible from const std::string &");
  static_assert(std::is_destructible<io::myCamera>::value,
    "myCamera must be destructible");
  static_assert(std::is_same<decltype(&io::myCamera::read),
                  bool (io::myCamera::*)(cv::Mat &, std::chrono::steady_clock::time_point &)>::value,
    "myCamera::read must match the required interface");
  return 0;
}
