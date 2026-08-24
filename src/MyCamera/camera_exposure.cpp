#include "camera_exposure.hpp"

#include <cstddef>
#include <fstream>
#include <iostream>

namespace
{

bool parse_double(const std::string & text, double & value)
{
  try {
    std::size_t consumed = 0;
    const double parsed = std::stod(text, &consumed);
    if (consumed != text.size()) return false;
    value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

bool parse_yaml_scalar_double_line(const std::string & line, const std::string & key, double & value)
{
  const std::size_t first = line.find_first_not_of(" \t");
  if (first == std::string::npos) return false;
  const std::string text = line.substr(first);
  if (text.rfind(key, 0) != 0) return false;

  const std::size_t colon = text.find(':');
  if (colon == std::string::npos) return false;

  std::string scalar = text.substr(colon + 1);
  const std::size_t comment = scalar.find('#');
  if (comment != std::string::npos) scalar = scalar.substr(0, comment);

  const std::size_t value_first = scalar.find_first_not_of(" \t\r\n");
  if (value_first == std::string::npos) return false;
  const std::size_t value_last = scalar.find_last_not_of(" \t\r\n");
  return parse_double(scalar.substr(value_first, value_last - value_first + 1), value);
}

}  // namespace

namespace io
{

std::optional<double> loadExposure(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream.is_open()) return std::nullopt;

  std::string line;
  while (std::getline(stream, line)) {
    double exposure = 0.0;
    if (parse_yaml_scalar_double_line(line, "exposure", exposure)) {
      return exposure;
    }
  }
  return std::nullopt;
}

bool applyExposure(myCamera & camera, double exposure)
{
  return camera.setExposure(exposure);
}

bool applySavedExposure(myCamera & camera, const std::string & path)
{
  const std::optional<double> exposure = loadExposure(path);
  if (!exposure) return false;
  reportExposure(*exposure, applyExposure(camera, *exposure));
  return true;
}

void reportExposure(double exposure, bool applied)
{
  std::cout << "Requested exposure: " << exposure;
  if (!applied) {
    std::cout << " (backend rejected setting)";
  }
  std::cout << '\n';
}

}  // namespace io
