#ifndef IO__CAMERA_EXPOSURE_HPP_
#define IO__CAMERA_EXPOSURE_HPP_

#include <optional>
#include <string>

#include "myCamera.hpp"

namespace io
{

std::optional<double> loadExposure(const std::string & path = "calibration/camera_exposure.yaml");
bool applyExposure(myCamera & camera, double exposure);
bool applySavedExposure(myCamera & camera, const std::string & path = "calibration/camera_exposure.yaml");
void reportExposure(double exposure, bool applied);

}  // namespace io

#endif  // IO__CAMERA_EXPOSURE_HPP_
