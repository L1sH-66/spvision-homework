#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <opencv2/core.hpp>
#include <string>

namespace io {
class Camera {
 public:
  explicit Camera(const std::string& config_path);
  ~Camera();
  void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp);

 private:
  Camera(const Camera&) = delete;
  Camera& operator=(const Camera&) = delete;

  void* handle_{nullptr};
  bool opened_{false};
  bool grabbing_{false};
};
}  // namespace io

#endif  // IO__CAMERA_HPP
