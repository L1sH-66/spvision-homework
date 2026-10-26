#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include "io/camera.hpp"
#include "opencv2/opencv.hpp"
#include "tasks/yolo.hpp"

namespace {
struct LightColor {
  std::string name;
  cv::Scalar bgr;
};

// Estimate visible red/blue light from the ORIGINAL image, before annotations.
// This is an image heuristic, not the YOLO class/color output.
LightColor estimateLightColor(const cv::Mat& image,
                              const std::vector<cv::Point>& polygon) {
  const LightColor unknown{"UNKNOWN", cv::Scalar(0, 255, 255)};
  if (image.empty() || image.type() != CV_8UC3) return unknown;
  const cv::Rect roi = cv::boundingRect(polygon) &
                       cv::Rect(0, 0, image.cols, image.rows);
  if (roi.empty()) return unknown;

  std::vector<cv::Point> local;
  for (const auto& point : polygon) local.push_back(point - roi.tl());
  cv::Mat mask = cv::Mat::zeros(roi.size(), CV_8UC1);
  cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{local}, cv::Scalar(255));
  cv::Mat hsv;
  cv::cvtColor(image(roi), hsv, cv::COLOR_BGR2HSV);
  int red = 0, blue = 0;
  for (int y = 0; y < hsv.rows; ++y) {
    const auto* pixels = hsv.ptr<cv::Vec3b>(y);
    const auto* valid = mask.ptr<unsigned char>(y);
    for (int x = 0; x < hsv.cols; ++x) {
      if (!valid[x]) continue;
      const auto& pixel = pixels[x];
      if (pixel[1] < 80 || pixel[2] < 70) continue;
      if (pixel[0] <= 12 || pixel[0] >= 168) ++red;
      if (pixel[0] >= 90 && pixel[0] <= 135) ++blue;
    }
  }
  if (red >= 8 && red > blue * 1.5) return {"RED", cv::Scalar(0, 0, 255)};
  if (blue >= 8 && blue > red * 1.5) return {"BLUE", cv::Scalar(255, 0, 0)};
  return unknown;
}

void drawLabel(cv::Mat& image, const std::string& text, cv::Point position,
               const cv::Scalar& color) {
  constexpr double scale = 0.6;
  int baseline = 0;
  const cv::Size size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX,
                                       scale, 2, &baseline);
  position.x = std::max(2, std::min(position.x, image.cols - size.width - 2));
  position.y = std::max(size.height + 2,
                        std::min(position.y, image.rows - baseline - 2));
  cv::putText(image, text, position, cv::FONT_HERSHEY_SIMPLEX, scale,
              cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
  cv::putText(image, text, position, cv::FONT_HERSHEY_SIMPLEX, scale,
              color, 2, cv::LINE_AA);
}
}  // namespace

int main() {
  try {
    io::Camera camera("./configs/camera.yaml");
    auto_aim::YOLO yolo("./configs/yolo.yaml");
    int frame_count = 0;
    while (true) {
      cv::Mat img;
      std::chrono::steady_clock::time_point timestamp;
      camera.read(img, timestamp);
      if (img.empty()) continue;
      const auto armors = yolo.detect(img, frame_count++);

      cv::Mat display;
      cv::resize(img, display, cv::Size(640, 480));
      const double sx = static_cast<double>(display.cols) / img.cols;
      const double sy = static_cast<double>(display.rows) / img.rows;
      for (const auto& armor : armors) {
        if (armor.points.size() != 4) continue;
        std::vector<cv::Point> original, points;
        bool valid = true;
        for (const auto& point : armor.points) {
          if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
              point.x < 0 || point.y < 0 ||
              point.x >= img.cols || point.y >= img.rows) {
            valid = false;
            break;
          }
          original.emplace_back(cvRound(point.x), cvRound(point.y));
          points.emplace_back(cvRound(point.x * sx), cvRound(point.y * sy));
        }
        if (!valid) continue;
        const auto light = estimateLightColor(img, original);
        cv::polylines(display, std::vector<std::vector<cv::Point>>{points},
                      true, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        cv::Point center(0, 0);
        for (const auto& point : points) center += point;
        center.x /= 4;
        center.y /= 4;
        for (std::size_t i = 0; i < points.size(); ++i) {
          cv::circle(display, points[i], 4, cv::Scalar(0, 255, 255),
                     cv::FILLED, cv::LINE_AA);
          const int dx = points[i].x < center.x ? -20 : 8;
          const int dy = points[i].y < center.y ? -8 : 22;
          drawLabel(display, std::to_string(i + 1),
                    points[i] + cv::Point(dx, dy), cv::Scalar(0, 255, 255));
        }
        const cv::Rect bounds = cv::boundingRect(points);
        drawLabel(display, "Light: " + light.name,
                  cv::Point(bounds.x, bounds.y - 12), light.bgr);
      }
      cv::imshow("img", display);
      const int key = cv::waitKey(1);
      if (key == 'q' || key == 'Q' || key == 27) break;
    }
  } catch (const std::exception& error) {
    std::cerr << "ERROR: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
