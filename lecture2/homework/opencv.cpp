#include "opencv2/opencv.hpp"

#include <chrono>
#include <exception>
#include <iostream>
#include <string>

#include "io/camera.hpp"
#include "tasks/apriltag_detector.hpp"
#include "tools/img_tools.hpp"

int main() {
  try {
    io::Camera camera("./configs/camera.yaml");
    auto_charge::AprilTagDetector detector("./configs/yolo.yaml");

    while (true) {
      cv::Mat img;
      std::chrono::steady_clock::time_point timestamp;
      camera.read(img, timestamp);

      const auto tags = detector.detect(img);
      for (const auto& tag : tags) {
        tools::draw_points(img, tag.corners, cv::Scalar(0, 255, 0), 2);
        const cv::Point label_point(static_cast<int>(tag.center.x + 8.0F),
                                    static_cast<int>(tag.center.y - 8.0F));
        tools::draw_text(img, "AprilTag " + std::to_string(tag.id), label_point,
                         cv::Scalar(0, 255, 0), 0.7, 2);
      }

      cv::Mat display;
      cv::resize(img, display, cv::Size(640, 480));
      cv::imshow("AprilTag", display);
      const int key = cv::waitKey(1);
      if (key == 'q' || key == 'Q' || key == 27) {
        break;
      }
    }
  } catch (const std::exception& error) {
    std::cerr << "ERROR: " << error.what() << '\n';
    return 1;
  }

  return 0;
}
