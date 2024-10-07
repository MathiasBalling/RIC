#include "bug0.hpp"
#include "bug1.hpp"
#include "bug2.hpp"
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <string>

cv::Point start_pos;
cv::Point end_pos;
bool start_clicked = false;
bool end_clicked = false;

void on_mouse(int event, int x, int y, int flags, void *userdata) {
  (void)userdata; // Prevent unused variable warning
  (void)flags;    // Prevent unused variable warning
  if (event == cv::EVENT_LBUTTONDOWN) {
    if (!start_clicked) {
      start_pos = cv::Point(x, y);
      start_clicked = true;
      std::print("Start position: ({}, {})\n", start_pos.x, start_pos.y);
    } else if (!end_clicked) {
      end_pos = cv::Point(x, y);
      end_clicked = true;
      std::print("End position: ({}, {})\n", end_pos.x, end_pos.y);
    }
  }
}

int main() {
  // Read the image
  cv::Mat img_ws1 = cv::imread("../../../assets/ws2.png");
  cv::Mat img_final;
  if (img_ws1.empty()) {
    std::println("Could not read the image");
    return 1;
  }

  std::string window_name = "Bug1";
  cv::namedWindow(window_name);                // Create the window
  cv::setMouseCallback(window_name, on_mouse); // Set the on_mouse
  while (true) {
    // Clone the edges image
    cv::Mat img_ws1_temp = img_ws1.clone();
    if (start_clicked) {
      cv::circle(img_ws1_temp, start_pos, 10, cv::Scalar(0, 0, 255),
                 cv::FILLED);
    }
    if (end_clicked) {
      cv::circle(img_ws1_temp, end_pos, 10, cv::Scalar(0, 0, 255), cv::FILLED);
      img_final = img_ws1_temp.clone();
      break;
    }
    cv::imshow(window_name, img_ws1_temp);
    if (cv::waitKey(10) == 'q') {
      return 0;
    }
  }
  // Analyze the image
  std::println("Analyzing the image...");
  bug0(img_ws1, start_pos, end_pos, img_final);
  cv::imshow(window_name, img_final);
  cv::waitKey(0);
  return 0;
}
