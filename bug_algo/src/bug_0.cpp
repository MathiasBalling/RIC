#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;

Point start_pos;
Point end_po;
bool start_clicked = false;
bool end_clicked = false;
const std::string WINDOW_NAME = "Bug0";

double distance(Point2i p1, Point2i p2) {
  Point end_po2;
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

double distance(Vec2f p1, Vec2f p2) { return norm(p2 - p1); }

Vec2f normalize(Vec2f vec) { return vec / norm(vec); }

// Check if a point is within the boundaries of the map and on white
bool is_valid(Point2i point, const Mat &map) {
  if (point.x < 0 || point.x >= map.cols || point.y < 0 ||
      point.y >= map.rows) {
    return false;
  }
  return map.at<Vec3b>(point) == Vec3b(255, 255, 255);
}
double deg2rad(double deg) { return deg * M_PI / 180; }

// Function to move the robot directly toward the goal
Point2i move_toward_goal(Point2i current, const Point2i goal, int step_size) {
  Vec2f goal_pos = Vec2f(goal.x, goal.y);
  Vec2f current_pos = Vec2f(current.x, current.y);
  Vec2f direction = normalize(goal_pos - current_pos);
  Point2i new_pos = Point2i(current.x + direction[0] * step_size,
                            current.y + direction[1] * step_size);
  return new_pos;
}

void on_mouse(int event, int x, int y, int flags, void *userdata) {
  (void)flags; // Prevent unused variable warning
  Mat *mat = (Mat *)userdata;
  if (event == EVENT_LBUTTONDOWN) {
    if (is_valid(Point2i(x, y), *mat)) {
      if (!start_clicked) {
        start_pos = Point(x, y);
        start_clicked = true;
        std::println("Start position: ({}, {})\n", start_pos.x, start_pos.y);
      } else if (!end_clicked) {
        end_po = Point(x, y);
        end_clicked = true;
        std::println("End position: ({}, {})\n", end_po.x, end_po.y);
      }
    } else {
      std::println("Invalid start position. Please select a white pixel.\n");
    }
  }
}

bool bug0_algorithm(const Mat &map, Mat &final_map, const Point2i start,
                    const Point2i goal, const int step_size = 3) {
  Point2i current = start;
  Point2i last_position = start;
  while (distance(current, goal) > step_size) {
    Point2i next = move_toward_goal(current, goal, step_size);
    // Draw the line on the final map
    circle(final_map, current, step_size / 1.5, Scalar(0, 0, 255), FILLED);
    imshow(WINDOW_NAME, final_map);
    if (waitKey(20) == 'q') {
      return false;
    }
    if (is_valid(next, map)) {
      last_position = current;
      current = next;
    } else {
      // Find the closest point on the obstacle boundary
      double min_distance = std::numeric_limits<double>::max();
      Point2i closest_point = current;
      for (int i = 0; i < 360; i += 10) {
        Point2i point(current.x + step_size * cos(deg2rad(i)),
                      current.y + step_size * sin(deg2rad(i)));

        if (is_valid(point, map)) {
          double dist = distance(point, goal);
          // Should be further than the step size away
          bool has_visited = distance(point, last_position) < (step_size * 1.5);

          if (dist < min_distance && !has_visited) {
            min_distance = dist;
            closest_point = point;
          }
        }
      }
      last_position = current;
      current = closest_point;
    }
  }
  return true;
}

int main() {
  // Create a the map
  Mat img_ws1 = Mat::zeros(500, 500, CV_8UC3);
  img_ws1 = Scalar(255, 255, 255);
  Mat img_final;
  // Draw the obstacles
  rectangle(img_ws1, Point(100, 100), Point(400, 200), Scalar(0), FILLED);
  circle(img_ws1, Point(100, 400), 50, Scalar(0), FILLED);
  // Draw a triangle
  Point triangle_pts[1][3];
  triangle_pts[0][0] = Point(200, 300);
  triangle_pts[0][1] = Point(400, 300);
  triangle_pts[0][2] = Point(350, 400);
  const Point *ppt[1] = {triangle_pts[0]};
  int npt[] = {3};
  fillPoly(img_ws1, ppt, npt, 1, Scalar(0));

  img_ws1 = imread("../../../assets/ws4.png");

  namedWindow(WINDOW_NAME);                          // Create the window
  setMouseCallback(WINDOW_NAME, on_mouse, &img_ws1); // Set the on_mouse
  while (true) {
    // Clone the edges image
    Mat img_ws1_temp = img_ws1.clone();
    if (start_clicked) {
      circle(img_ws1_temp, start_pos, 5, Scalar(0, 0, 255), FILLED);
    }
    if (end_clicked) {
      circle(img_ws1_temp, end_po, 5, Scalar(0, 0, 255), FILLED);
      img_final = img_ws1_temp.clone();
      break;
    }
    imshow(WINDOW_NAME, img_ws1_temp);
    if (waitKey(100) == 'q') {
      return 0;
    }
  }
  // Remove the mouse callback
  setMouseCallback(WINDOW_NAME, NULL, NULL);

  // Analyze the image
  std::println("Analyzing the image...");
  std::println("Distance to goal: {:.0f}", distance(start_pos, end_po));
  if (bug0_algorithm(img_ws1, img_final, start_pos, end_po)) {
    std::println("Goal reached!");
  } else {
    std::println("Goal not reached!");
  }
  waitKey(0);
  destroyAllWindows();
  return 0;
}
