#include <cmath>
#include <cstddef>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <print>
#include <string>
#include <vector>
using namespace cv;

Point2d start_pos;
Point2d end_po;
bool start_clicked = false;
bool end_clicked = false;
const std::string WINDOW_NAME = "Bug0";
bool wall_right = false;

double distance(Point2d p1, Point2d p2) { return norm(p2 - p1); }

double distance(Vec2d p1, Vec2d p2) { return norm(p2 - p1); }

double calculateAngle(const Vec2d vec1, const Vec2d vec2) {
  // Calculate the dot product
  double dotProduct = vec1.dot(vec2);
  // Calculate the magnitudes of the vectors
  double magnitude1 = norm(vec1);
  double magnitude2 = norm(vec2);

  // Calculate the cosine of the angle
  double cosTheta = dotProduct / (magnitude1 * magnitude2);

  // Clamp cosTheta to avoid numerical errors (values slightly outside [-1, 1])
  cosTheta = std::max(-1.0, std::min(1.0, cosTheta));

  // Calculate the angle in radians
  double angle = std::acos(cosTheta);

  return angle;
}

Vec2d perpendicular_tangent(Point2d p1, Point2d p2) {
  Vec2d vec = p2 - p1;
  return normalize(Vec2d(-vec[1], vec[0]));
}
Matx22d rotationMatrix(double theta) {
  return Matx22d(cos(theta), -sin(theta), sin(theta), cos(theta));
};

// Check if a point is within the boundaries of the map and on white
bool is_free_space(Point2d point, const Mat &map) {
  if (point.x < 0.0 || point.x >= map.cols || point.y < 0.0 ||
      point.y >= map.rows) {
    return false;
  }
  return map.at<Vec3b>(point) == Vec3b(255, 255, 255);
}

bool straight_line_to_goal(Point2d p, Point2d goal, double step_size,
                           const Mat &map) {
  Vec2d direction = normalize((Vec2d)(goal - p));
  Point2d point = p;
  while (is_free_space(point, map)) {
    if (distance(point, goal) < step_size) {
      return true;
    }
    point += (Point2d)direction * step_size;
  }
  return false;
}

double deg2rad(double deg) { return deg * M_PI / 180; }

bool is_at_wall(Point2d p, double step_size, const Mat &map) {
  step_size -= 1.;
  for (int x = -step_size; x <= step_size; x++) {
    for (int y = -step_size; y <= step_size; y++) {
      Point2d point = p + Point2d(x, y);
      if (!is_free_space(point, map)) {
        return true;
      }
    }
  }
  return false;
}

double path_distance(std::vector<Point2d> path) {
  double len = 0.;
  for (size_t i = 0; i < path.size() - 1; i++) {
    len += distance(path.at(i), path.at(i + 1));
  }
  return len;
}

// Function to move the robot directly toward the goal
Point2d move_toward_goal(Point2d current, const Point2d goal, int step_size) {
  Vec2d direction = normalize((Vec2d)(goal - current));
  Point2d new_pos = Point2d(current.x + direction[0] * step_size,
                            current.y + direction[1] * step_size);
  return new_pos;
}

void on_mouse(int event, int x, int y, int flags, void *userdata) {
  (void)flags; // Prevent unused variable warning
  Mat *mat = (Mat *)userdata;
  if (event == EVENT_LBUTTONDOWN) {
    if (is_free_space(Point2d(x, y), *mat)) {
      if (!start_clicked) {
        start_pos = Point2d(x, y);
        start_clicked = true;
        std::println("Start position: ({}, {})\n", start_pos.x, start_pos.y);
      } else if (!end_clicked) {
        end_po = Point2d(x, y);
        end_clicked = true;
        std::println("End position: ({}, {})\n", end_po.x, end_po.y);
      }
    } else {
      std::println("Invalid start position. Please select a white pixel.\n");
    }
  }
}

bool bug0_algorithm(const Mat &map, Mat &final_map, const Point2d start,
                    const Point2d goal, const double step_size = 3.) {
  std::vector<Point2d> path = {start};
  Point2d current = start;
  Point2d last_position = Point2d(-10, -10);
  while (distance(current, goal) > step_size) {
    Point2d goal_dir_pos = move_toward_goal(current, goal, step_size);
    // Draw the line on the final map
    circle(final_map, current, step_size / 1.5, Scalar(0, 0, 255), FILLED);
    Mat temp = final_map.clone();
    if (is_free_space(goal_dir_pos, map) &&
        !is_at_wall(goal_dir_pos, step_size, map)) {
      last_position = current;
      current = goal_dir_pos;
    } else {
      Vec2d dir = normalize((Vec2d)(current - last_position));
      Point2d best_point = current;
      int start_angle = 0;
      if (!is_free_space(current + (Point2d)(dir * step_size * 2), map)) {
        start_angle = 135;
      }
      if (wall_right) {
        for (int i = (135 - start_angle); i > -135; i -= 5) {
          Vec2d r = rotationMatrix(deg2rad(i)) * dir * step_size;
          Point2d np = current + (Point2d)r;
          if (is_free_space(np, map) && is_at_wall(np, step_size, map)) {
            best_point = np;
            break;
          }
        }
      } else {
        for (int i = -(135 - start_angle); i < 135; i += 5) {
          Vec2d r = rotationMatrix(deg2rad(i)) * dir * step_size;
          Point2d np = current + (Point2d)r;
          if (is_free_space(np, map) && is_at_wall(np, step_size, map)) {
            best_point = np;
            break;
          }
        }
      }
      if (best_point != current) {
        last_position = current;
        current = best_point;
      } else {
        std::println("Stuck");
      }
    }
    imshow(WINDOW_NAME, temp);
    if (waitKey(4) == 'q') {
      return false;
    }
    path.push_back(current);
  }

  std::println("Len: {}", path_distance(path));
  return true;
}

int main() {
  // Create a the map
  Mat img_ws1 = Mat::zeros(500, 500, CV_8UC3);
  img_ws1 = Scalar(255, 255, 255);
  Mat img_final;
  // Draw the obstacles
  rectangle(img_ws1, Point2d(100, 100), Point2d(400, 200), Scalar(0), FILLED);
  circle(img_ws1, Point2d(100, 400), 50, Scalar(0), FILLED);
  // Draw a triangle
  Point triangle_pts[1][3];
  triangle_pts[0][0] = Point2d(200, 300);
  triangle_pts[0][1] = Point2d(400, 300);
  triangle_pts[0][2] = Point2d(350, 400);
  const Point *ppt[1] = {triangle_pts[0]};
  int npt[] = {3};
  fillPoly(img_ws1, ppt, npt, 1, Scalar(0));

  img_ws1 = imread("../../../assets/ws5.png");

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
