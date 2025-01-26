#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;

static const int LIDAR_MAX_DISTANCE = 50;
static const int LIDAR_ANGLE_STEP = 5;

static Point start_pos;
static Point goal_pos;
static bool start_clicked = false;
static bool end_clicked = false;
static const std::string WINDOW_NAME = "Tanget Bug Algorithm";

enum class TanBugMode {
  ToGoal,
  BoundaryFollowing,
};

double distance(Point2i p1, Point2i p2) {
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

double distance(Vec2f p1, Vec2f p2) { return norm(p2 - p1); }

Vec2f normalize(Vec2f vec) { return vec / norm(vec); }

// Check if a point is within the boundaries of the map and on white
bool is_free_space(Point2i point, const Mat &map) {
  if (point.x < 0 || point.x >= map.cols || point.y < 0 ||
      point.y >= map.rows) {
    return false;
  }
  return map.at<Vec3b>(point) == Vec3b(255, 255, 255);
}

double deg2rad(double deg) { return deg * M_PI / 180; }

bool bug_tan_algorithm(const Mat &map, Mat &final_map, const Point2i start,
                       const Point2i goal, const bool animate = true,
                       const int step_size = 5) {
  Vec2f goal_pos = Vec2f(goal.x, goal.y);
  Vec2f last_direction = Vec2f(0, 0);
  Point2i current_pos_point = start;
  Point2i last_pos_point = goal;

  while (distance(current_pos_point, goal) > step_size) {
    Vec2f current_pos = Vec2f(current_pos_point.x, current_pos_point.y);
    // Normalized direction vector
    Vec2f direction = normalize(goal_pos - current_pos);

    // Temporary map to draw the lidar
    Mat temp_map = final_map.clone();
    // Test how far the direction vector can go
    float max_distance = distance(current_pos, goal_pos);
    size_t i = 1;
    while (true) {
      Point2i new_pos =
          current_pos_point +
          Point2i(direction[0] * step_size * i, direction[1] * step_size * i);
      if (!is_free_space(new_pos, map)) {
        max_distance = distance(current_pos, Vec2f(new_pos.x, new_pos.y));
        break;
      } else if (distance(new_pos, goal) < step_size) {
        break;
      }
      i++;
    }
    Vec2f direction_result = direction * max_distance;
    for (size_t i = 0; i < 360; i += LIDAR_ANGLE_STEP) {
      Vec2f direction_i = direction;
      double theta = deg2rad(i);
      Matx22f rotationMatrix(cos(theta), -sin(theta), sin(theta), cos(theta));
      direction_i = rotationMatrix * direction_i;
      uint16_t max_steps = LIDAR_MAX_DISTANCE;
      for (size_t j = 0; j < max_steps; j++) {
        Vec2f new_pos = current_pos + direction_i * (float)j;
        if (!is_free_space(Point2i(new_pos[0], new_pos[1]), map)) {
          max_steps = j;
          float weighted_direction =
              (LIDAR_MAX_DISTANCE - distance(new_pos, current_pos)) /
              LIDAR_MAX_DISTANCE;
          weighted_direction = pow(weighted_direction * 3, 3);
          direction_result -= direction_i * weighted_direction;
          break;
        }
      }
      if (animate) {
        // Draw the direction vector
        Point2i direction_end =
            current_pos_point +
            Point2i(direction_i[0] * max_steps, direction_i[1] * max_steps);
        // Red color if it is hitting the obstacle
        Scalar color = Scalar(0, 255, 0);
        if (max_steps < LIDAR_MAX_DISTANCE) {
          color = Scalar(0, 0, 255);
        }
        line(temp_map, current_pos_point, direction_end, color, 1);
      }
    }
    direction_result += last_direction * 150;

    if (animate) {
      imshow(WINDOW_NAME, temp_map);
    }
    if (waitKey(1) == 'q') {
      return false;
    }
    direction_result = normalize(direction_result);
    Point2i next_pos_point =
        current_pos_point + Point2i(direction_result[0] * step_size,
                                    direction_result[1] * step_size);
    bool has_visited =
        distance(next_pos_point, last_pos_point) < (step_size * 1.5);
    if (has_visited || !is_free_space(next_pos_point, map)) {
      double min_distance = std::numeric_limits<double>::max();
      Point2i closest_point = current_pos_point;
      for (int i = 0; i < 360; i += 10) {
        Vec2f direction_i = direction;
        double theta = deg2rad(i);
        Matx22f rotationMatrix(cos(theta), -sin(theta), sin(theta), cos(theta));
        direction_i = rotationMatrix * direction_i;
        Point2i point = current_pos_point + Point2i(direction_i[0] * step_size,
                                                    direction_i[1] * step_size);
        if (is_free_space(point, map)) {
          double dist = distance(point, goal);
          bool has_visited =
              distance(point, last_pos_point) < (step_size * 1.5);
          // Should be further than the step size away
          if (dist < min_distance && !has_visited) {
            min_distance = dist;
            closest_point = point;
          }
        }
      }
      last_pos_point = current_pos_point;
      current_pos_point = closest_point;
    } else {
      last_direction = direction_result;
      last_pos_point = current_pos_point;
      current_pos_point = next_pos_point;
    }
    // Line marking the path taken
    line(final_map, last_pos_point, current_pos_point, Scalar(255, 0, 0), 2);
  }

  return true;
}

void on_mouse(int event, int x, int y, int flags, void *userdata) {
  (void)flags; // Prevent unused variable warning
  Mat *mat = (Mat *)userdata;
  if (event == EVENT_LBUTTONDOWN) {
    if (is_free_space(Point2i(x, y), *mat)) {
      if (!start_clicked) {
        start_pos = Point(x, y);
        start_clicked = true;
        std::println("Start position: ({}, {})\n", start_pos.x, start_pos.y);
      } else if (!end_clicked) {
        goal_pos = Point(x, y);
        end_clicked = true;
        std::println("End position: ({}, {})\n", goal_pos.x, goal_pos.y);
      }
    } else {
      std::println("Invalid start position. Please select a white pixel.\n");
    }
  }
}

Mat make_map() {
  Mat img_ws1 = Mat::zeros(500, 500, CV_8UC3);
  img_ws1 = Scalar(255, 255, 255);
  rectangle(img_ws1, Point(100, 100), Point(400, 200), Scalar(0), FILLED);
  circle(img_ws1, Point(100, 400), 50, Scalar(0), FILLED);
  Point triangle_pts[1][3];
  triangle_pts[0][0] = Point(200, 200);
  triangle_pts[0][1] = Point(400, 240);
  triangle_pts[0][2] = Point(350, 400);
  const Point *ppt[1] = {triangle_pts[0]};
  int npt[] = {3};
  fillPoly(img_ws1, ppt, npt, 1, Scalar(0));
  return img_ws1;
}

int main() {
  // Create a the map
  Mat img_ws1 = make_map();
  // Optionally load an image
  img_ws1 = imread("../../../assets/ws4.png");
  while (1) {
    // Make a copy of the map for the final path
    Mat img_final = img_ws1.clone();

    // Make window and get start/end points
    namedWindow(WINDOW_NAME);                          // Create the window
    setMouseCallback(WINDOW_NAME, on_mouse, &img_ws1); // Set the on_mouse
    while (true) {
      // Clone the edges image
      Mat img_ws1_temp = img_ws1.clone();
      if (start_clicked) {
        circle(img_ws1_temp, start_pos, 5, Scalar(0, 0, 255), FILLED);
      }
      if (end_clicked) {
        circle(img_ws1_temp, goal_pos, 5, Scalar(0, 0, 255), FILLED);
        img_final = img_ws1_temp.clone();
        break;
      }
      imshow(WINDOW_NAME, img_ws1_temp);
      if (waitKey(100) == 'q') {
        destroyAllWindows();
        return 0;
      }
    }
    // Remove the mouse callback
    setMouseCallback(WINDOW_NAME, NULL, NULL);

    // Analyze the image
    std::println("Analyzing the image...");
    std::println("Straight distance to goal: {:.2f}",
                 distance(start_pos, goal_pos));
    std::println("Starting tangent bug algorithm...");
    // Run the bug algorithm and print the result
    if (bug_tan_algorithm(img_ws1, img_final, start_pos, goal_pos, false)) {
      std::println("Goal reached!");
    } else {
      std::println("Goal not reached!");
    }

    // Show the final image
    imshow(WINDOW_NAME, img_final);
    if (waitKey(0) == 'q') {
      break;
    } else {
      // Reset for new run
      start_clicked = false;
      end_clicked = false;
    }
  }
  destroyAllWindows();
  return 0;
}
