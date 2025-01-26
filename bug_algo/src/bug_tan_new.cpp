#include <cassert>
#include <cmath>
#include <cstddef>
#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <print>
#include <vector>
using namespace cv;

static const int LIDAR_MAX_DISTANCE = 50;
static const int LIDAR_ANGLE_STEP = 1;

static Point2d s_start_pos;
static Point2d s_goal_pos;
static bool s_start_clicked = false;
static bool s_end_clicked = false;
static const std::string WINDOW_NAME = "Tanget Bug Algorithm";

enum class TanBugMode {
  ToGoal,
  BoundaryFollowing,
};

struct LidarData {
  int angle;
  double distance;
  // Only assigned when distance is under LIDAR_MAX_DISTANCE
  Point2d position = Point2d();
};

double distance(Point2d p1, Point2d p2) { return norm(p2 - p1); }

double distance(Vec2d p1, Vec2d p2) { return norm(p2 - p1); }

Vec2d perpendicular_tangent(Point2d p1, Point2d p2) {
  Vec2d vec = p2 - p1;
  return normalize(Vec2d(-vec[1], vec[0]));
}

// Check if a point is within the boundaries of the map and on white
bool is_free_space(Point2d point, const Mat &map) {
  if (point.x < 0.0 || point.x >= map.cols || point.y < 0.0 ||
      point.y >= map.rows) {
    return false;
  }
  return map.at<Vec3b>(point) == Vec3b(255, 255, 255);
}

double deg2rad(double deg) { return deg * M_PI / 180; }

LidarData find_closest_q_points(std::vector<LidarData> lidar, Point2d goal) {
  // Find closest obstacle egde to goal else go to BoundaryFollowing
  // Goal is always first element
  LidarData goal_dir = lidar.at(0);
  if (goal_dir.distance == LIDAR_MAX_DISTANCE) {
    // Free to goal, obstacle check already done
    return goal_dir;
  }
  std::vector<LidarData> q_points;
  for (size_t i = 0; i < lidar.size(); i++) {
    size_t left_idx = i == 0 ? lidar.size() - 1 : i - 1;
    size_t right_idx = i == lidar.size() - 1 ? 0 : i + 1;

    // Find the Q-points
    int i_dist = lidar.at(i).distance;
    int left_dist = lidar.at(left_idx).distance;
    int right_dist = lidar.at(right_idx).distance;
    if (i_dist != LIDAR_MAX_DISTANCE &&
        (i_dist > left_dist || left_dist == LIDAR_MAX_DISTANCE) &&
        (i_dist > right_dist || right_dist == LIDAR_MAX_DISTANCE)) {
      q_points.push_back(lidar.at(i));
    }
  }

  if (q_points.size() == 0) {
    q_points.push_back(lidar.at(0));
  }

  LidarData closest_q_point = q_points.at(0);
  // Use the heuristic: d(x,Qi) + d(Qi,goal)
  double closest_q_point_dist =
      distance(closest_q_point.position, goal) + closest_q_point.distance;
  for (const auto p : q_points) {
    if ((distance(p.position, goal) + p.distance) < closest_q_point_dist) {
      closest_q_point = p;
    }
  }
  return closest_q_point;
}
LidarData find_closest_lidar(std::vector<LidarData> lidar) {
  // Find closest obstacle egde to goal else go to BoundaryFollowing
  LidarData closest = lidar.at(0);
  for (size_t i = 0; i < lidar.size(); i++) {
    if (lidar.at(i).distance < closest.distance) {
      closest = lidar.at(i);
    }
  }
  return closest;
}

bool bug_tan_algorithm(const Mat &map, Mat &final_map, const Point2d start,
                       const Point2d goal, const bool animate = true,
                       const double step_size = 4.0) {
  TanBugMode state = TanBugMode::ToGoal;
  Point2d cur_pos = start;
  Vec2d cur_dir = normalize((Vec2d)(goal - cur_pos));

  double dist_reach = distance(start, goal);
  double dist_followed = dist_reach;

  std::vector<Point2d> visited_positions;

  Mat lidar_map;

  while (distance(cur_pos, goal) > step_size) {
    circle(final_map, cur_pos, 2, Scalar(0, 255, 0), FILLED);
    // Only clone if we want to animate.
    if (animate) {
      lidar_map = final_map.clone();
      circle(lidar_map, cur_pos, 5, Scalar(0, 255, 0), FILLED);
    }
    // Lidar scan stores (angle, distance)
    std::vector<LidarData> lidar;

    Vec2d goal_dir = normalize((Vec2d)(goal - cur_pos));
    for (size_t angle = 0; angle < 360; angle += LIDAR_ANGLE_STEP) {
      // angle = 0 is goal direction.
      double theta = deg2rad(angle);
      Matx22d rotationMatrix(cos(theta), -sin(theta), sin(theta), cos(theta));

      Vec2d direction_i = rotationMatrix * goal_dir;

      double max_steps = LIDAR_MAX_DISTANCE;
      for (double len = 0.0; len < max_steps; len += 0.3) {

        Vec2d dir_len = direction_i * len;
        Point2d lidar_point = cur_pos + (Point2d)dir_len;
        lidar_point.x = round(lidar_point.x);
        lidar_point.y = round(lidar_point.y);

        if (!is_free_space(lidar_point, map)) {
          max_steps = len;
          break;
        }
      }
      Point2d direction_end = cur_pos + (Point2d)(direction_i * max_steps);
      lidar.push_back(LidarData(angle, max_steps, direction_end));
      if (animate) {
        // Blue color if it is hitting the obstacle
        if (max_steps < LIDAR_MAX_DISTANCE) {
          Scalar color = Scalar(255, 0, 0);
          circle(lidar_map, direction_end, 2, color, FILLED);
        }
      }
    }

    // Go straight to Goal or nearest Q point to goal
    switch (state) {
    case TanBugMode::ToGoal: {
      LidarData best_q_point = find_closest_q_points(lidar, goal);

      LidarData closest_obstacle = find_closest_lidar(lidar);
      if ((best_q_point.angle == 0 &&
           best_q_point.distance == LIDAR_MAX_DISTANCE) ||
          distance(cur_pos, goal) < LIDAR_MAX_DISTANCE) {
        if (closest_obstacle.distance < LIDAR_MAX_DISTANCE / 2. &&
            distance(cur_pos, goal) >= LIDAR_MAX_DISTANCE) {
          goal_dir -=
              normalize((Vec2d)(closest_obstacle.position - cur_pos)) * 0.8;
          goal_dir = normalize(goal_dir);
        }
        cur_dir = goal_dir;
        cur_pos += (Point2d)(goal_dir * step_size);
        putText(lidar_map, "Using goal", Point(final_map.cols / 2, 100),
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 2);
      } else {
        putText(lidar_map, "Using Q points", Point(final_map.cols / 2, 100),
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 2);
        // Check if we are moving closer
        double best_q_point_hueristic =
            distance(best_q_point.position, goal) + best_q_point.distance;
        if (best_q_point_hueristic < dist_followed) {
          dist_followed = best_q_point_hueristic;
        } else {
          state = TanBugMode::BoundaryFollowing;
          break;
        }

        // Follow perpendicular with the obstacle
        Vec2d perp_tang =
            perpendicular_tangent(cur_pos, closest_obstacle.position);

        circle(lidar_map, closest_obstacle.position, 5, Scalar(255, 0, 0),
               FILLED);
        // Dot product to see if they point the same dir.
        double dot = perp_tang.dot(goal_dir);
        double dir = 1;
        if (dot < 0) {
          dir = -1;
        }
        if (closest_obstacle.distance < LIDAR_MAX_DISTANCE / 2.) {
          perp_tang -=
              normalize((Vec2d)(closest_obstacle.position - cur_pos)) * 0.2;
          normalize(perp_tang);
        }
        cur_dir = perp_tang * dir;
        cur_pos = cur_pos + (Point2d)(perp_tang * dir * step_size);
      }
      break;
    }

    case TanBugMode::BoundaryFollowing: {
      LidarData closest_obstacle = find_closest_lidar(lidar);

      // Follow perpendicular with the obstacle
      Vec2d perp_tang =
          perpendicular_tangent(cur_pos, closest_obstacle.position);

      // Dot product to see if they point the same dir.
      double dot = perp_tang.dot(cur_dir);
      double dir = 1;
      if (dot < 0) {
        dir = -1;
      }
      if (closest_obstacle.distance < LIDAR_MAX_DISTANCE / 2.) {
        perp_tang -=
            normalize((Vec2d)(closest_obstacle.position - cur_pos)) * 0.5;
        normalize(perp_tang);
      } else if (closest_obstacle.distance == LIDAR_MAX_DISTANCE) {
        std::println("Too far..");
        state = TanBugMode::ToGoal;
        break;
      }
      cur_dir = perp_tang * dir;
      cur_pos = cur_pos + (Point2d)(perp_tang * dir * step_size);

      dist_reach = distance(cur_pos, goal);
      if (dist_reach <= dist_followed) {
        state = TanBugMode::ToGoal;
      }

      break;
    }
    }

    if (animate) {
      std::string text;
      switch (state) {
      case TanBugMode::ToGoal:
        text = "Going to Goal";
        break;
      case TanBugMode::BoundaryFollowing:
        text = "Following Boundary";
        break;
      }
      putText(lidar_map, text, Point(final_map.cols / 2 - 15, 50),
              FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 2);
      imshow(WINDOW_NAME, lidar_map);
    }
    if (waitKey(10) == 'q') {
      return false;
    }
  }

  return true;
}

void on_mouse(int event, int x, int y, int flags, void *userdata) {
  (void)flags; // Prevent unused variable warning
  Mat *mat = (Mat *)userdata;
  if (event == EVENT_LBUTTONDOWN) {
    if (is_free_space(Point2d(x, y), *mat)) {
      if (!s_start_clicked) {
        s_start_pos = Point2d(x, y);
        s_start_clicked = true;
        std::println("Start position: ({}, {})", s_start_pos.x, s_start_pos.y);
      } else if (!s_end_clicked) {
        s_goal_pos = Point2d(x, y);
        s_end_clicked = true;
        std::println("End position: ({}, {})", s_goal_pos.x, s_goal_pos.y);
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
      if (s_start_clicked) {
        circle(img_final, s_start_pos, 5, Scalar(0, 0, 255), FILLED);
      }
      if (s_end_clicked) {
        circle(img_final, s_goal_pos, 5, Scalar(0, 0, 255), FILLED);
        img_final = img_final.clone();
        break;
      }
      imshow(WINDOW_NAME, img_final);
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
                 distance(s_start_pos, s_goal_pos));
    std::println("Starting tangent bug algorithm...");
    // Run the bug algorithm and print the result
    if (bug_tan_algorithm(img_ws1, img_final, s_start_pos, s_goal_pos, true)) {
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
      s_start_clicked = false;
      s_end_clicked = false;
    }
  }
  destroyAllWindows();
  return 0;
}
