#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <format>
#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <print>
#include <unordered_set>
#include <vector>
using namespace cv;

static const double LIDAR_ANGLE_STEP = 1.;
static const double LIDAR_DISTANCE_STEP = 0.1;
static const double LIDAR_MAX_DISTANCE = 50.;
static const double MIN_DISTANCE_WALL = LIDAR_MAX_DISTANCE / 3.;

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

double deg2rad(double deg) { return deg * M_PI / 180; }

LidarData find_closest_q_point(std::vector<LidarData> lidar, Point2d goal) {
  // Find closest obstacle egde to goal else go to BoundaryFollowing
  std::vector<LidarData> q_points;
  for (size_t i = 0; i < lidar.size(); i++) {
    size_t left_idx = i == 0 ? lidar.size() - 1 : i - 1;
    size_t right_idx = i == lidar.size() - 1 ? 0 : i + 1;

    // Find the Q-points
    double i_dist = lidar.at(i).distance;
    double left_dist = lidar.at(left_idx).distance;
    double right_dist = lidar.at(right_idx).distance;
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
  for (const auto q : q_points) {
    double p_dist = distance(q.position, goal) + q.distance;
    if (p_dist < closest_q_point_dist) {
      closest_q_point = q;
      closest_q_point_dist = p_dist;
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
  Vec2d last_dir = normalize((Vec2d)(cur_pos - goal));

  double dist_reach = distance(start, goal);
  double dist_followed = dist_reach;

  std::unordered_set<std::string> visited_positions;
  visited_positions.insert(std::format("{},{}", (int)start.x, (int)start.y));

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

      Vec2d direction_i = rotationMatrix(theta) * goal_dir;

      double max_steps = LIDAR_MAX_DISTANCE;
      Point2d direction_end;
      for (double len = LIDAR_DISTANCE_STEP; len < max_steps;
           len += LIDAR_DISTANCE_STEP) {
        Vec2d dir_len = direction_i * len;
        direction_end = cur_pos + (Point2d)dir_len;

        if (!is_free_space(direction_end, map)) {
          direction_end = cur_pos + (Point2d)dir_len;
          max_steps = len;
          break;
        }
      }
      lidar.push_back(LidarData(angle, max_steps, direction_end));
      if (animate) {
        // Blue color if it is hitting the obstacle
        if (max_steps < LIDAR_MAX_DISTANCE) {
          Scalar color = Scalar(0, 0, 255);
          circle(lidar_map, direction_end, 2, color, FILLED);
        }
      }
    }

    // Go straight to Goal or nearest Q point to goal
    switch (state) {
    case TanBugMode::ToGoal: {
      LidarData closest_obstacle = find_closest_lidar(lidar);
      LidarData goal_dir_lidar = lidar.at(0);
      if ((goal_dir_lidar.angle == 0 &&
           goal_dir_lidar.distance >= LIDAR_MAX_DISTANCE) ||
          distance(cur_pos, goal) < LIDAR_MAX_DISTANCE) {
        ///////////// Direct to goal /////////////
        Vec2d wall_vec =
            normalize((Vec2d)(closest_obstacle.position - cur_pos)) * 0.5;
        // Move away from wall if we get too close
        if (closest_obstacle.distance < MIN_DISTANCE_WALL &&
            distance(cur_pos, goal) >= LIDAR_MAX_DISTANCE) {
          goal_dir = normalize(goal_dir - wall_vec);
        }
        last_dir = cur_dir;
        cur_dir = goal_dir;
        cur_pos += (Point2d)(goal_dir * step_size);
        putText(lidar_map, "Using goal", Point(final_map.cols / 2, 100),
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 2);
        if (visited_positions.contains(
                std::format("{},{}", (int)cur_pos.x, (int)cur_pos.y))) {
          return false;
        } else {
          visited_positions.insert(
              std::format("{},{}", (int)cur_pos.x, (int)cur_pos.y));
        }
      } else {
        ///////////// Using Q-points /////////////
        LidarData best_q_point = find_closest_q_point(lidar, goal);
        circle(lidar_map, best_q_point.position, 5, Scalar(255, 0, 0), FILLED);
        putText(lidar_map, "Using Q points", Point(final_map.cols / 2, 100),
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 2);
        // Check if we are moving closer
        double best_q_point_hueristic =
            distance(best_q_point.position, goal) + best_q_point.distance;
        if (best_q_point_hueristic <= dist_followed) {
          dist_followed = best_q_point_hueristic;
        } else {
          state = TanBugMode::BoundaryFollowing;
          break;
        }
        // Follow perpendicular with the obstacle
        Vec2d perp_tang =
            perpendicular_tangent(cur_pos, closest_obstacle.position);
        // Dot product to see if they point the same dir.
        double dot = perp_tang.dot(best_q_point.position - cur_pos);
        double dir = dot < 0. ? -1. : 1.;
        Vec2d wall_vec =
            normalize((Vec2d)(closest_obstacle.position - cur_pos)) * 0.15 *
            dir;
        if (closest_obstacle.distance < MIN_DISTANCE_WALL) {
          perp_tang = normalize(perp_tang - wall_vec);
        } else {
          perp_tang = normalize(perp_tang + wall_vec);
        }
        last_dir = cur_dir;
        cur_dir = perp_tang * dir;
        cur_pos = cur_pos + (Point2d)(perp_tang * dir * step_size);
        if (visited_positions.contains(
                std::format("{},{}", (int)cur_pos.x, (int)cur_pos.y))) {
          return false;
        } else {
          visited_positions.insert(
              std::format("{},{}", (int)cur_pos.x, (int)cur_pos.y));
        }
      }
      break;
    }

    case TanBugMode::BoundaryFollowing: {
      LidarData closest_obstacle = find_closest_lidar(lidar);
      LidarData best_q_point = find_closest_q_point(lidar, goal);

      // Follow perpendicular with the obstacle
      Vec2d perp_tang =
          perpendicular_tangent(cur_pos, closest_obstacle.position);

      // Dot product to see if they point the same dir.
      double dot = perp_tang.dot(cur_dir);
      double dir = dot < 0 ? -1 : 1;
      // Point away from the wall if perpendicular_tangent is not good
      // engouh
      Vec2d wall_vec =
          normalize((Vec2d)(closest_obstacle.position - cur_pos)) * 0.15 * dir;
      if (closest_obstacle.distance >= LIDAR_MAX_DISTANCE) {
        // If we get too far away from the wall.
        std::println("Too far..");
        // Change state to get back on track
        state = TanBugMode::ToGoal;
        break;
      } else if (closest_obstacle.distance < MIN_DISTANCE_WALL) {
        // Move away from wall
        perp_tang = normalize(perp_tang - wall_vec);
      } else {
        // Move closer to wall
        perp_tang = normalize(perp_tang + wall_vec);
      }

      // Update position
      last_dir = cur_dir;
      cur_dir = perp_tang * dir;
      // Large changes in angle indicate a corner
      if (abs(calculateAngle(cur_dir, last_dir)) > M_PI / 3.) {
        Vec2d ccw_dir = rotationMatrix(M_PI / 2.) * last_dir;
        Vec2d cw_dir = rotationMatrix(-M_PI / 2.) * last_dir;
        double cur_len = LIDAR_MAX_DISTANCE;
        double ccw_len = LIDAR_MAX_DISTANCE;
        double cw_len = LIDAR_MAX_DISTANCE;
        for (double len = LIDAR_DISTANCE_STEP; len < LIDAR_MAX_DISTANCE;
             len += LIDAR_DISTANCE_STEP) {
          Point2d new_pos = cur_pos + (Point2d)(cur_dir * len);
          Point2d ccw_pos = cur_pos + (Point2d)(ccw_dir * len);
          Point2d cw_pos = cur_pos + (Point2d)(cw_dir * len);

          if (cur_len == LIDAR_MAX_DISTANCE && !is_free_space(new_pos, map)) {
            cur_len = len;
          }
          if (ccw_len == LIDAR_MAX_DISTANCE && !is_free_space(ccw_pos, map)) {
            ccw_len = len;
          }
          if (cw_len == LIDAR_MAX_DISTANCE && !is_free_space(cw_pos, map)) {
            cw_len = len;
          }
        }
        double max = std::max(cur_len, std::max(ccw_len, cw_len));
        if (max == ccw_len) {
          cur_dir = ccw_dir;
        } else if (max == cw_len) {
          cur_dir = cw_dir;
        }
        // Else keep original
      }
      Point2d temp_pos = cur_pos;
      cur_pos = cur_pos + (Point2d)(perp_tang * dir * step_size);
      if (!is_free_space(cur_pos, map)) {
        // Use old pos for now
        std::println("OHH not a good direction");
        cur_pos = temp_pos;
      }

      dist_reach =
          distance(best_q_point.position, goal) + best_q_point.distance;
      if (dist_reach <= (dist_followed - step_size)) {
        state = TanBugMode::ToGoal;
      }

      if (visited_positions.contains(
              std::format("{},{}", (int)cur_pos.x, (int)cur_pos.y))) {
        return false;
      } else {
        visited_positions.insert(
            std::format("{},{}", (int)cur_pos.x, (int)cur_pos.y));
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
    if (waitKey(5) == 'q') {
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
  Mat map = Mat::zeros(500, 500, CV_8UC3);
  map = Scalar(255, 255, 255);
  rectangle(map, Point(100, 100), Point(400, 200), Scalar(0), FILLED);
  circle(map, Point(100, 400), 50, Scalar(0), FILLED);
  Point triangle_pts[1][3];
  triangle_pts[0][0] = Point(200, 200);
  triangle_pts[0][1] = Point(400, 240);
  triangle_pts[0][2] = Point(350, 400);
  const Point *ppt[1] = {triangle_pts[0]};
  int npt[] = {3};
  fillPoly(map, ppt, npt, 1, Scalar(0));
  return map;
}

int main() {
  // Create a the map
  Mat img_ws1 = make_map();
  // Optionally load an image
  img_ws1 = imread("../../../assets/ws5.png");
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
