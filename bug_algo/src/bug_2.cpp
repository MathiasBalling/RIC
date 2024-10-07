#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;

const int STEP_SIZE = 5;

Point start_pos;
Point goal_pos;
bool start_clicked = false;
bool end_clicked = false;
const std::string WINDOW_NAME = "Bug 2 Algorithm";

// Calculate Euclidean distance between two points
double distance(Point2i p1, Point2i p2) {
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

// Function to normalize a 2D vector
Vec2f normalize(Vec2f vec) { return vec / norm(vec); }

// Check if a point is within the boundaries of the map and is on white (free
// space)
bool is_valid(Point2i point, const Mat &map) {
  if (point.x < 0 || point.x >= map.cols || point.y < 0 ||
      point.y >= map.rows) {
    return false;
  }
  return map.at<Vec3b>(point) ==
         Vec3b(255, 255, 255); // Only walkable on white areas
}

// Convert degrees to radians
double deg2rad(double deg) { return deg * M_PI / 180; }

// Check if a point is on the m-line (line from start to goal)
bool on_m_line(const Point2i &point, const Point2i &start,
               const Point2i &goal) {
  double m = static_cast<double>(goal.y - start.y) /
             (goal.x - start.x + 1e-5); // Slope of m-line
  double expected_y = m * (point.x - start.x) + start.y;
  return abs(point.y - expected_y) < 5.0; // Tolerance for pixel precision
}

// Boundary following function
Point2i follow_boundary(const Point2i &current_pos, const Mat &map) {
  // Check 8-neighborhood (N, NE, E, SE, S, SW, W, NW) around the current
  // position
  std::vector<Point2i> neighbors = {
      Point2i(0, -STEP_SIZE), Point2i(STEP_SIZE, -STEP_SIZE),
      Point2i(STEP_SIZE, 0),  Point2i(STEP_SIZE, STEP_SIZE),
      Point2i(0, STEP_SIZE),  Point2i(-STEP_SIZE, STEP_SIZE),
      Point2i(-STEP_SIZE, 0), Point2i(-STEP_SIZE, -STEP_SIZE)};

  for (const auto &neighbor : neighbors) {
    Point2i new_pos = current_pos + neighbor;
    if (is_valid(new_pos, map)) {
      return new_pos; // Return first valid position found
    }
  }
  return current_pos; // If no valid move is found, stay at current position
}

// The Bug 2 algorithm
bool bug_2_algorithm(const Mat &map, Mat &final_map, const Point2i start,
                     const Point2i goal) {
  Point2i current_pos = start;
  Point2i hit_point = current_pos; // Point where the robot hits an obstacle
  bool following_obstacle = false; // Are we currently following an obstacle?
  bool found_closer_m_line = false;

  double min_dist_to_goal_on_hit = distance(start, goal);

  while (distance(current_pos, goal) > STEP_SIZE) {
    if (!following_obstacle && on_m_line(current_pos, start, goal)) {
      // Move along the m-line towards the goal
      Vec2f direction =
          normalize(Vec2f(goal.x - current_pos.x, goal.y - current_pos.y));
      Point2i next_pos = current_pos + Point2i(direction[0] * STEP_SIZE,
                                               direction[1] * STEP_SIZE);

      // Check if next position is valid
      if (is_valid(next_pos, map)) {
        current_pos = next_pos;
      } else {
        // Start boundary following when obstacle is encountered
        std::print("Obstacle encountered at ({}, {}), starting boundary "
                   "following...\n",
                   current_pos.x, current_pos.y);
        following_obstacle = true;
        hit_point = current_pos;
        min_dist_to_goal_on_hit = distance(current_pos, goal);
      }
    } else {
      // Follow the boundary of the obstacle
      Point2i next_pos = follow_boundary(current_pos, map);

      // Check if we're back on the m-line closer to the goal than when we hit
      // the obstacle
      if (on_m_line(next_pos, start, goal) &&
          distance(next_pos, goal) < min_dist_to_goal_on_hit) {
        std::print("Returning to m-line at ({}, {})\n", next_pos.x, next_pos.y);
        following_obstacle = false; // Resume m-line movement
        found_closer_m_line = true;
      } else {
        current_pos = next_pos; // Continue following the boundary
      }

      // Stop if we're stuck
      if (current_pos == hit_point && !found_closer_m_line) {
        std::print("Goal unreachable due to obstacle!\n");
        return false;
      }
    }

    // Draw the current path on the map
    circle(final_map, current_pos, 2, Scalar(255, 0, 0), FILLED);
    imshow(WINDOW_NAME, final_map);
    if (waitKey(50) == 'q') {
      return false;
    }
  }

  return true;
}

// Mouse callback function for selecting start and goal points
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
        goal_pos = Point(x, y);
        end_clicked = true;
        std::println("End position: ({}, {})\n", goal_pos.x, goal_pos.y);
      }
    } else {
      std::println(
          "Invalid start/goal position. Please select a white pixel.\n");
    }
  }
}

// Function to create a sample map with obstacles
Mat make_map() {
  Mat img_ws1 = Mat::zeros(500, 500, CV_8UC3);
  img_ws1 = Scalar(255, 255, 255); // White background
  rectangle(img_ws1, Point(100, 100), Point(400, 200), Scalar(0),
            FILLED); // Black rectangle obstacle
  circle(img_ws1, Point(100, 400), 50, Scalar(0),
         FILLED); // Black circle obstacle
  Point triangle_pts[1][3] = {
      {Point(200, 300), Point(400, 300), Point(350, 400)}};
  const Point *pts[1] = {triangle_pts[0]};
  int npt[] = {3};
  fillPoly(img_ws1, pts, npt, 1, Scalar(0)); // Black triangle obstacle
  return img_ws1;
}

int main() {
  // Create the map
  Mat img_ws1 = make_map();
  Mat img_final = img_ws1.clone();

  // Create window and set mouse callback to select start/goal points
  namedWindow(WINDOW_NAME);
  setMouseCallback(WINDOW_NAME, on_mouse, &img_ws1);

  while (true) {
    Mat img_ws1_temp = img_ws1.clone();
    if (start_clicked) {
      circle(img_ws1_temp, start_pos, 5, Scalar(0, 0, 255),
             FILLED); // Mark start point
    }
    if (end_clicked) {
      circle(img_ws1_temp, goal_pos, 5, Scalar(0, 0, 255),
             FILLED); // Mark goal point
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

  // Run the Bug 2 algorithm
  std::println("Analyzing the image...");
  std::println("Distance to goal: {:.2f}", distance(start_pos, goal_pos));
  std::println("Starting Bug 2 algorithm...");

  bool success = bug_2_algorithm(img_ws1, img_final, start_pos, goal_pos);

  if (success) {
    std::println("Path found to goal!");
  } else {
    std::println("Goal is unreachable.");
  }

  // Display the final path
  imshow(WINDOW_NAME, img_final);
  waitKey(0);

  return 0;
}
