use std::collections::VecDeque;

use opencv::{
    core::{Mat, MatTrait, MatTraitConst, Point2i, Scalar, Vec3b},
    highgui::{imshow, move_window, named_window, wait_key},
};

use crate::util::{is_obstacle, is_valid_position};

const WINDOW_NAME: &str = "Brushfire";

pub fn brushfire(img: &Mat) -> (Mat, VecDeque<Point2i>) {
    println!("Performing brushfire algorithm");

    let mut distance_map =
        Mat::new_rows_cols_with_default(img.rows(), img.cols(), img.typ(), Scalar::all(255.0))
            .unwrap();
    let mut obstacle_queue = VecDeque::<Point2i>::new();

    // Directions for 4-connected neighbors
    let directions = [(1, 0), (-1, 0), (0, 1), (0, -1)];

    // Initialize the queue with all obstacle positions and boundary pixels
    let (rows, cols) = (img.rows(), img.cols());
    for y in 0..rows {
        for x in 0..cols {
            let pos = Point2i::new(x, y);
            if is_obstacle(img, pos) || x == 0 || y == 0 || x == cols - 1 || y == rows - 1 {
                obstacle_queue.push_back(pos);
                let pixel = distance_map.at_2d_mut::<Vec3b>(y, x).unwrap();
                *pixel = Vec3b::all(0);
            }
        }
    }

    // Copy the queue for later use in the GVD algorithm
    let obstacle_queue_copy = obstacle_queue.clone();

    // Breadth-First Search (BFS) to propagate distances
    while let Some(pos) = obstacle_queue.pop_front() {
        let current_distance = distance_map.at_2d::<Vec3b>(pos.y, pos.x).unwrap()[0] as i32;

        // Find neighbors
        for dir in &directions {
            let neighbor = Point2i::new(pos.x + dir.0, pos.y + dir.1);

            // Check if the neighbor is within the image boundaries
            if is_valid_position(img, neighbor) {
                let neighbor_pixel = distance_map.at_2d::<Vec3b>(neighbor.y, neighbor.x).unwrap();
                let neighbor_distance = neighbor_pixel[0] as i32;

                // Check if the distance should be updated
                if neighbor_distance > current_distance + 1 {
                    // Check if the distance is within 8-bit range
                    assert!(current_distance + 1 < 256);

                    let neighbor_pixel_mut = distance_map
                        .at_2d_mut::<Vec3b>(neighbor.y, neighbor.x)
                        .unwrap();
                    *neighbor_pixel_mut = Vec3b::all((current_distance + 1) as u8);
                    obstacle_queue.push_back(neighbor);
                }
            }
        }
    }

    // Make window to show the brushfire algorithm
    named_window(WINDOW_NAME, 0).expect("failed to create Brushfire window");
    move_window(WINDOW_NAME, 650, 0).expect("failed to move window");
    imshow(WINDOW_NAME, &distance_map).expect("failed to show Brushfire image");
    wait_key(1).expect("failed to wait key");

    // Return
    (distance_map, obstacle_queue_copy)
}
