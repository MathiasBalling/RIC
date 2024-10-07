use std::collections::VecDeque;

use opencv::{
    core::{self, Mat, Point2i, Scalar, Vec3b},
    highgui::{imshow, move_window, named_window, wait_key},
    prelude::*,
};

use crate::util::{self, is_valid_position};

const WINDOW_NAME: &str = "Generalized Voronoi Diagram (GVD)";

const UNINITIALIZED_LABEL: u8 = 255;
const EDGE_LABEL: u8 = 254;

const DIRECTIONS: [(i32, i32); 4] = [(0, 1), (1, 0), (0, -1), (-1, 0)];

pub fn gvd(img: &Mat, dtm_img: &Mat, mut obstacle_queue: VecDeque<core::Point2i>) -> Vec<Point2i> {
    println!("Performing Generalized Voronoi Diagram (GVD) algorithm");

    let mut obstacle_label_map = Mat::new_rows_cols_with_default(
        img.rows(),
        img.cols(),
        core::CV_8UC1,
        Scalar::all(UNINITIALIZED_LABEL as f64),
    )
    .expect("failed to create obstacle label map");

    let mut obstacle_queue_copy = obstacle_queue.clone();

    // Map obstacle pixels
    // 0 is Voronoi edges. 1 to 4 are the walls. 5 and up are obstacles. 255 is unexplored.
    // Start indexing at 5
    let mut obstacle_label_idx = 4;

    // Breadth-First Search (BFS) to propagate obstacle labels
    while let Some(pos) = obstacle_queue.pop_front() {
        let obstacle_label = obstacle_label_map.at_2d_mut::<u8>(pos.y, pos.x).unwrap();
        // Assign obstacle label to the neighbor
        match (pos.x, pos.y) {
            (0, _) => {
                *obstacle_label = 0; // Add obstacle label to the top region
            }
            (_, 0) => {
                *obstacle_label = 1; // Add obstacle label to the left region
            }
            _ if pos.x == img.cols() - 1 => {
                *obstacle_label = 2; // Add obstacle label to the right region
            }
            _ if pos.y == img.rows() - 1 => {
                *obstacle_label = 3; // Add obstacle label to the bottom region
            }
            _ => {
                add_label_obstacles(
                    img,
                    &mut obstacle_label_map,
                    pos,
                    &mut obstacle_label_idx,
                    &mut obstacle_queue,
                ); // Add obstacle label to obstacles
            }
        }
    }
    // Now all obstacles have a label
    // Map the Voronoi edges and regions

    // Lable the surrounding pixels for obstacles
    while let Some(pos) = obstacle_queue_copy.pop_front() {
        // Logic
        // Add label to neighbors with the same or higher distance
        // If the neighbor has another label, then this is an edge
        let obstacle_pixel = dtm_img.at_2d::<Vec3b>(pos.y, pos.x).unwrap();
        let obstacle_distance = obstacle_pixel[0] as i32;
        let obstacle_label = *obstacle_label_map.at_2d::<u8>(pos.y, pos.x).unwrap();
        for dir in &DIRECTIONS {
            let neighbor_pos = Point2i::new(pos.x + dir.0, pos.y + dir.1);
            if is_valid_position(img, neighbor_pos) {
                let neighbor_pixel = dtm_img
                    .at_2d::<Vec3b>(neighbor_pos.y, neighbor_pos.x)
                    .unwrap();
                let neighbor_distance = neighbor_pixel[0] as i32;
                if neighbor_distance >= obstacle_distance && neighbor_distance != 0 {
                    let neighbor_label = obstacle_label_map
                        .at_2d_mut::<u8>(neighbor_pos.y, neighbor_pos.x)
                        .unwrap();
                    if *neighbor_label == UNINITIALIZED_LABEL {
                        *neighbor_label = obstacle_label;
                        obstacle_queue_copy.push_back(neighbor_pos);
                    }
                }
            }
        }
    }

    // Add the Voronoi edges between the regions
    let edges = find_voronoi_edges(&obstacle_label_map);
    edges.iter().for_each(|pos| {
        let obstacle_label = obstacle_label_map.at_2d_mut::<u8>(pos.y, pos.x).unwrap();
        *obstacle_label = EDGE_LABEL;
    });

    // Generate the GVD image
    let mut gvd_img = img.clone();
    // Draw the the regions
    let colors = util::generate_random_color(obstacle_label_idx as usize);
    for x in 0..img.cols() {
        for y in 0..img.rows() {
            let obstacle_label = obstacle_label_map.at_2d::<u8>(y, x).unwrap();
            let distance = dtm_img.at_2d::<Vec3b>(y, x).unwrap()[0] as i32;
            let gvd_pixel = gvd_img.at_2d_mut::<Vec3b>(y, x).unwrap();
            if *obstacle_label != UNINITIALIZED_LABEL // Avoid out-of-bounds
                && *obstacle_label != EDGE_LABEL // Keep the Voronoi edges white
                && (distance != 0 || x == 0 || y == 0 || x == img.cols() - 1 || y == img.rows() - 1)
            // Keep the obstacles
            {
                *gvd_pixel = colors[*obstacle_label as usize];
            }
        }
    }

    // Show the GVD image
    named_window(WINDOW_NAME, 0).expect("failed to create Brushfire window");
    move_window(WINDOW_NAME, 0, 515).expect("failed to move window");
    imshow(WINDOW_NAME, &gvd_img).expect("failed to show Brushfire image");
    wait_key(1).expect("failed to wait key");

    edges
}

fn add_label_obstacles(
    img: &Mat,
    obstacle_label_map: &mut Mat,
    pos: Point2i,
    obstacle_label_idx: &mut u8,
    obstacle_queue: &mut VecDeque<Point2i>,
) {
    // Find the label of the neighbor
    // If all the neighbors have the same label or are unexplored, assign a new label
    // If the neighbors have different labels and is obstacle, we need to merge the labels
    let mut neighbor_labels = Vec::new();
    for (dx, dy) in DIRECTIONS {
        let neighbor_pos = Point2i::new(pos.x + dx, pos.y + dy);
        if is_valid_position(img, neighbor_pos) {
            let neighbor_label = obstacle_label_map
                .at_2d::<u8>(neighbor_pos.y, neighbor_pos.x)
                .unwrap();
            if *neighbor_label != UNINITIALIZED_LABEL {
                neighbor_labels.push(*neighbor_label);
            }
        }
    }

    let obstacle_label = obstacle_label_map.at_2d_mut::<u8>(pos.y, pos.x).unwrap();
    if neighbor_labels.is_empty() {
        // Assign a new label
        *obstacle_label = *obstacle_label_idx;
        *obstacle_label_idx += 1;
    } else {
        // Assign the minimum label
        let max_label = *neighbor_labels.iter().max().unwrap();
        let min_label = *neighbor_labels.iter().min().unwrap();
        if max_label != min_label && max_label != UNINITIALIZED_LABEL {
            *obstacle_label = min_label;
            // Merge the labels
            for (dx, dy) in DIRECTIONS {
                let neighbor_pos = Point2i::new(pos.x + dx, pos.y + dy);
                if is_valid_position(img, neighbor_pos) {
                    let neighbor_label = obstacle_label_map
                        .at_2d_mut::<u8>(neighbor_pos.y, neighbor_pos.x)
                        .unwrap();
                    if *neighbor_label != min_label {
                        *neighbor_label = min_label;
                        obstacle_queue.push_back(neighbor_pos);
                    }
                }
            }
        } else {
            *obstacle_label = min_label;
        }
    }
}

fn find_voronoi_edges(obstacle_label_map: &Mat) -> Vec<Point2i> {
    let mut edges = Vec::new();

    for y in 0..obstacle_label_map.rows() {
        for x in 0..obstacle_label_map.cols() {
            let pos = Point2i::new(x, y);
            let current_label = *obstacle_label_map.at_2d::<u8>(y, x).unwrap();

            for dir in &DIRECTIONS {
                let neighbor_pos = Point2i::new(pos.x + dir.0, pos.y + dir.1);
                if is_valid_position(obstacle_label_map, neighbor_pos) {
                    let neighbor_label = *obstacle_label_map
                        .at_2d::<u8>(neighbor_pos.y, neighbor_pos.x)
                        .unwrap();
                    if current_label != neighbor_label {
                        edges.push(pos);
                        break;
                    }
                }
            }
        }
    }

    edges
}
