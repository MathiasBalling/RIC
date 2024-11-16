use std::{
    cell::RefCell,
    cmp::Ordering,
    collections::{BinaryHeap, HashMap},
    rc::Rc,
};

use opencv::{
    core::{Mat, MatTraitConst, Point2f, Point2i, Scalar},
    highgui,
    imgproc::{circle, line},
};

use crate::util::{generate_random_pos, is_obstacle, is_valid_position};

const WINDOW_NAME: &str = "PRM";

#[derive(Debug)]
struct Node {
    position: Point2i,
    connections: Vec<Rc<RefCell<Node>>>,
}

impl Node {
    fn new(position: Point2i) -> Rc<RefCell<Node>> {
        Rc::new(RefCell::new(Node {
            position,
            connections: Vec::new(),
        }))
    }

    fn add_connection(&mut self, neighbor: Rc<RefCell<Node>>) {
        self.connections.push(neighbor);
    }
}

pub fn prm(img: &Mat) {
    println!("Starting PRM algorithm");

    // Clone to draw on
    let mut prm_img = img.clone();

    // Generate samples proportional to the size of the image
    let sample_count = img.rows() * img.cols() / 8000;
    println!("Generating {} samples", sample_count);
    let mut samples = generate_samples(sample_count as u32, img);
    let mut graph = HashMap::<(i32, i32), Rc<RefCell<Node>>>::new();

    let mut key = (0, 0);
    let mut key2 = (0, 0);
    // Initialize the graph with the samples
    for sample in samples.iter() {
        let node = Node::new(*sample);
        graph.insert((sample.x, sample.y), node);
        if key == (0, 0) {
            key = (sample.x, sample.y);
        } else {
            key2 = (sample.x, sample.y);
        }
    }

    // Connect all sample nodes.
    while let Some(sample) = samples.pop() {
        circle(
            &mut prm_img,
            sample,
            2,
            Scalar::new(0.0, 0.0, 255.0, 0.0),
            2,
            8,
            0,
        )
        .expect("Could not draw circle");

        // Find the nearest neighbors
        for sample2 in samples.iter() {
            if has_edge(img, sample, *sample2) {
                let node1 = graph
                    .get(&(sample.x, sample.y))
                    .expect("Could not get node1");
                let node2 = graph
                    .get(&(sample2.x, sample2.y))
                    .expect("Could not get node2");
                node1.borrow_mut().add_connection(Rc::clone(node2));
                node2.borrow_mut().add_connection(Rc::clone(node1));

                // Draw the edge
                draw_edge(&mut prm_img, sample, *sample2);
            }
        }
    }

    // Find the path
    let path = find_path(img, key.into(), key2.into(), graph);
    println!("Path: {:?}", path);
    // Draw the path
    for i in 0..path.len() - 1 {
        draw_path(&mut prm_img, path[i].into(), path[i + 1].into());
    }

    // for (key, value) in &graph {
    //     println!("Node at {:?} has connections:", key);
    //     let connections = &value.borrow().connections;
    //     for connection in connections.iter() {
    //         println!("Connection: {:?}", connection.borrow().position);
    //     }
    // }

    highgui::named_window(WINDOW_NAME, 0).expect("Could not create window");
    highgui::move_window(WINDOW_NAME, 650, 0).expect("failed to move window");
    highgui::imshow(WINDOW_NAME, &prm_img).expect("Could not show image");
    highgui::wait_key(1).expect("Could not close window");
}

fn has_edge(img: &Mat, pt1: Point2i, pt2: Point2i) -> bool {
    if !is_valid_position(img, pt1) || !is_valid_position(img, pt2) {
        return false;
    }

    // Check the direction
    let dir = pt2 - pt1;
    let nomalized_dir = {
        let temp = Point2f {
            x: dir.x as f32,
            y: dir.y as f32,
        };
        temp / temp.norm() as f32
    };

    let mut pos = Point2f {
        x: pt1.x as f32,
        y: pt1.y as f32,
    };
    let end_pos = Point2f {
        x: pt2.x as f32,
        y: pt2.y as f32,
    };
    // Check if the edge between pt1 and pt2 is obstructed
    while (end_pos - pos).norm() > 1.0 {
        if is_obstacle(
            img,
            Point2i {
                x: pos.x as i32,
                y: pos.y as i32,
            },
        ) {
            return false;
        }
        pos += nomalized_dir;
    }

    true
}

fn draw_edge(img: &mut Mat, pt1: Point2i, pt2: Point2i) {
    // Draw a line between the two points
    let color = Scalar::new(0.0, 255.0, 0.0, 0.0);
    line(img, pt1, pt2, color, 1, 8, 0).expect("Could not draw line");
}

fn draw_path(img: &mut Mat, pt1: Point2i, pt2: Point2i) {
    // Draw a line between the two points
    let color = Scalar::new(255.0, 0.0, 0.0, 0.0);
    line(img, pt1, pt2, color, 1, 8, 0).expect("Could not draw line");
}

#[derive(Debug, Eq, PartialEq)]
struct State {
    cost: i32,
    position: (i32, i32),
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn find_path(
    img: &Mat,
    pt1: Point2i,
    pt2: Point2i,
    graph: HashMap<(i32, i32), Rc<RefCell<Node>>>,
) -> Vec<(i32, i32)> {
    let start = (pt1.x, pt1.y);
    let goal = (pt2.x, pt2.y);

    let mut dist: HashMap<(i32, i32), i32> = HashMap::new();
    let mut prev: HashMap<(i32, i32), Option<(i32, i32)>> = HashMap::new();
    let mut heap = BinaryHeap::new();

    for node in graph.keys() {
        dist.insert(*node, i32::MAX);
        prev.insert(*node, None);
    }

    dist.insert(start, 0);
    heap.push(State {
        cost: 0,
        position: start,
    });

    while let Some(State { cost, position }) = heap.pop() {
        if position == goal {
            let mut path = Vec::new();
            let mut current = goal;
            while let Some(prev_node) = prev[&current] {
                path.push(current);
                current = prev_node;
            }
            path.push(start);
            path.reverse();
            return path;
        }

        if cost > dist[&position] {
            continue;
        }

        if let Some(node) = graph.get(&position) {
            let connections = &node.borrow().connections;
            for node in connections {
                let neighbor = node.borrow().position;
                let next = (neighbor.x, neighbor.y);
                let edge_cost = (((position.0 - next.0).pow(2) + (position.1 - next.1).pow(2))
                    as f64)
                    .sqrt() as i32;

                let next_cost = cost + edge_cost;

                if next_cost < dist[&next] {
                    heap.push(State {
                        cost: next_cost,
                        position: next,
                    });
                    dist.insert(next, next_cost);
                    prev.insert(next, Some(position));
                }
            }
        }
    }

    Vec::new() // Return an empty path if no path is found
}

fn generate_samples(count: u32, img: &Mat) -> Vec<Point2i> {
    let mut samples = vec![];
    let (row_max, col_max) = (img.rows(), img.cols());
    while samples.len() < count as usize {
        let pos = generate_random_pos(row_max, col_max);

        if !is_obstacle(img, pos) {
            samples.push(pos);
        }
    }
    samples
}
