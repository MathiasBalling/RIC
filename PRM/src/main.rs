// Implement PRM for a 2D planning problem.
//      You can assume the robot to be a point to reduce the complexity.
//      Use for instance the same map as before, e.g., for Bug.
//
// Requirements:
//      1. Verify that the algorithm works for a number of cases (you can use the
//         same map and build different roadmaps and use the same set of start/goals, e.g. 5)
//      2. Report e.g. success, path length and other aspects you find relevant
//      3. Discuss some of your choices, e.g. sampling strategy or connection strategy and investigate the impact (might be qualitative)
//      4. Optional: check for 'difficult' nodes and extend the roadmap
use opencv::{highgui, Result};
use util::make_map;

mod prm;
mod util;

const WINDOW_NAME: &str = "Original";

fn main() -> Result<()> {
    let img = make_map();

    // Show the original image
    highgui::named_window(WINDOW_NAME, 0)?;
    highgui::imshow(WINDOW_NAME, &img)?;
    highgui::wait_key(1)?;

    // Run PRM algorithm with planning
    prm::prm(&img);

    // Wait for a key press
    println!("Press 'q' to quit");
    while highgui::wait_key(0)? != 'q' as i32 {}

    // Clean up
    highgui::destroy_all_windows()?;
    Ok(())
}
