use opencv::{
    highgui::{self, imshow, named_window, wait_key},
    Result,
};
use util::make_map;

mod brushfire;
mod gvd;
mod path_panner;
mod util;

const WINDOW_NAME: &str = "Original";

fn main() -> Result<()> {
    let img = make_map();

    // Show the original image
    named_window(WINDOW_NAME, 0)?;
    imshow(WINDOW_NAME, &img)?;
    wait_key(1)?;

    // Perform brushfire algorithm
    let (distance_map, obstacle_queue) = brushfire::brushfire(&img);

    // Perform Generalized Voronoi Diagram (GVD) algorithm
    let verionoi_edges = gvd::gvd(&img, &distance_map, obstacle_queue);

    // Perform path planning algorithm
    path_panner::path_panner(&img, verionoi_edges);

    // Wait for a key press
    println!("Press 'q' to quit");
    while wait_key(0)? != 'q' as i32 {}

    // Clean up
    highgui::destroy_all_windows()?;
    Ok(())
}
