#![allow(dead_code)]

use anyhow::Result;
use opencv::{
    core::{Point, Scalar},
    highgui,
    imgproc::circle,
};
use particles::ParticleFilter;

mod map;
mod particles;
mod robot;
mod util;

const WINDOW_NAME: &str = "Particle Filter";

fn main() -> Result<()> {
    let mut map = map::Map::new_obstacle_map();
    let particles = ParticleFilter::distribute_particles_map(&map, 10);

    for particle in particles.particles.iter() {
        let transform = particle.get_transform();
        circle(
            &mut map.data,
            Point::new(transform.x as i32, transform.y as i32),
            2,
            Scalar::new(255.0, 0.0, 0.0, 255.0),
            2,
            8,
            0,
        )?;
    }

    highgui::imshow(WINDOW_NAME, &map.data)?;
    highgui::wait_key(0)?;

    Ok(())
}
