#![allow(dead_code)]

use anyhow::Result;
use opencv::{
    core::{Point, Scalar},
    highgui, imgproc,
};

use particles::ParticleFilter;
use robot::Robot;

mod map;
mod particles;
mod robot;
mod transform;

const WINDOW_NAME: &str = "Particle Filter";

fn main() -> Result<()> {
    let mut map = map::Map::new_obstacle_map();
    let mut robot = Robot::new(180.0, 100.0, 0.0);
    let robot_readings = Robot::get_lidar_reading(&map, robot.get_transform());

    let mut particles = ParticleFilter::distribute_particles_map(&map, 1000);
    particles.update_weights_from_readings(&robot_readings, &map);

    let robot_transform = robot.get_transform();
    imgproc::circle(
        &mut map.data,
        Point::new(robot_transform.x as i32, robot_transform.y as i32),
        8,
        Scalar::new(0.0, 0.0, 255.0, 255.0),
        -1,
        8,
        0,
    )?;

    for particle in particles.particles.iter() {
        let transform = particle.get_transform();
        let color = particle.get_weight();
        imgproc::circle(
            &mut map.data,
            Point::new(transform.x as i32, transform.y as i32),
            8,
            Scalar::new(0.0, 255.0, 0.0, 255.0),
            -1,
            8,
            0,
        )?;
    }

    highgui::imshow(WINDOW_NAME, &map.data)?;
    highgui::wait_key(0)?;

    Ok(())
}
