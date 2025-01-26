use opencv::core::MatTraitConst;
use rand::{distributions::Uniform, Rng};
use std::f64::consts;

use crate::{
    map::Map,
    robot::{LidarReading, Robot},
    transform::Transform,
};

#[derive(Debug)]
pub struct Particle {
    weight: f64,
    transform: Transform,
}

impl Particle {
    /// Creates a new [`Particle`].
    pub fn new(x: f64, y: f64, theta: f64, weight: f64) -> Self {
        Self {
            weight,
            transform: Transform { x, y, theta },
        }
    }
    /// Creates a new [`Particle`] from a [`Transform`].
    pub fn new_from_transform(transform: Transform, weight: f64) -> Self {
        Self { weight, transform }
    }

    /// Sets the transform of this [`Particle`].
    pub fn set_transform(&mut self, transform: &Transform) {
        self.transform = *transform;
    }
    /// Returns a reference to the get transform of this [`Particle`].
    pub fn get_transform(&self) -> &Transform {
        &self.transform
    }
    /// Sets the weight of this [`Particle`].
    pub fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
    /// Returns the  weight of this [`Particle`].
    pub fn get_weight(&self) -> f64 {
        self.weight
    }
}

#[derive(Debug)]
pub struct ParticleFilter {
    pub particles: Vec<Particle>,
}

impl ParticleFilter {
    pub fn new() -> Self {
        Self { particles: vec![] }
    }

    /// Distributes particles in the map.
    pub fn distribute_particles_map(map: &Map, particle_count: u32) -> Self {
        let mut p_filter = Self::new();
        let weight = 1.0 / particle_count as f64;
        let mut rng = rand::thread_rng();
        let uniform_x = Uniform::new(0, map.get_data().cols());
        let uniform_y = Uniform::new(0, map.get_data().cols());
        let uniform_theta = Uniform::new(0.0, 2.0 * consts::PI);

        let mut count = 0;
        while count < particle_count {
            let x = rng.sample(uniform_x);
            let y = rng.sample(uniform_y);
            if !map.is_occupied(x, y) {
                let theta = rng.sample(uniform_theta);
                let particle = Particle::new(x as f64, y as f64, theta, weight);
                p_filter.particles.push(particle);
                count += 1;
            }
        }
        p_filter
    }
    pub fn apply_transform(&mut self, map: &Map, d_transform: &Transform) {
        let transformed_particles: Vec<Particle> = self
            .particles
            .iter()
            .filter_map(|particle| {
                let mut transform = *particle.get_transform();
                transform += d_transform;
                if map.is_occupied(transform.x as i32, transform.y as i32) {
                    Some(Particle::new_from_transform(
                        transform,
                        particle.get_weight(),
                    ))
                } else {
                    None
                }
            })
            .collect();
        self.particles = transformed_particles;
    }

    fn calculate_weight(lidar_distance: f64, particle_distance: f64) -> f64 {
        let lambda = 0.3; // Decay rate, adjust as needed
        let difference = (lidar_distance - particle_distance).abs();
        (-lambda * difference).exp()
    }

    pub fn update_weights_from_readings(&mut self, robot_readings: &Vec<LidarReading>, map: &Map) {
        for particle in self.particles.iter_mut() {
            let lidar_particle = Robot::get_lidar_reading(map, particle.get_transform());
            // Update the weights using the lidar readings from the robot and particle
            let mut weight = 0.0;
            for (lidar_reading, particle_reading) in lidar_particle.iter().zip(robot_readings) {
                // Calculate the weight using the distance
                // The closer the readings match, the higher the weight
                weight += Self::calculate_weight(lidar_reading.distance, particle_reading.distance);
            }
            particle.set_weight(weight);
        }
        Self::normalize_weights(self);
    }

    fn normalize_weights(&mut self) {
        let sum: f64 = self.particles.iter().map(|p| p.get_weight()).sum();
        for particle in self.particles.iter_mut() {
            particle.set_weight(particle.get_weight() / sum);
        }
    }

    pub fn resample(&mut self) {
        todo!()
    }

    pub fn get_best_estimate(&self) -> Transform {
        let mut max_weight = 0.0;
        let mut best_estimate = Transform::default();
        for particle in self.particles.iter() {
            if particle.get_weight() > max_weight {
                max_weight = particle.get_weight();
                best_estimate = *particle.get_transform();
            }
        }
        best_estimate
    }

    pub fn get_estimated_position(&self) -> Transform {
        let mut sum = Transform::default();
        for particle in self.particles.iter() {
            sum += particle.get_transform();
        }
        sum / self.particles.len() as f64
    }
}
