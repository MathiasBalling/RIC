use opencv::core::MatTraitConst;
use rand::{distributions::Uniform, Rng};
use std::f64::consts;

use crate::{map::Map, util::Transform};

#[derive(Debug)]
pub struct Particle {
    weight: f64,
    transform: Transform,
}

impl Particle {
    pub fn new(x: f64, y: f64, theta: f64, weight: f64) -> Self {
        Self {
            weight,
            transform: Transform { x, y, theta },
        }
    }
    pub fn set_transform(&mut self, x: f64, y: f64, theta: f64) {
        self.transform = Transform { x, y, theta };
    }
    pub fn get_transform(&self) -> Transform {
        self.transform
    }
    pub fn set_weight(&mut self, weight: f64) {
        self.weight = weight;
    }
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
    pub fn apply_transform(&mut self, d_transform: Transform) {
        for particle in self.particles.iter_mut() {
            let transform = particle.get_transform();
            particle.set_transform(
                transform.x + d_transform.x,
                transform.y + d_transform.y,
                transform.theta + d_transform.theta,
            );
        }
    }

    pub fn resample(&mut self) {}
}
